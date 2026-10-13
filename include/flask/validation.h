#pragma once
#include "flask/model.h"
#include <algorithm>

namespace flask {
struct ValidationConfig {
    double step=0.02;
    std::size_t max_samples=20000;
    bool simd=true;
    bool continuous_sphere_collision=false;
    int subdivision_depth=16;
    bool enforce_physical_continuity=true;
    bool enforce_physical_endpoints=true;
    double junction_tolerance=1e-6;
};
struct ValidationStats {
    std::size_t edges=0,samples=0,batches=0,collision_rejections=0,physical_rejections=0,
                certificate_rejections=0;
};
template<int N,int R> class Validator {
public:
    using State=typename Model<N,R>::State;
    using Curve=typename Model<N,R>::Curve;
    Validator(const Model<N,R> &model,ValidationConfig config={}):model_(model),config_(config) {
        if(!std::isfinite(config.step) || config.step<=0 || config.max_samples<2 ||
           config.subdivision_depth<0 || config.subdivision_depth>24 ||
           !std::isfinite(config.junction_tolerance)||config.junction_tolerance<=0)
            throw std::invalid_argument("Invalid trajectory validation configuration");
    }
    const ValidationConfig &config() const { return config_; }
    ValidationStats stats;
    bool edge(const Curve &curve) {
        ++stats.edges;
        if(!std::isfinite(curve.T)||curve.T<=0||!curve.coefficients.allFinite()) return false;
        if(!model_.intervalValid(curve)) { ++stats.physical_rejections; return false; }
        const double count=std::ceil(curve.T/config_.step);
        if(!std::isfinite(count)||count+1>config_.max_samples) return false;
        const auto intervals=static_cast<std::size_t>(count);
        const std::size_t batches=(intervals+1+7)/8;
        for(std::size_t b=0;b<batches;++b) {
            PositionBatch batch;
            for(std::size_t lane=0;lane<8;++lane) {
                const auto i=b+lane*batches; // Algorithm 3 distributed lanes.
                if(i>intervals) continue;
                PhysicalSample point; ++stats.samples;
                if(!model_.sample(curve,curve.T*double(i)/intervals,point)) {
                    ++stats.physical_rejections; return false;
                }
                batch.active|=1u<<lane; batch.position[lane]=point.position;
            }
            ++stats.batches;
            if(collisionMask(batch,model_.config().obstacles,model_.config().radius,config_.simd)) {
                ++stats.collision_rejections; return false;
            }
        }
        if(config_.continuous_sphere_collision && !certify(bernstein(curve),config_.subdivision_depth)) {
            ++stats.certificate_rejections; return false;
        }
        return true;
    }
    bool path(const std::vector<Curve> &curves,const State &start,const State &goal,bool requireGoalPhysical=true) {
        if(curves.empty()) {
            if((start.values-goal.values).norm()>=1e-10 || !model_.nodeValid(start)) return false;
            const auto &c=model_.config();
            return !c.physical_start.size() || !c.physical_goal.size() ||
                model_.physicalError(c.physical_start,c.physical_goal)<=config_.junction_tolerance;
        }
        if((curves.front().evaluate(0).values-start.values).norm()>1e-6 ||
           (curves.back().evaluate(curves.back().T).values-goal.values).norm()>1e-6) return false;
        PhysicalSample first,last,previous;
        if(!model_.sample(curves.front(),0,first)) return false;
        if(config_.enforce_physical_endpoints && model_.config().physical_start.size() &&
           model_.physicalError(first.state,model_.config().physical_start)>config_.junction_tolerance) return false;
        for(std::size_t i=0;i<curves.size();++i) {
            if(i && (curves[i-1].evaluate(curves[i-1].T).values-curves[i].evaluate(0).values).norm()>1e-6)
                return false;
            if(!edge(curves[i]) || !model_.sample(curves[i],0,first) ||
               !model_.sample(curves[i],curves[i].T,last)) return false;
            if(i && config_.enforce_physical_continuity &&
               model_.physicalError(previous.state,first.state)>config_.junction_tolerance) return false;
            previous=last;
        }
        return !config_.enforce_physical_endpoints || !requireGoalPhysical || !model_.config().physical_goal.size() ||
               model_.physicalError(last.state,model_.config().physical_goal)<=config_.junction_tolerance;
    }
private:
    using Polygon=std::vector<Eigen::Vector3d>;
    Polygon bernstein(const Curve &curve) const {
        const int degree=Curve::Degree;
        Polygon controls(degree+1,Eigen::Vector3d::Zero());
        for(int k=0;k<=degree;++k) for(int i=0;i<=k;++i)
            controls[k].head(model_.config().spatial_dim)+=
                curve.coefficients.col(i).head(model_.config().spatial_dim)*(curve.normalized ? 1:std::pow(curve.T,i))*
                rrtpff::binomial(k,i)/rrtpff::binomial(degree,i);
        return controls;
    }
    bool certify(const Polygon &controls,int depth) const {
        Eigen::Vector3d lo=controls.front(),hi=lo;
        for(const auto &p:controls) { lo=lo.cwiseMin(p); hi=hi.cwiseMax(p); }
        // Inflate the hull to account for round-off; this is conservative
        // geometry subdivision, not a formal all-constraints certificate.
        const double padding=1e-10*(1+lo.norm()+hi.norm());
        lo.array()-=padding; hi.array()+=padding;
        bool separated=true;
        for(const auto &s:model_.config().obstacles) {
            const auto closest=s.center.cwiseMax(lo).cwiseMin(hi);
            const double radius=s.radius+model_.config().radius;
            if((closest-s.center).squaredNorm()<=radius*radius) { separated=false; break; }
        }
        if(separated) return true;
        if(depth==0) return false; // Uncertified means reject, never accept.
        auto temp=controls;
        Polygon left(controls.size()),right(controls.size());
        left.front()=temp.front(); right.back()=temp.back();
        for(std::size_t k=1;k<controls.size();++k) {
            for(std::size_t j=0;j<controls.size()-k;++j) temp[j]=(temp[j]+temp[j+1])/2;
            left[k]=temp[0]; right[controls.size()-1-k]=temp[controls.size()-1-k];
        }
        return certify(left,depth-1) && certify(right,depth-1);
    }
    const Model<N,R> &model_;
    ValidationConfig config_;
};
} // namespace flask
