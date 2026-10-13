#pragma once
#include "flask/validation.h"
#include <chrono>
#include <random>
#include <numeric>

namespace flask {
template<int N,int R> Eigen::VectorXd metricScale(const Model<N,R> &model) {
    Eigen::VectorXd scale(N*R);
    for(int d=0;d<R;++d) {
        const double extent=(model.config().upper.segment(d*N,N)-model.config().lower.segment(d*N,N)).norm();
        scale.segment(d*N,N).setConstant(extent>1e-12 ? extent:1);
    }
    return scale;
}
template<int N,int R> double flatMetric(const typename BVP<N,R>::State &a,
    const typename BVP<N,R>::State &b,const Eigen::VectorXd &scale) {
    double sum=0;
    for(int d=0;d<R;++d)
        sum+=((a.values.template segment<N>(d*N)-b.values.template segment<N>(d*N)).array()/
              scale.segment(d*N,N).array()).matrix().norm();
    return sum;
}
template<int N,int R> std::optional<typename BVP<N,R>::State> interpolateFlat(
    const BVP<N,R> &bvp,const typename BVP<N,R>::State &from,
    const typename BVP<N,R>::State &to,double fraction) {
    if(!std::isfinite(fraction) || fraction<0 || fraction>1)
        throw std::invalid_argument("Interpolation fraction outside [0,1]");
    if(fraction==0 || (from.values-to.values).norm()<1e-12) return from;
    if(fraction==1) return to;
    const auto motion=bvp.optimal(from,to);
    if(!motion) return {};
    return motion->evaluate(fraction*motion->T);
}
template<int N,int R> std::optional<typename BVP<N,R>::State> interpolateForTree(
    const BVP<N,R> &bvp,const typename BVP<N,R>::State &parent,
    const typename BVP<N,R>::State &target,double fraction,bool forward) {
    // A goal-tree edge must be a suffix of target -> parent, not a
    // prefix of parent -> target. Do not integrate with negative time.
    return forward ? interpolateFlat(bvp,parent,target,fraction):
                     interpolateFlat(bvp,target,parent,1-fraction);
}
struct PlannerConfig {
    std::uint32_t seed=41;
    std::size_t iterations=10000,connect_steps=64,max_nodes=20000;
    double timeout=5,range=1,goal_bias=0.05,goal_tolerance=0.1;
    double unicycle_sample_speed=0;
    Eigen::VectorXd metric_scale;
    bool shortcut=true;
    std::size_t shortcut_attempts=100;
};
template<int N,int R> struct Plan {
    bool success=false;
    std::string reason;
    std::vector<typename BVP<N,R>::Curve> curves;
    std::size_t iterations=0,nodes=0,bvp_queries=0,shortcuts=0;
    double seconds=0,search_seconds=0,cost_before_shortcut=0;
    double cost() const { double sum=0; for(const auto &c:curves) sum+=c.cost; return sum; }
    double duration() const { double sum=0; for(const auto &c:curves) sum+=c.T; return sum; }
    // Composite Simpson integration of translational speed, excluding yaw.
    double length(int spatial_dim) const {
        double sum=0;
        for(const auto &c:curves) {
            const int intervals=std::max(100,2*int(std::ceil(c.T/.002)));
            double integral=0;
            for(int i=0;i<=intervals;++i) {
                const double speed=c.derivative(c.T*i/intervals,1).head(spatial_dim).norm();
                integral+=(i==0 || i==intervals ? 1:(i%2 ? 4:2))*speed;
            }
            sum+=integral*c.T/(3*intervals);
        }
        return sum;
    }
};
template<int N,int R> class RRTConnect {
public:
    using State=typename BVP<N,R>::State;
    using Curve=typename BVP<N,R>::Curve;
    using Clock=std::chrono::steady_clock;
    RRTConnect(const Model<N,R> &model,const BVP<N,R> &bvp,Validator<N,R> &validator,
               PlannerConfig config={}):model_(model),bvp_(bvp),validator_(validator),config_(std::move(config)) {
        if(config_.metric_scale.size()==0) config_.metric_scale=metricScale(model_);
        if(config_.metric_scale.size()!=N*R || !config_.metric_scale.allFinite() ||
           (config_.metric_scale.array()<=0).any() || !std::isfinite(config_.range) || config_.range<=0 ||
           !std::isfinite(config_.timeout) || config_.timeout<=0 || config_.iterations==0 ||
           config_.max_nodes<2 || config_.connect_steps==0 || !std::isfinite(config_.goal_bias) ||
           config_.goal_bias<0 || config_.goal_bias>1)
            throw std::invalid_argument("Invalid RRTConnect configuration");
        if(!std::isfinite(config_.unicycle_sample_speed) || config_.unicycle_sample_speed<0 ||
           (config_.unicycle_sample_speed>0 && (model_.config().robot!=Robot::Unicycle ||
            config_.unicycle_sample_speed>model_.config().unicycle.max_velocity)))
            throw std::invalid_argument("Invalid Unicycle pose lifting speed");
    }
    Plan<N,R> solve(const State &start,const State &goal) {
        const auto begin=Clock::now();
        deadline_=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.timeout));
        Plan<N,R> result;
        auto finish=[&](std::string reason) {
            result.reason=std::move(reason); result.seconds=std::chrono::duration<double>(Clock::now()-begin).count();
            if(result.search_seconds==0) result.search_seconds=result.seconds;
            return result;
        };
        if(!model_.nodeValid(start)||!model_.nodeValid(goal)) return finish("invalid_endpoint");
        if((start.values-goal.values).norm()<1e-12) {
            result.success=validator_.path({},start,goal);
            return finish(result.success ? "already_at_goal":"physical_boundary_mismatch");
        }
        ++result.bvp_queries;
        auto direct=bvp_.optimal(start,goal);
        if(direct && validator_.path({*direct},start,goal)) {
            result.success=true; result.curves={*direct}; result.nodes=2;
            result.cost_before_shortcut=result.cost(); return finish("direct_bvp");
        }
        std::vector<Node> from{{start,0,{}}},to{{goal,0,{}}};
        std::mt19937 rng(config_.seed); std::uniform_real_distribution<double> unit(0,1);
        for(std::size_t iteration=0;iteration<config_.iterations && !expired();++iteration) {
            result.iterations=iteration+1;
            if(from.size()+to.size()>=config_.max_nodes) return finish("node_limit");
            const bool forward=iteration%2==0;
            auto &active=forward ? from:to; auto &other=forward ? to:from;
            State target;
            if(unit(rng)<config_.goal_bias) target=forward ? goal:start;
            else {
                for(int k=0;k<N*R;++k) target.values(k)=model_.config().lower(k)+
                    unit(rng)*(model_.config().upper(k)-model_.config().lower(k));
                if constexpr(N==2 && R>=2) {
                    if(config_.unicycle_sample_speed>0) {
                        // Original Unicycle state is (x,y,theta); speed is
                        // an explicitly chosen lifting control, not a state.
                        const double heading=(2*unit(rng)-1)*3.14159265358979323846;
                        target.values(2)=config_.unicycle_sample_speed*std::cos(heading);
                        target.values(3)=config_.unicycle_sample_speed*std::sin(heading);
                    }
                }
            }
            // A sampled target may be invalid while the range-limited edge
            // toward it is valid. RRTConnect checks the actual new state and
            // motion, not the raw uniform sample (public OMPL growTree).
            const auto grown=extend(active,target,forward,result);
            if(grown.status==Trapped) continue;
            const State meet=active[grown.index].state;
            Extension joined;
            for(std::size_t k=0;k<config_.connect_steps && !expired();++k) {
                if(from.size()+to.size()>=config_.max_nodes) break;
                joined=extend(other,meet,!forward,result);
                if(joined.status!=Advanced) break;
            }
            if(joined.status!=Reached) continue;
            const auto si=forward ? grown.index:joined.index;
            const auto gi=forward ? joined.index:grown.index;
            auto path=extract(from,to,si,gi);
            if(!validator_.path(path,start,goal)) continue;
            result.success=true; result.curves=std::move(path); result.nodes=from.size()+to.size();
            result.cost_before_shortcut=result.cost();
            result.search_seconds=std::chrono::duration<double>(Clock::now()-begin).count();
            if(config_.shortcut) shorten(result,start,goal);
            return finish("connected");
        }
        result.nodes=from.size()+to.size();
        return finish(expired() ? "timeout":"iteration_limit");
    }
    void shorten(Plan<N,R> &plan,const State &start,const State &goal) {
        if(!plan.success || plan.curves.size()<2) return;
        if(deadline_==Clock::time_point::max()) deadline_=Clock::now()+
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.timeout));
        std::size_t attempts=0;
        for(std::size_t i=0;i<plan.curves.size() && !expired();++i) {
            for(std::size_t j=plan.curves.size();j>i+1 && !expired();--j) {
                if(++attempts>config_.shortcut_attempts) return;
                ++plan.bvp_queries;
                const auto a=plan.curves[i].evaluate(0),b=plan.curves[j-1].evaluate(plan.curves[j-1].T);
                auto candidate=bvp_.optimal(a,b); if(!candidate) continue;
                double old=0; for(std::size_t k=i;k<j;++k) old+=plan.curves[k].cost;
                if(candidate->cost>=old-1e-9) continue;
                auto proposed=plan.curves;
                proposed.erase(proposed.begin()+i,proposed.begin()+j);
                proposed.insert(proposed.begin()+i,*candidate);
                if(!validator_.path(proposed,start,goal)) continue;
                plan.curves=std::move(proposed); ++plan.shortcuts; break;
            }
        }
    }
private:
    struct Node { State state; std::size_t parent; std::optional<Curve> edge; };
    enum Status { Trapped,Advanced,Reached };
    struct Extension { Status status=Trapped; std::size_t index=0; };
    double distance(const State &a,const State &b) const {
        return flatMetric<N,R>(a,b,config_.metric_scale);
    }
    bool expired() const { return Clock::now()>=deadline_; }
    Extension extend(std::vector<Node> &tree,const State &target,bool forward,Plan<N,R> &result) {
        if(expired()) return {};
        std::size_t best=0; double nearest=INFINITY;
        for(std::size_t i=0;i<tree.size();++i) {
            const double d=distance(tree[i].state,target); if(d<nearest) { best=i; nearest=d; }
        }
        if(nearest<1e-8) return {Reached,best};
        State endpoint=target;
        if(nearest>config_.range) {
            // Kinodynamic adaptation: outgoing prefix at start, incoming
            // suffix at goal. The public geometric port interpolates
            // nearest -> sample in both trees; that is not used here.
            ++result.bvp_queries;
            const auto limited=interpolateForTree(bvp_,tree[best].state,target,config_.range/nearest,forward);
            if(!limited) return {};
            endpoint=*limited;
            if(distance(tree[best].state,endpoint)<1e-8) return {};
        }
        if(!model_.nodeValid(endpoint)) return {};
        ++result.bvp_queries;
        // Goal-tree curves run new -> parent -> goal, all in positive time.
        auto curve=forward ? bvp_.optimal(tree[best].state,endpoint):bvp_.optimal(endpoint,tree[best].state);
        if(!curve || !validator_.edge(*curve)) return {};
        tree.push_back({endpoint,best,std::move(curve)});
        return {nearest<=config_.range ? Reached:Advanced,tree.size()-1};
    }
    static std::vector<Curve> extract(const std::vector<Node> &from,const std::vector<Node> &to,
                                     std::size_t si,std::size_t gi) {
        std::vector<Curve> path;
        while(si) { path.push_back(*from[si].edge); si=from[si].parent; }
        std::reverse(path.begin(),path.end());
        while(gi) { path.push_back(*to[gi].edge); gi=to[gi].parent; }
        return path;
    }
    const Model<N,R> &model_;
    const BVP<N,R> &bvp_;
    Validator<N,R> &validator_;
    PlannerConfig config_;
    Clock::time_point deadline_=Clock::time_point::max();
};
} // namespace flask
