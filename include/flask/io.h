#pragma once
#include "flask/planner.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
namespace flask {
struct Scene {
    int n=2,r=2;
    std::string robot_name="integrator",planner_name="rrtconnect",source,configuration;
    Eigen::VectorXd start,goal;
    Eigen::MatrixXd weight;
    double rho=1,output_step=.01;
    ModelConfig model;
    PlannerConfig planner;
    ValidationConfig validation;
};
Scene loadScene(const std::filesystem::path &);
void emitVector(YAML::Emitter &,const Eigen::VectorXd &);
template<int N,int R> int runScene(const Scene &scene,const std::filesystem::path &output) {
    using State=typename BVP<N,R>::State;
    if(scene.start.size()!=N*R || scene.goal.size()!=N*R) throw std::invalid_argument("Endpoint dimension mismatch");
    State start,goal; start.values=scene.start; goal.values=scene.goal;
    Model<N,R> model(scene.model);
    BVP<N,R> bvp(scene.weight,scene.rho);
    Validator<N,R> validator(model,scene.validation);
    if(scene.planner_name!="rrtconnect") throw std::invalid_argument("This executable implements FLASK-RRTConnect only");
    RRTConnect<N,R> planner(model,bvp,validator,scene.planner);
    const auto plan=planner.solve(start,goal);
    const double length=plan.length(scene.model.spatial_dim);
    double start_error=0,goal_error=0,junction_error=0;
    if(!plan.curves.empty()) {
        PhysicalSample first,last,previous;
        for(std::size_t i=0;i<plan.curves.size();++i) {
            const auto &c=plan.curves[i];
            if(!model.sample(c,0,first) || !model.sample(c,c.T,last))
                throw std::runtime_error("Invalid physical endpoint during diagnostics");
            if(i) junction_error=std::max(junction_error,model.physicalError(previous.state,first.state));
            else if(scene.model.physical_start.size()) start_error=model.physicalError(first.state,scene.model.physical_start);
            previous=last;
        }
        if(scene.model.physical_goal.size()) goal_error=model.physicalError(last.state,scene.model.physical_goal);
    }
    // Export an unsuccessful run as such; never replace it with a straight
    // segment or silently fall back to another algorithm.
    YAML::Emitter out; out.SetDoublePrecision(17);
    out<<YAML::BeginMap<<YAML::Key<<"success"<<YAML::Value<<plan.success
       <<YAML::Key<<"reason"<<YAML::Value<<plan.reason
       <<YAML::Key<<"source"<<YAML::Value<<scene.source
       <<YAML::Key<<"input_configuration"<<YAML::Value<<YAML::Load(scene.configuration)
       <<YAML::Key<<"robot"<<YAML::Value<<scene.robot_name
       <<YAML::Key<<"planner"<<YAML::Value<<scene.planner_name
       <<YAML::Key<<"seed"<<YAML::Value<<scene.planner.seed
       <<YAML::Key<<"n"<<YAML::Value<<N<<YAML::Key<<"r"<<YAML::Value<<R
       <<YAML::Key<<"rho"<<YAML::Value<<scene.rho
       <<YAML::Key<<"asymptotic_optimality_verified"<<YAML::Value<<false
       <<YAML::Key<<"continuous_control_certificate"<<YAML::Value<<false
       <<YAML::Key<<"continuous_sphere_check"<<YAML::Value<<scene.validation.continuous_sphere_collision
       <<YAML::Key<<"physical_continuity_enforced"<<YAML::Value<<scene.validation.enforce_physical_continuity
       <<YAML::Key<<"physical_endpoints_enforced"<<YAML::Value<<scene.validation.enforce_physical_endpoints
       <<YAML::Key<<"physical_start_error"<<YAML::Value<<start_error
       <<YAML::Key<<"physical_goal_error"<<YAML::Value<<goal_error
       <<YAML::Key<<"max_physical_junction_error"<<YAML::Value<<junction_error
       <<YAML::Key<<"dynobench_checker_verified"<<YAML::Value<<false
       <<YAML::Key<<"reduced_order_result"<<YAML::Value<<((scene.model.robot==Robot::Quad2D || scene.model.robot==Robot::Quad3D)&&R==2)
       <<YAML::Key<<"avx2_available"<<YAML::Value<<avx2Available()
       <<YAML::Key<<"sampling_step"<<YAML::Value<<scene.validation.step
       <<YAML::Key<<"goal_region_radius"<<YAML::Value<<0
       <<YAML::Key<<"iterations"<<YAML::Value<<plan.iterations
       <<YAML::Key<<"nodes"<<YAML::Value<<plan.nodes
       <<YAML::Key<<"bvp_queries"<<YAML::Value<<plan.bvp_queries
       <<YAML::Key<<"shortcuts"<<YAML::Value<<plan.shortcuts
       <<YAML::Key<<"planning_seconds"<<YAML::Value<<plan.seconds
       <<YAML::Key<<"search_seconds"<<YAML::Value<<plan.search_seconds
       <<YAML::Key<<"postprocessing_seconds"<<YAML::Value<<std::max(0.,plan.seconds-plan.search_seconds)
       <<YAML::Key<<"trajectory_length_m"<<YAML::Value;
    if(plan.success) out<<length; else out<<YAML::Null;
    out
       <<YAML::Key<<"cost"<<YAML::Value;
    if(plan.success) out<<plan.cost(); else out<<YAML::Null;
    out<<YAML::Key<<"cost_before_shortcut"<<YAML::Value;
    if(plan.success) out<<plan.cost_before_shortcut; else out<<YAML::Null;
    out<<YAML::Key<<"duration"<<YAML::Value;
    if(plan.success) out<<plan.duration(); else out<<YAML::Null;
    out<<YAML::Key<<"flat_start"<<YAML::Value; emitVector(out,start.values);
    out<<YAML::Key<<"flat_goal"<<YAML::Value; emitVector(out,goal.values);
    out<<YAML::Key<<"segments"<<YAML::Value<<YAML::BeginSeq;
    for(const auto &curve:plan.curves) {
        out<<YAML::BeginMap<<YAML::Key<<"duration"<<YAML::Value<<curve.T
           <<YAML::Key<<"cost"<<YAML::Value<<curve.cost
           <<YAML::Key<<"power_coefficients"<<YAML::Value<<YAML::BeginSeq;
        for(int axis=0;axis<N;++axis) { emitVector(out,curve.coefficients.row(axis).transpose()); }
        out<<YAML::EndSeq<<YAML::EndMap;
    }
    out<<YAML::EndSeq<<YAML::Key<<"samples"<<YAML::Value<<YAML::BeginSeq;
    double elapsed=0;
    for(std::size_t c=0;c<plan.curves.size();++c) {
        const auto &curve=plan.curves[c];
        const auto intervals=static_cast<std::size_t>(std::ceil(curve.T/scene.validation.step));
        for(std::size_t i=0;i<=intervals;++i) {
            if(c && i==0) continue;
            const double t=curve.T*double(i)/intervals;
            PhysicalSample point;
            if(!model.sample(curve,t,point)) throw std::runtime_error("Export encountered an invalid physical sample");
            out<<YAML::BeginMap<<YAML::Key<<"time"<<YAML::Value<<elapsed+t
               <<YAML::Key<<"state"<<YAML::Value; emitVector(out,point.state);
            out<<YAML::Key<<"control"<<YAML::Value; emitVector(out,point.control);
            out<<YAML::EndMap;
        }
        elapsed+=curve.T;
    }
    out<<YAML::EndSeq<<YAML::Key<<"validation"<<YAML::Value<<YAML::BeginMap
       <<YAML::Key<<"edges"<<YAML::Value<<validator.stats.edges
       <<YAML::Key<<"samples"<<YAML::Value<<validator.stats.samples
       <<YAML::Key<<"batches"<<YAML::Value<<validator.stats.batches
       <<YAML::Key<<"collision_rejections"<<YAML::Value<<validator.stats.collision_rejections
       <<YAML::Key<<"physical_rejections"<<YAML::Value<<validator.stats.physical_rejections
       <<YAML::Key<<"certificate_rejections"<<YAML::Value<<validator.stats.certificate_rejections
       <<YAML::EndMap<<YAML::EndMap;
    if(!out.good()) throw std::runtime_error(out.GetLastError());
    std::ofstream file(output);
    if(!file) throw std::runtime_error("Cannot open result file: "+output.string());
    file<<out.c_str()<<'\n'; file.close();
    if(!file) throw std::runtime_error("Result write failed");
    // Native states/actions layout for the DynoBench visualizers. This is a
    // resampling adapter, not a successful zero-order-hold dynamics check.
    if(plan.success && !plan.curves.empty()) {
        YAML::Emitter native; native.SetDoublePrecision(17);
        const auto intervals=std::size_t(std::ceil(plan.duration()/scene.output_step));
        if(intervals>1000000) throw std::runtime_error("Native output sample budget exceeded");
        std::vector<PhysicalSample> points;
        std::size_t segment=0; double offset=0;
        for(std::size_t i=0;i<=intervals;++i) {
            const double t=std::min(double(i)*scene.output_step,plan.duration());
            while(segment+1<plan.curves.size() && t>offset+plan.curves[segment].T) {
                offset+=plan.curves[segment].T; ++segment;
            }
            PhysicalSample point;
            if(!model.sample(plan.curves[segment],std::clamp(t-offset,0.,plan.curves[segment].T),point))
                throw std::runtime_error("Native output encountered an invalid physical sample");
            points.push_back(std::move(point));
        }
        native<<YAML::BeginMap<<YAML::Key<<"dt"<<YAML::Value<<scene.output_step
              <<YAML::Key<<"last_step_duration"<<YAML::Value<<plan.duration()-(intervals-1)*scene.output_step
              <<YAML::Key<<"dynobench_checker_verified"<<YAML::Value<<false
              <<YAML::Key<<"result"<<YAML::Value<<YAML::BeginSeq<<YAML::BeginMap
              <<YAML::Key<<"states"<<YAML::Value<<YAML::BeginSeq;
        for(const auto &point:points) emitVector(native,point.state);
        native<<YAML::EndSeq<<YAML::Key<<"actions"<<YAML::Value<<YAML::BeginSeq;
        for(std::size_t i=0;i+1<points.size();++i) emitVector(native,points[i].control);
        native<<YAML::EndSeq<<YAML::EndMap<<YAML::EndSeq<<YAML::EndMap;
        auto native_path=output; native_path.replace_extension(".dynobench.yaml");
        std::ofstream native_file(native_path); native_file<<native.c_str()<<'\n'; native_file.close();
        if(!native.good() || !native_file) throw std::runtime_error("Native result write failed");
    }
    std::cout<<std::setprecision(17)<<(plan.success ? "SUCCESS ":"FAILED ")<<plan.reason<<" nodes="<<plan.nodes
             <<" cost="<<plan.cost()<<" seconds="<<plan.seconds<<" length="<<length
             <<" search_seconds="<<plan.search_seconds<<'\n';
    return plan.success ? 0:2;
}
int runIntegrator(const Scene &,const std::filesystem::path &);
int runUnicycle(const Scene &,const std::filesystem::path &);
int runQuad2(const Scene &,const std::filesystem::path &);
int runQuad3(const Scene &,const std::filesystem::path &);
} // namespace flask
