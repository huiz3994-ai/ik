#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "rm75_kinematics_optimization/adaptive_dls_ik.h"

using rm75_kinematics::AdaptiveDlsIk;
using rm75_kinematics::IkOptions;
using rm75_kinematics::IkResult;
using rm75_kinematics::IkStatus;
using rm75_kinematics::KinematicsModel;
using rm75_kinematics::Pose;
using rm75_kinematics::toString;

struct Row {
  std::string solver;
  std::string scenario;
  int sample;
  IkResult result;
  double joint_delta;
  Eigen::VectorXd target_q;
  Eigen::VectorXd seed_q;
};

static double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t i=static_cast<std::size_t>(std::floor(p*(v.size()-1)));
  return v[i];
}

static Eigen::VectorXd sampleQ(std::mt19937& rng, const rm75_kinematics::JointLimits& l,
                               double fraction) {
  Eigen::VectorXd q(l.lower.size());
  std::uniform_real_distribution<double> u(-fraction,fraction);
  for (Eigen::Index i=0;i<q.size();++i) {
    const double mid=.5*(l.lower[i]+l.upper[i]);
    q[i]=mid+u(rng)*.5*(l.upper[i]-l.lower[i]);
  }
  return q;
}

class KdlBaseline {
 public:
  KdlBaseline(const std::string& urdf, const rm75_kinematics::JointLimits& limits) {
    std::ifstream in(urdf); const std::string xml((std::istreambuf_iterator<char>(in)),{});
    KDL::Tree tree;
    if(xml.empty() || !kdl_parser::treeFromString(xml,tree) || !tree.getChain("base_link","Link7",chain_))
      throw std::runtime_error("Unable to create KDL RM75 chain");
    lower_.resize(7); upper_.resize(7);
    for(int i=0;i<7;++i){lower_(i)=limits.lower[i];upper_(i)=limits.upper[i];}
  }
  IkResult solve(const Pose& target,const Eigen::VectorXd& seed,const KinematicsModel& model,
                 bool orientation_required=true) const {
    const auto start=std::chrono::steady_clock::now(); KDL::JntArray q0(7),q(7);
    for(int i=0;i<7;++i)q0(i)=seed[i];
    KDL::ChainFkSolverPos_recursive fk(chain_); KDL::ChainIkSolverVel_pinv vel(chain_,1e-5,150);
    KDL::ChainIkSolverPos_NR_JL ik(chain_,lower_,upper_,fk,vel,150,1e-6);
    const KDL::Rotation r(target.rotation(0,0),target.rotation(0,1),target.rotation(0,2),
                          target.rotation(1,0),target.rotation(1,1),target.rotation(1,2),
                          target.rotation(2,0),target.rotation(2,1),target.rotation(2,2));
    const int code=ik.CartToJnt(q0,KDL::Frame(r,KDL::Vector(target.position.x(),target.position.y(),target.position.z())),q);
    IkResult out; out.q.resize(7); for(int i=0;i<7;++i)out.q[i]=q(i);
    out.iterations=code>=0?0:150; out.solve_time_ms=1e3*std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    if(out.q.allFinite()) {
      const auto e=model.poseError(model.forward(out.q),target); out.position_error=e.head<3>().norm(); out.orientation_error=e.tail<3>().norm();
      out.min_limit_margin=rm75_kinematics::NullspaceOptimizer::minimumLimitMargin(out.q,model.limits());
    }
    if(code>=0 && out.position_error<=.002 && (!orientation_required || out.orientation_error<=.034906585)) {out.status=IkStatus::SUCCESS;out.detail="KDL converged";}
    else {out.status=code==-5?IkStatus::MAX_ITERATIONS:IkStatus::UNREACHABLE_OR_STALLED;out.detail="KDL code "+std::to_string(code);}
    return out;
  }
 private: KDL::Chain chain_; KDL::JntArray lower_,upper_;
};

int main(int argc,char** argv) {
  int count=1000; unsigned seed=7506; std::string output="/tmp/rm75_benchmark";
  for(int i=1;i<argc;++i) {
    const std::string a=argv[i];
    if(a=="--count"&&i+1<argc) count=std::stoi(argv[++i]);
    else if(a=="--seed"&&i+1<argc) seed=static_cast<unsigned>(std::stoul(argv[++i]));
    else if(a=="--output"&&i+1<argc) output=argv[++i];
    else { std::cerr<<"Usage: rm75_ik_benchmark [--count N] [--seed N] [--output PREFIX]\n"; return 2; }
  }
  if(count<=0) return 2;
  KinematicsModel model(RM75_DEFAULT_URDF);
  AdaptiveDlsIk solver(model); const auto limits=model.limits(); KdlBaseline kdl(RM75_DEFAULT_URDF,limits); std::mt19937 rng(seed);
  const std::vector<std::string> names={"kdl","legacy_fixed_dls","adaptive_dls","adaptive_dls_nullspace"};
  std::vector<Row> rows; rows.reserve(static_cast<std::size_t>(count)*names.size());
  Eigen::VectorXd previous_truth=sampleQ(rng,limits,.5);
  std::uniform_real_distribution<double> small(-.01,.01);
  for(int sample=0;sample<count;++sample) {
    std::string scenario; Eigen::VectorXd truth=sampleQ(rng,limits,.82); Eigen::VectorXd initial=sampleQ(rng,limits,.55);
    switch(sample%6) {
      case 0: scenario="random_position"; break;
      case 1: scenario="random_6d"; break;
      case 2: scenario="large_initial_error"; initial=-.55*truth; break;
      case 3: scenario="near_joint_limit"; truth=sampleQ(rng,limits,.5); truth[sample%7]=limits.upper[sample%7]-.03; break;
      case 4: scenario="near_singular"; truth.setZero(); for(int k=0;k<7;++k)truth[k]=small(rng); break;
      default: scenario="continuous_small_step"; initial=previous_truth; truth=previous_truth; for(int k=0;k<7;++k)truth[k]+=small(rng); break;
    }
    truth=truth.cwiseMax((limits.lower.array()+1e-5).matrix())
               .cwiseMin((limits.upper.array()-1e-5).matrix());
    previous_truth=truth;
    const Pose target=model.forward(truth);
    for(const auto& name:names) {
      IkOptions o; o.timeout_sec=.08; o.max_iterations=180;
      if(scenario=="random_position") o.orientation_enabled=false;
      if(name=="kdl") { const IkResult r=kdl.solve(target,initial,model,o.orientation_enabled); rows.push_back({name,scenario,sample,r,(r.q-initial).norm(),truth,initial}); continue; }
      if(name=="legacy_fixed_dls") { o.damping_min=.025; o.damping_max=.025; }
      if(name=="adaptive_dls_nullspace") {
        // Isolate joint-centering impact. Reference and manipulability objectives are
        // independently available options, but combining all gains would confound this row.
        o.enable_joint_centering=true;
      }
      const IkResult r=solver.solve(target,initial,o);
      rows.push_back({name,scenario,sample,r,(r.q-initial).norm(),truth,initial});
    }
  }
  std::ofstream csv(output+".csv");
  csv<<"solver,scenario,sample,status,time_ms,iterations,position_error_m,orientation_error_rad,joint_delta_rad,min_limit_margin_rad,min_singular_value,damping";
  for(int i=1;i<=7;++i) csv<<",target_q"<<i;
  for(int i=1;i<=7;++i) csv<<",seed_q"<<i;
  csv<<"\n";
  csv<<std::setprecision(12);
  for(const auto& x:rows) { csv<<x.solver<<','<<x.scenario<<','<<x.sample<<','<<toString(x.result.status)<<','
    <<x.result.solve_time_ms<<','<<x.result.iterations<<','<<x.result.position_error<<','
    <<x.result.orientation_error<<','<<x.joint_delta<<','<<x.result.min_limit_margin<<','
    <<x.result.min_singular_value<<','<<x.result.damping;
    for(int i=0;i<7;++i) csv<<','<<x.target_q[i];
    for(int i=0;i<7;++i) csv<<','<<x.seed_q[i];
    csv<<'\n';
  }
  std::ofstream json(output+".json");
  json<<"{\n  \"model\": \"RM75-6FB\",\n  \"base\": \"base_link\",\n  \"tip\": \"Link7\",\n"
      <<"  \"seed\": "<<seed<<",\n  \"samples\": "<<count<<",\n  \"target_sampling\": \"six deterministic scenario classes; exact target_q and seed_q retained in CSV\",\n  \"solvers\": {\n";
  for(std::size_t ni=0;ni<names.size();++ni) {
    const auto& name=names[ni]; int success=0,success_ori_count=0,numeric=0; double pos=0,ori=0,success_pos=0,success_ori=0,delta=0,margin=0,max_pos=0;
    std::vector<double> times; std::map<std::string,int> failures;
    for(const auto& x:rows) if(x.solver==name) {
      times.push_back(x.result.solve_time_ms); pos+=x.result.position_error; ori+=x.result.orientation_error;
      delta+=x.joint_delta; margin+=x.result.min_limit_margin; max_pos=std::max(max_pos,x.result.position_error);
      if(x.result.status==IkStatus::SUCCESS) {
        ++success; success_pos+=x.result.position_error;
        if(x.scenario!="random_position") {success_ori+=x.result.orientation_error; ++success_ori_count;}
      } else ++failures[toString(x.result.status)];
      if(!std::isfinite(x.result.position_error)||!x.result.q.allFinite()) ++numeric;
    }
    json<<"    \""<<name<<"\": {\"success_rate\": "<<double(success)/count
        <<", \"mean_time_ms\": "<<std::accumulate(times.begin(),times.end(),0.0)/count
        <<", \"p50_time_ms\": "<<percentile(times,.50)<<", \"p95_time_ms\": "<<percentile(times,.95)
        <<", \"p99_time_ms\": "<<percentile(times,.99)<<", \"mean_final_position_error_m\": "<<pos/count
        <<", \"mean_final_orientation_error_rad\": "<<ori/count
        <<", \"success_mean_position_error_m\": "<<(success?success_pos/success:0.0)
        <<", \"orientation_tasks_success_mean_error_rad\": "<<(success_ori_count?success_ori/success_ori_count:0.0)<<", \"max_final_position_error_m\": "<<max_pos
        <<", \"mean_joint_delta_rad\": "<<delta/count<<", \"mean_min_limit_margin_rad\": "<<margin/count
        <<", \"numeric_anomalies\": "<<numeric<<", \"failures\": {";
    bool first=true; for(const auto& f:failures) { if(!first)json<<','; first=false; json<<"\""<<f.first<<"\":"<<f.second; }
    json<<"}}"<<(ni+1<names.size()?",":"")<<"\n";
  }
  json<<"  }\n}\n";
  std::cout<<"Wrote "<<output<<".csv and "<<output<<".json ("<<rows.size()<<" solves)\n";
  return 0;
}
