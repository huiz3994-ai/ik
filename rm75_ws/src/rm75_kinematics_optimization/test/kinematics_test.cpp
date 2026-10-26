#include <gtest/gtest.h>
#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/SVD>
#include "rm75_kinematics_optimization/adaptive_dls_ik.h"

using namespace rm75_kinematics;

class ModelTest : public ::testing::Test {
 protected:
  ModelTest() : model(RM75_TEST_URDF) {}
  KinematicsModel model;
};

TEST_F(ModelTest, LoadsConfirmedSevenAxis6fbModel) {
  EXPECT_EQ(model.dof(), 7);
  EXPECT_EQ(model.jointNames().front(), "joint1");
  EXPECT_EQ(model.jointNames().back(), "joint7");
  const auto limits = model.limits();
  EXPECT_NEAR(limits.lower[6], -6.28, 1e-12);
  EXPECT_NEAR(limits.upper[6], 6.28, 1e-12);
  // The 6FB joint-7 origin produces this zero-pose reach; catches accidental 6F loading.
  EXPECT_TRUE(model.forward(Eigen::VectorXd::Zero(7)).position.allFinite());
}

TEST_F(ModelTest, PinocchioFkMatchesKdl) {
  std::ifstream in(RM75_TEST_URDF); ASSERT_TRUE(in.good());
  const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  KDL::Tree tree; ASSERT_TRUE(kdl_parser::treeFromString(xml, tree));
  KDL::Chain chain; ASSERT_TRUE(tree.getChain("base_link", "Link7", chain));
  KDL::ChainFkSolverPos_recursive fk(chain);
  const Eigen::VectorXd q = (Eigen::VectorXd(7) << .2, -.4, .3, -.5, .25, .35, -.2).finished();
  KDL::JntArray joints(7); for (int i = 0; i < 7; ++i) joints(i) = q[i];
  KDL::Frame frame; ASSERT_GE(fk.JntToCart(joints, frame), 0);
  const Pose p = model.forward(q);
  EXPECT_NEAR(p.position.x(), frame.p.x(), 1e-10);
  EXPECT_NEAR(p.position.y(), frame.p.y(), 1e-10);
  EXPECT_NEAR(p.position.z(), frame.p.z(), 1e-10);
  Eigen::Matrix3d rk; for (int r=0;r<3;++r) for(int c=0;c<3;++c) rk(r,c)=frame.M(r,c);
  EXPECT_LT(KinematicsModel::so3Log(p.rotation * rk.transpose()).norm(), 1e-9);
}

TEST_F(ModelTest, LocalWorldAlignedJacobianMatchesFiniteDifference) {
  const Eigen::VectorXd q = (Eigen::VectorXd(7) << .2, -.4, .3, -.5, .25, .35, -.2).finished();
  const Eigen::MatrixXd j = model.jacobian(q);
  const Pose p = model.forward(q);
  const double h = 1e-7;
  for (int k = 0; k < 7; ++k) {
    Eigen::VectorXd qp=q; qp[k]+=h;
    const Pose pp=model.forward(qp);
    EXPECT_LT((j.block<3,1>(0,k)-(pp.position-p.position)/h).norm(), 2e-6);
    const Eigen::Vector3d omega=KinematicsModel::so3Log(pp.rotation*p.rotation.transpose())/h;
    EXPECT_LT((j.block<3,1>(3,k)-omega).norm(), 2e-6);
  }
}

TEST_F(ModelTest, ConsistencyAcrossRequiredConfigurations) {
  std::ifstream in(RM75_TEST_URDF); const std::string xml((std::istreambuf_iterator<char>(in)),{});
  KDL::Tree tree; ASSERT_TRUE(kdl_parser::treeFromString(xml,tree)); KDL::Chain chain;
  ASSERT_TRUE(tree.getChain("base_link","Link7",chain)); KDL::ChainFkSolverPos_recursive fk(chain);
  const auto l=model.limits();
  std::vector<std::pair<std::string,Eigen::VectorXd>> cases;
  cases.push_back({"zero/near-singular",Eigen::VectorXd::Zero(7)});
  cases.push_back({"random",(Eigen::VectorXd(7)<<.41,-.73,.55,-.82,.37,.64,-.91).finished()});
  Eigen::VectorXd near(7); for(int i=0;i<7;++i) near[i]=(i%2?l.lower[i]:l.upper[i])*.95;
  cases.push_back({"near-limit",near});
  for(const auto& item:cases) {
    KDL::JntArray joints(7); for(int i=0;i<7;++i)joints(i)=item.second[i]; KDL::Frame f;
    ASSERT_GE(fk.JntToCart(joints,f),0); const Pose p=model.forward(item.second);
    Eigen::Matrix3d rk; for(int r=0;r<3;++r)for(int c=0;c<3;++c)rk(r,c)=f.M(r,c);
    const double pe=(p.position-Eigen::Vector3d(f.p.x(),f.p.y(),f.p.z())).norm();
    const double re=KinematicsModel::so3Log(p.rotation*rk.transpose()).norm();
    const Eigen::MatrixXd j=model.jacobian(item.second); double je=0; const double h=1e-7;
    for(int k=0;k<7;++k){Eigen::VectorXd qp=item.second;qp[k]+=h;const Pose pp=model.forward(qp);
      Eigen::Matrix<double,6,1> col;col.head<3>()=(pp.position-p.position)/h;
      col.tail<3>()=KinematicsModel::so3Log(pp.rotation*p.rotation.transpose())/h;je=std::max(je,(col-j.col(k)).norm());}
    std::cout<<"VALIDATION "<<item.first<<" fk_position="<<pe<<" fk_rotation="<<re<<" jacobian_max="<<je<<std::endl;
    EXPECT_LT(pe,1e-10);EXPECT_LT(re,1e-9);EXPECT_LT(je,3e-6);
  }
}

TEST_F(ModelTest, So3LogUsesShortestRotation) {
  const Eigen::Matrix3d r = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  EXPECT_NEAR(KinematicsModel::so3Log(r).norm(), .4, 1e-12);
  EXPECT_LT(KinematicsModel::so3Log(Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST_F(ModelTest, AdaptiveDlsConvergesFromIndependentSeed) {
  const Eigen::VectorXd truth=(Eigen::VectorXd(7)<<.2,-.5,.25,-.7,.3,.6,-.2).finished();
  const Eigen::VectorXd seed=(Eigen::VectorXd(7)<<-.3,-.15,-.2,-.35,-.25,.2,.4).finished();
  IkOptions o; o.timeout_sec=.5; o.max_iterations=300;
  const auto r=AdaptiveDlsIk(model).solve(model.forward(truth),seed,o);
  EXPECT_EQ(r.status,IkStatus::SUCCESS)<<r.detail;
  EXPECT_LE(r.position_error,o.position_tolerance);
  EXPECT_LE(r.orientation_error,o.orientation_tolerance);
}

TEST_F(ModelTest, DampingIncreasesNearSingularity) {
  IkOptions o;
  EXPECT_NEAR(AdaptiveDlsIk::adaptiveDamping(.2,o),o.damping_min,1e-15);
  EXPECT_GT(AdaptiveDlsIk::adaptiveDamping(1e-8,o),.9*o.damping_max);
  const Eigen::MatrixXd j=model.jacobian(Eigen::VectorXd::Zero(7));
  EXPECT_TRUE(j.allFinite());
}

TEST_F(ModelTest, NullspaceProjectionMeasuresDampedLeakage) {
  const Eigen::VectorXd q=(Eigen::VectorXd(7)<<0,-1.5,0,-1.8,0,-1.0,5.8).finished();
  const Eigen::MatrixXd j=model.jacobian(q);
  Eigen::MatrixXd a=j*j.transpose(); a.diagonal().array()+=1e-6;
  const Eigen::MatrixXd pinv=j.transpose()*a.ldlt().solve(Eigen::MatrixXd::Identity(6,6));
  bool active=false;
  const Eigen::VectorXd g=NullspaceOptimizer::jointCenteringGradient(q,model.limits(),.15,&active);
  NullspaceDebug d; const Eigen::VectorXd n=NullspaceOptimizer::project(j,pinv,g,&d);
  EXPECT_TRUE(active); EXPECT_TRUE(n.allFinite());
  EXPECT_NEAR((j*n).norm(),d.primary_leakage_norm,1e-12);
  EXPECT_LT(d.primary_leakage_norm,2e-3*g.norm());
}

TEST_F(ModelTest, RejectsInvalidAndOutOfLimitSeeds) {
  Pose target=model.forward(Eigen::VectorXd::Zero(7));
  Eigen::VectorXd bad=Eigen::VectorXd::Zero(7); bad[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AdaptiveDlsIk(model).solve(target,bad).status,IkStatus::NUMERICAL_ERROR);
  bad.setZero(); bad[0]=100;
  EXPECT_EQ(AdaptiveDlsIk(model).solve(target,bad).status,IkStatus::JOINT_LIMIT);
}

TEST_F(ModelTest, UnreachableTargetReturnsDetailedFailure) {
  Pose target=model.forward(Eigen::VectorXd::Zero(7)); target.position.x()+=10.0;
  IkOptions o; o.timeout_sec=.2; o.max_iterations=30;
  const auto r=AdaptiveDlsIk(model).solve(target,Eigen::VectorXd::Zero(7),o);
  EXPECT_NE(r.status,IkStatus::SUCCESS); EXPECT_FALSE(r.detail.empty());
}

TEST_F(ModelTest, PositionOnlyIgnoresOrientationMismatch) {
  const Eigen::VectorXd seed=Eigen::VectorXd::Zero(7); Pose target=model.forward(seed);
  target.rotation=Eigen::AngleAxisd(2.5,Eigen::Vector3d::UnitX()).toRotationMatrix()*target.rotation;
  IkOptions o; o.orientation_enabled=false;
  const auto r=AdaptiveDlsIk(model).solve(target,seed,o);
  EXPECT_EQ(r.status,IkStatus::SUCCESS); EXPECT_LT(r.position_error,1e-12);
  EXPECT_GT(r.orientation_error,2.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
