#include <cstdlib>
#include <iomanip>
#include <iostream>
#include "rm75_kinematics_optimization/kinematics_model.h"

int main(int argc, char** argv) {
  if (argc != 9) {
    std::cerr << "Usage: rm75_fk_cli URDF q1 q2 q3 q4 q5 q6 q7\n";
    return 2;
  }
  try {
    Eigen::VectorXd q(7); for (int i=0;i<7;++i) q[i]=std::stod(argv[i+2]);
    rm75_kinematics::KinematicsModel model(argv[1]); const auto t=model.forward(q).isometry().matrix();
    std::cout<<std::setprecision(17)<<"[";
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) {
      if(r||c) std::cout<<",";
      std::cout<<t(r,c);
    }
    std::cout<<"]\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }
  return 0;
}
