#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_matrix");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

  unsigned int order=10;
  unsigned int nin=1;
  unsigned int nout=1;
  
  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
 
  eigen_control_toolbox::DiscreteStateSpace ss(A,B,C,D);
  Eigen::MatrixXd io_mtx = ss.computeInputToOutputMatrix();
  ROS_INFO_STREAM("io_mtx=\n"<<io_mtx);
  
  Eigen::MatrixXd obsv_mtx = ss.computeObservatibilityMatrix();
  ROS_INFO_STREAM("obsv_mtx=\n" << obsv_mtx);
  
  Eigen::VectorXd u(order*nin);
  Eigen::VectorXd y(order*nout);
  
  u.setRandom();
  y.setRandom();
  
  ss.setStateFromIO(u,y);
  ROS_INFO_STREAM("state:\n"<<ss.getState());
//   ss.update(u.tail(nin));
  ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y.tail(nout));
  
  
  eigen_control_toolbox::DiscreteStateSpace ss2;
  if (!ss2.importMatricesFromParam(nh,"ss"))
  {
    ROS_ERROR("error");
    return -1;
  }
  return 0; 
}