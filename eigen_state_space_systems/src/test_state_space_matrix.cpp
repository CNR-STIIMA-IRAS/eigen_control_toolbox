#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_matrix");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

  unsigned int order=10; // system order
  unsigned int nin=1;    // number of inputs
  unsigned int nout=1;   // number of outputs
  
  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
 
  eigen_control_toolbox::DiscreteStateSpace ss(A,B,C,D);

  
  Eigen::VectorXd u(nin);   //input vector
  Eigen::VectorXd y(nout);  //output vector
  
  u.setRandom();
  y.setRandom();
  
  ss.setStateFromLastIO(u,y); // initialize initial state value for dumpless startup
  ROS_INFO_STREAM("state:\n"<<ss.getState());
  ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y);
  
  y=ss.update(u); // computing one step, updating state and output
  
  eigen_control_toolbox::DiscreteStateSpace ss2;
  if (!ss2.importMatricesFromParam(nh,"ss"))
  {
    ROS_ERROR("error");
    return -1;
  }
  return 0; 
}
