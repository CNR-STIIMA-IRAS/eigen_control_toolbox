#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"pid_controller");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

  eigen_control_toolbox::FirstOrderLowPass lpf(500,1e-3);
  lpf.importMatricesFromParam(nh,"/meto_cfg/filter");
  unsigned int order = lpf.getOrder();
  unsigned int nin   = lpf.getNumberOfInputs();
  unsigned int nout  = lpf.getNumberOfOutputs();
  
  Eigen::VectorXd vect1=  Eigen::VectorXd::LinSpaced(18, 1, 18);
  ROS_INFO_STREAM("vect1: "<<vect1.transpose());
  
  Eigen::Map<Eigen::MatrixXd> mat1(vect1.data(), 6,3);
  ROS_INFO_STREAM("mat1: \n"<<mat1);
  Eigen::MatrixXd mat2=mat1.transpose();
  ROS_INFO_STREAM("mat2: \n"<<mat2);
  
  Eigen::VectorXd last_u(order*nin);
  Eigen::VectorXd last_y(order*nout);
  
  last_u.setZero();
  last_y.setZero();
  
  lpf.setStateFromIO(last_u,last_y);
  lpf.setStateFromLastIO(last_u.head(1),   last_y.head(1));
  ROS_INFO_STREAM("state:\n"<<lpf.getState());
  ROS_INFO_STREAM("output:\n"<<lpf.getOutput() << "\ndesired:\n"<<last_y.tail(nout));
  
  for (unsigned int i=0;i<10;i++)
  {
    Eigen::VectorXd u(nin);
    u.setConstant(1);
    lpf.update(u);
    ROS_INFO_STREAM("output:"<<lpf.getOutput().transpose() << ", input:"<<u.transpose());
    
  }
  
  return 0; 
}