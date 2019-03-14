#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_filters");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

  double natural_frequency = 500; // [rad/s]
  double sampling_period=0.001; // s
  eigen_control_toolbox::FirstOrderLowPass lpf(natural_frequency,sampling_period);

//  lpf.importMatricesFromParam(nh,"/filter"); //you can load filter coefficient from ROS param
  unsigned int order = lpf.getOrder();
  unsigned int nin   = lpf.getNumberOfInputs();
  unsigned int nout  = lpf.getNumberOfOutputs();


  double u=0;
  double y=0;

  lpf.setStateFromLastIO(u,  y);
  ROS_INFO_STREAM("state:\n"<<lpf.getState());
  ROS_INFO_STREAM("output:\n"<<lpf.getOutput() << "\ndesired:\n"<<y);
  
  for (unsigned int i=0;i<10;i++)
  {
    u=1;
    y=lpf.update(u);
    ROS_INFO_STREAM("output:"<<y << ", input:"<<u);

  }
  
  return 0; 
}
