#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_controllers.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_pid");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

//  eigen_control_toolbox::PID p;
//  p.setP();
  return 0;
}
