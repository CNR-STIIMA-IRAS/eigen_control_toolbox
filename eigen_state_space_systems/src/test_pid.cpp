#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_controllers.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_pid");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));

  eigen_control_toolbox::Controller proportional;
  if (!proportional.importMatricesFromParam(nh,"ctrl1"))
  {
    ROS_ERROR("Failing initializing controller ctrl1");
    return 0;
  }
  ROS_INFO("ctrl1:");
  proportional.print();

  eigen_control_toolbox::Controller pi;
  if (!pi.importMatricesFromParam(nh,"ctrl2"))
  {
    ROS_ERROR("Failing initializing controller ctrl1");
    return 0;
  }

  ROS_INFO("ctrl2:");
  pi.print();

  for (unsigned int idx=0;idx<200;idx++)
  {
    double controller_input=1;
    double p_output=proportional.update(controller_input);
    ROS_INFO("controller_input=%f, proportional output=%f",controller_input,p_output);
  }

  for (unsigned int idx=0;idx<200;idx++)
  {
    double controller_input=1;
    double pi_output=pi.update(controller_input);
    double pi_sat=std::max(std::min(pi_output,2.0),-2.0);
    pi.antiwindup(pi_sat,pi_output);
    ROS_INFO("controller_input=%f, PI output=%f, PI saturated output=%f",controller_input,pi_output,pi_sat);
   }


  return 0;
}
