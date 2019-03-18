#ifndef eigen_controller_impl_201811280951
#define eigen_controller_impl_201811280951
#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_state_space_systems/eigen_controllers.h>

namespace eigen_control_toolbox
{

inline Controller::Controller(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D)
{
  Controller::setMatrices(A,B,C,D);
};

inline void Controller::setMatrices(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D)
{
  eigen_control_toolbox::DiscreteStateSpace::setMatrices(A, B, C, D);
  
  m_Baw.resize(m_order,m_nout);
  m_Baw.setZero();
}

inline Controller::Controller()
{

};

inline void Controller::setAntiWindupMatrix(const Eigen::Ref< Eigen::MatrixXd > Baw)
{
  assert(Baw.rows()==m_order);
  assert(Baw.cols()==m_nout);
  
  m_Baw=Baw;
}

inline bool Controller::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  DiscreteStateSpace::importMatricesFromParam(nh,name);
  Eigen::MatrixXd aw_gain;   //antiwindup_gain
  std::vector<double> aw_states; //antiwindup_gain
  
  if (!eigen_utils::getParam(nh, name+"/antiwindup_gain", aw_gain))
  {
    ROS_DEBUG("[Controller] cannot find '%s/antiwindup_gain' parameter!. SET NULL!!!!!",name.c_str());
    aw_gain.resize(m_nin,m_nout);
    aw_gain.setZero();
  }
  if ( (!aw_gain.cols()==m_nout) || (!aw_gain.rows()==m_nin))
  {
    ROS_WARN("antiwindup_gain size is wrong ([%zu,%zu] instead of [%u,%u]",aw_gain.rows(),aw_gain.cols(),m_nin,m_nout);
    return false;
  }
  
  if (!rosparam_utilities::getParamVector(nh, name+"/antiwindup_states", aw_states))
  {
    ROS_DEBUG("[Controller] cannot find '%s/antiwindup_states' parameter!",name.c_str());
    aw_states.resize(m_order,0);
  }
  if (!aw_states.size()==m_order)
  {
    ROS_WARN("antiwindup_states size is wrong (%zu instead of %u",aw_states.size(),m_order);
    return false;
  }
  
  Eigen::MatrixXd Baw=m_B*aw_gain;  
  // Baw norder x nout
  // B  norder x nin
  // aw_gain nin x nout
  // Baw=m_B*aw_gain  ===>  (norder x nout) = (norder x nin) x ( nin x nout)
  
  
  for (unsigned int iord=0;iord<m_order;iord++)
  {
    if (!aw_states.at(iord))
      Baw.row(iord).setZero();
  }

  setAntiWindupMatrix(Baw);
  
}

inline Eigen::VectorXd Controller::update(const Eigen::Ref< Eigen::VectorXd > input)
{
  return eigen_control_toolbox::DiscreteStateSpace::update(input);
}

inline void Controller::antiwindup(const Eigen::Ref< Eigen::VectorXd > saturated_output, Eigen::Ref< Eigen::VectorXd > unsaturated_output)
{
  Eigen::VectorXd aw=saturated_output-unsaturated_output;
  m_state+=m_Baw*aw;
}

inline double Controller::update(const double& input)
{
  assert(m_nin==1);
  Eigen::VectorXd u(1);
  u(0)=input;
  assert(m_nout==1);
  m_output=update(u);
  return m_output(0);
  
}


inline void Controller::setPI(const double &Kp, const double &Ki, const double& sampling_period)
{
  Eigen::MatrixXd A(1,1);
  Eigen::MatrixXd B(1,1);
  Eigen::MatrixXd C(1,1);
  Eigen::MatrixXd D(1,1);

  A(0,0)=1.0;
  B(0,0)=sampling_period;
  C(1,0)=Ki;
  D(0,0)=Kp;


}

inline bool Controller::importPIDFromParam(const ros::NodeHandle &nh, const std::string &controller_name)
{
  double Kp=0;
  if (nh.hasParam(name+"/proportional_gain"))
  {
    if (!nh.getParam(name+"/proportional_gain",Kp))
    {
      ROS_ERROR("%s/proportional_gain is not a double",name.c_str());
      return false;
    }
    if (Kp<0)
    {
      ROS_INFO("%s/proportional_gain is negative, are you sure?",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/proportional_gain is not defined",name.c_str());
    return false;
  }

  double Ki=0;
  if (nh.hasParam(name+"/integral_gain"))
  {
    if (!nh.getParam(name+"/integral_gain",Ki))
    {
      ROS_ERROR("%s/integral_gain is not a double",name.c_str());
      return false;
    }
    if (Ki<0)
    {
      ROS_INFO("%s/integral_gain is negative, are you sure?",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/integral_gain is not defined",name.c_str());
    return false;
  }

  double sample_period;

  if (nh.hasParam(name+"/sample_period"))
  {
    if (!nh.getParam(name+"/sample_period",sample_period))
    {
      ROS_ERROR("%s/sample_period is not a double",name.c_str());
      return false;
    }
    if (sample_period<0)
    {
      ROS_ERROR("%s/sample_period should be positive",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/sample_period is not defined",name.c_str());
    return false;
  }

  setPI(Kp,Ki,sample_period);
  return true;

}

}



#endif
