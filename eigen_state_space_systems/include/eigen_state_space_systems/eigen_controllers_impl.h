#ifndef eigen_controller_impl_201811280951
#define eigen_controller_impl_201811280951
#include <rosparam_utilities/rosparam_utilities.h>

// #include <eigen_state_space_systems/eigen_controllers.h>

namespace eigen_control_toolbox
{

Controller::Controller(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D)
{
  Controller::setMatrices(A,B,C,D);
};

void Controller::setMatrices(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D)
{
  eigen_control_toolbox::DiscreteStateSpace::setMatrices(A, B, C, D);
  
  m_Baw.resize(m_order,m_nout);
  m_Baw.setZero();
}

Controller::Controller()
{

};



void Controller::setAntiWindupMatrix(const Eigen::Ref< Eigen::MatrixXd > Baw)
{
  assert(Baw.rows()==m_order);
  assert(Baw.cols()==m_nout);
  
  m_Baw=Baw;
}

bool Controller::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  DiscreteStateSpace::importMatricesFromParam(nh,name);
  Eigen::MatrixXd aw_gain;   //antiwindup_gain
  std::vector<double> aw_states; //antiwindup_gain
  
  if (!eigen_utils::getParam(nh, name+"/antiwindup_gain", aw_gain))
  {
    ROS_WARN("[Controller] cannot find '%s/antiwindup_gain' parameter!. SET NULL!!!!!",name.c_str());
    //return false;
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
    ROS_WARN("[Controller] cannot find '%s/antiwindup_states' parameter!",name.c_str());
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

Eigen::VectorXd Controller::update(const Eigen::Ref< Eigen::VectorXd > input)
{
  return eigen_control_toolbox::DiscreteStateSpace::update(input);
}

void Controller::antiwindup(const Eigen::Ref< Eigen::VectorXd > saturated_output, Eigen::Ref< Eigen::VectorXd > unsaturated_output)
{
  Eigen::VectorXd aw=saturated_output-unsaturated_output;
  m_state+=m_Baw*aw;
}

double Controller::update(const double& input)
{
  assert(m_nin==1);
  Eigen::VectorXd u(1);
  u(0)=input;
  assert(m_nout==1);
  m_output=update(u);
  return m_output(0);
  
}

}
#endif