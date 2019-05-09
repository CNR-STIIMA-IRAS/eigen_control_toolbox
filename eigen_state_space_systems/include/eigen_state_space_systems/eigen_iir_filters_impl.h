#ifndef   eigen_iir_filters_impl_201811280954
#define   eigen_iir_filters_impl_201811280954

// #include <eigen_state_space_systems/eigen_iir_filters.h>



namespace eigen_control_toolbox
{
  
inline FirstOrderLowPass::FirstOrderLowPass()
{
  m_natural_frequency=1;
  m_sampling_period=1;
  computeMatrices(m_natural_frequency, m_sampling_period);
};

inline FirstOrderLowPass::FirstOrderLowPass(const double& natural_frequency, const double& sample_period)
{
  m_natural_frequency=natural_frequency;
  m_sampling_period=sample_period;
  computeMatrices(natural_frequency, sample_period);
}


inline void FirstOrderLowPass::computeMatrices(const double& natural_frequency, const double& sample_period)
{
  Eigen::MatrixXd A(1,1);
  Eigen::MatrixXd B(1,1);
  Eigen::MatrixXd C(1,1);
  Eigen::MatrixXd D(1,1);
  
  assert(sample_period>0);
  assert(natural_frequency>0);
  assert(natural_frequency<M_PI/sample_period);
  
  // xn=coef*x+(1-coef)*u
  // y=x;
  // Dy=(xn-x)/st=(coef-1)/st*x+(1-coef)/st*u
  
  double coef=std::exp(-sample_period*natural_frequency);
  A(0,0)=coef;
  B(0,0)=1-coef;
  C(0,0)=1.0;
  D(0,0)=0.0;
  
  setMatrices(A,B,C,D);
};

inline bool FirstOrderLowPass::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  double natural_frequency;
  double sample_period;
  
  if (nh.hasParam(name+"/natural_frequency"))
  {
    if (!nh.getParam(name+"/natural_frequency",natural_frequency))
    {
      ROS_ERROR("%s/natural_frequency is not a double",name.c_str());
      return false;
    }
    if (natural_frequency<0)
    {
      ROS_ERROR("%s/natural_frequency should be positive",name.c_str());
      return false;
    }
  }
  else if (nh.hasParam(name+"/frequency"))
  {
    double frequency;
    if (!nh.getParam(name+"/frequency",frequency))
    {
      ROS_ERROR("%s/frequency is not a double",name.c_str());
      return false;
    }
    if (frequency<0)
    {
      ROS_ERROR("%s/frequency should be positive",name.c_str());
      return false;
    }
    natural_frequency=2*M_PI*frequency;
  }
  else 
  {
    ROS_ERROR("Neither %s/natural_frequency nor %s/frequency are defined",name.c_str(),name.c_str());
    return false;
  }
  
  
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
  computeMatrices(natural_frequency,sample_period);
  
  return true;
}

inline FirstOrderHighPass::FirstOrderHighPass()
{
  
};

inline FirstOrderHighPass::FirstOrderHighPass(const double& natural_frequency, const double& sample_period)
{
  m_natural_frequency=natural_frequency;
  m_sampling_period=sample_period;
  computeMatrices(natural_frequency,sample_period);
}

inline void FirstOrderHighPass::computeMatrices(const double& natural_frequency, const double& sample_period)
{
  Eigen::MatrixXd A(1,1);
  Eigen::MatrixXd B(1,1);
  Eigen::MatrixXd C(1,1);
  Eigen::MatrixXd D(1,1);
  
  assert(sample_period>0);
  assert(natural_frequency>0);
  
  // xn=coef*x+(1-coef)*u
  // y=x;
  // Dy=(xn-x)/st=(coef-1)/st*x+(1-coef)/st*u
  
  double coef=std::exp(-sample_period*natural_frequency);
  A(0,0)=coef;
  B(0,0)=1-coef;
  C(0,0)=-coef/sample_period;
  D(0,0)=coef/sample_period;
  
  setMatrices(A,B,C,D);
};

inline bool FirstOrderHighPass::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  double natural_frequency;
  double sample_period;
  
  if (nh.hasParam(name+"/natural_frequency"))
  {
    if (!nh.getParam(name+"/natural_frequency",natural_frequency))
    {
      ROS_ERROR("%s/natural_frequency is not a double",name.c_str());
      return false;
    }
    if (natural_frequency<0)
    {
      ROS_ERROR("%s/natural_frequency should be positive",name.c_str());
      return false;
    }
  }
  else if (nh.hasParam(name+"/frequency"))
  {
    double frequency;
    if (!nh.getParam(name+"/frequency",frequency))
    {
      ROS_ERROR("%s/frequency is not a double",name.c_str());
      return false;
    }
    if (frequency<0)
    {
      ROS_ERROR("%s/frequency should be positive",name.c_str());
      return false;
    }
    natural_frequency=2*M_PI*frequency;
  }
  else 
  {
    ROS_ERROR("Neither %s/natural_frequency nor %s/frequency are defined",name.c_str(),name.c_str());
    return false;
  }
  
  
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
  computeMatrices(natural_frequency,sample_period);
  
  return true;
}


}

#endif
