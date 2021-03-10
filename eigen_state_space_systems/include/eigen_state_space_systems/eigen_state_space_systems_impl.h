#ifndef   eigen_state_space_systems_impl_201811280956
#define   eigen_state_space_systems_impl_201811280956

#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <ros/console.h>

namespace eigen_control_toolbox 
{
  

  
inline DiscreteStateSpace::DiscreteStateSpace( const Eigen::Ref<Eigen::MatrixXd> A, 
                                        const Eigen::Ref<Eigen::MatrixXd> B, 
                                        const Eigen::Ref<Eigen::MatrixXd> C, 
                                        const Eigen::Ref<Eigen::MatrixXd> D)
{
  setMatrices(A,B,C,D);
};

inline void DiscreteStateSpace::setMatrices(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D)
{
  m_A = A;
  m_B = B;
  m_C = C;
  m_D = D;
  
  m_order = m_A.rows();
  if (m_A.cols() != m_order)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix A is not square");
  
  m_nin = m_B.cols();
  if (m_B.rows() != m_order)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix B has wrong rows dimension");
  
  
  m_nout = m_C.rows();
  if (m_C.cols() != m_order)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix C has wrong cols dimension");
  
  if (m_D.rows() != m_nout)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix D has wrong rows dimension");
  
  if (m_D.cols() != m_nin)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix D has wrong cols dimension");
  
  m_state.resize(m_order);
  m_state.setZero();
  m_output.resize(m_nout);
  m_output.setZero();
  
  
}


inline Eigen::VectorXd DiscreteStateSpace::update(const Eigen::Ref<Eigen::VectorXd> input)
{
  if (input.rows() != m_nin)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix input has wrong rows dimension");
  
  m_output = m_C*m_state + m_D*input;
  m_state = m_A*m_state+m_B*input;
  return m_output;
}

inline void DiscreteStateSpace::setState(const Eigen::Ref< Eigen::VectorXd > state)
{
  if (state.rows() != m_order)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix input has wrong rows dimension");
  m_state=state;
}

inline void DiscreteStateSpace::setStateFromIO(const Eigen::Ref< Eigen::VectorXd > past_inputs, const Eigen::Ref< Eigen::VectorXd > past_outputs)
{
  /*
   * 
   * y(0) = C*x(0)         + D u(0)
   * y(1) = C*A*x(0)       + D u(1) + C*B*u(0)
   * y(2) = C*A^2*x(0)     + D u(2) + C*B*u(1)+C*A*B*u(1)
   * ......
   * y(n) = C*A^(n-1)*x(0) + D u(n) + sum(j=0:n-1) C*A^j*B*u(n-1-j)
   *
   * Y=[y(0) ... y(n)]
   * U=[u(0) ... u(n)]
   * Y=obsv(A,C)*x(0)+ioMatrix(A,B,C,D)*U 
   * x(0)= obsv(A,C) \ ( Y-ioMatrix(A,B,C,D)*U )
   *
   * x(1) = A*x(0)+B*u(0)
   * ......
   * x(n) = A*x(n-1)+B*u(n-1)
   */


  if (past_inputs.rows() != m_nin*m_order)
    throw std::invalid_argument(("[ DiscreteStateSpace ]  past_inputs has wrong dimenstion: "+std::to_string(past_inputs.rows())+" instead of "+std::to_string(m_nin*m_order)).c_str());
  
  if (past_outputs.rows() != m_nout*m_order)
    throw std::invalid_argument("[ DiscreteStateSpace ] past_outputs has wrong dimenstion");
  
  Eigen::MatrixXd obsv=computeObservatibilityMatrix();
  Eigen::MatrixXd io_matrix=computeInputToOutputMatrix();
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(obsv, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.rank()<m_order)
  {
    ROS_DEBUG("Matrix is rank deficient");
    m_state.resize(m_order);
    m_state.setZero();
  }
  else 
  {
    m_state = svd.solve(past_outputs-io_matrix*past_inputs); // state at the begin of initialization interval
  } 
  for (unsigned int istep=0;istep<m_order;istep++)
  {
    Eigen::VectorXd u=past_inputs.block(istep*m_nin,0,m_nin,1);
    Eigen::VectorXd y= update(u);
  }
  
}

inline void DiscreteStateSpace::setStateFromLastIO(const Eigen::Ref< Eigen::VectorXd > inputs, const Eigen::Ref< Eigen::VectorXd > outputs)
{
  Eigen::VectorXd past_inputs(m_order*m_nin);
  Eigen::VectorXd past_outputs(m_order*m_nout);
  
  for (unsigned int idx=0;idx<m_order;idx++)
  {
    past_inputs.block(idx*m_nin,0,m_nin,1)=inputs;
    past_outputs.block(idx*m_nin,0,m_nin,1)=outputs;
  }
  setStateFromIO(past_inputs,past_outputs);
}

inline void DiscreteStateSpace::setStateFromLastIO(const double& input, const double& output)
{
  assert(m_nin==1);
  assert(m_nout==1);

  Eigen::VectorXd u(1);
  u(0)=input;

  Eigen::VectorXd y(1);
  y(0)=output;

  setStateFromLastIO(u,y);
}

inline bool DiscreteStateSpace::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  Eigen::MatrixXd A,B,C,D;
  std::string type;
  if (nh.hasParam(name+"/type"))
  {
    if (!nh.getParam(name+"/type",type))
    {
      ROS_ERROR("%s/type is not a string",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_DEBUG("%s/type is not defined, using state-space",name.c_str());
    type="state-space";
  }

  if (!type.compare("unity"))
  {
    A.resize(1,1);
    B.resize(1,1);
    C.resize(1,1);
    D.resize(1,1);
    A.setZero();
    B.setZero();
    C.setZero();
    D(0,0)=1;
    setMatrices(A,B,C,D);

    return true;
  }

  std::string what;
  if (!rosparam_utilities::getParam(nh, name+"/A", A, what))
  {
    ROS_ERROR("[DiscreteStateSpace] cannot find '%s/A' parameter!",name.c_str());
    return false;
  }
  if (!rosparam_utilities::getParam(nh, name+"/B", B, what))
  {
    ROS_ERROR("[DiscreteStateSpace] cannot find '%s/B' parameter!",name.c_str());
    return false;
  }
  if (!rosparam_utilities::getParam(nh, name+"/C", C, what))
  {
    ROS_ERROR("[DiscreteStateSpace] cannot find '%s/C' parameter!",name.c_str());
    return false;
  }
  if (!rosparam_utilities::getParam(nh, name+"/D", D, what))
  {
    ROS_ERROR("[DiscreteStateSpace] cannot find '%s/D' parameter!",name.c_str());
    return false;
  }
  setMatrices(A,B,C,D);
  return true;
}


inline double DiscreteStateSpace::update(const double& input)
{
  assert(m_nin==1);
  assert(m_nout==1);
  
  Eigen::VectorXd u(1);
  u(0)=input;
  m_output=update(u);
  return m_output(0);
  
}

inline Eigen::MatrixXd DiscreteStateSpace::computeInputToOutputMatrix()
{
  
  Eigen::MatrixXd io_matrix(m_nout*m_order,m_nin*m_order);
  io_matrix.setZero();
  for (unsigned int idx=0;idx<m_order;idx++)
    io_matrix.block(idx*m_nout,idx*m_nin,m_nout,m_nin)=m_D;
  
  Eigen::MatrixXd powA(m_order,m_order);
  powA.setIdentity();
  
  for (unsigned int idx=1;idx<m_order;idx++)
  {
    for (unsigned int idx2=0;idx2<(m_order-idx);idx2++)
    {
      io_matrix.block((idx+idx2)*m_nout,(idx2)*m_nin,m_nout,m_nin)=m_C*powA*m_B;
    }
    powA*=m_A;
  }
  return io_matrix;
}

inline Eigen::MatrixXd DiscreteStateSpace::computeObservatibilityMatrix()
{
  Eigen::MatrixXd obsv(m_nout*m_order,m_order);
  obsv.setZero();
  
  Eigen::MatrixXd pow_a;
  pow_a.resize(m_order,m_order);
  pow_a.setIdentity();
  
  for (unsigned int idx=0;idx<m_order;idx++)
  {
    obsv.block(idx*m_nout,0,m_nout,m_order)=m_C*pow_a;
    pow_a=pow_a*m_A;
    
  }
  
  return obsv;
}

inline void DiscreteStateSpace::print()
{
  ROS_INFO("system with %u inputs, %u states, %u outputs",m_nin,m_order,m_nout);
  ROS_INFO_STREAM("A:\n"<< m_A);
  ROS_INFO_STREAM("B:\n"<< m_B);
  ROS_INFO_STREAM("C:\n"<< m_C);
  ROS_INFO_STREAM("D:\n"<< m_D);
  ROS_INFO_STREAM("output:\n"<< m_output);
  ROS_INFO_STREAM("state:\n"<< m_state);

}

}

#endif
