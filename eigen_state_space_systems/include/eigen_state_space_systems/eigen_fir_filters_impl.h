#ifndef   eigen_fir_filters_impl_201811280952
#define   eigen_fir_filters_impl_201811280952

// #include <eigen_state_space_systems/eigen_fir_filters.h>

namespace eigen_control_toolbox
{
inline FirFilter::FirFilter()
{

}

inline FirFilter::FirFilter(Eigen::MatrixXd coeffs)
{
  computeMatrices(coeffs);
}

inline void FirFilter::computeMatrices(Eigen::MatrixXd coeffs)
{
  m_order=coeffs.cols();
  m_nout=coeffs.rows();
  m_nin=1;
  m_A.resize(m_order,m_order);
  m_B.resize(m_order,m_nin);
  m_C.resize(m_nout,m_order);
  m_D.resize(m_nout,m_nin);
  
  m_A.setZero();
  m_A.block(1,0,m_order-1,m_order-1).setIdentity();
  m_B.setZero();
  m_B(0)=1;
  m_C=coeffs.block(0,1,m_nout,m_order-1);
  m_D=coeffs.block(0,0,m_nout,1);

  
}


inline Eigen::VectorXd FirFilter::update(const Eigen::Ref< Eigen::VectorXd > input)
{
  if (input.rows() != m_nin)
    throw std::invalid_argument("[ DiscreteStateSpace ] Matrix input has wrong rows dimension");
  
  m_output = m_C*m_state + m_D*input;
  m_state.block(1,0,m_order-1,1)=m_state.block(0,0,m_order-1,1);
  m_state(0)=input(0);
  
  return m_output;
}

inline void FirFilter::setStateFromIO(const Eigen::Ref< Eigen::VectorXd > past_inputs, const Eigen::Ref< Eigen::VectorXd > past_outputs)
{
  if (past_inputs.rows() != m_nin*m_order)
    throw std::invalid_argument(("[ DiscreteStateSpace ]  past_inputs has wrong dimenstion: "+std::to_string(past_inputs.rows())+" instead of "+std::to_string(m_nin*m_order)).c_str());
  m_state=past_inputs;
}





}

#endif
