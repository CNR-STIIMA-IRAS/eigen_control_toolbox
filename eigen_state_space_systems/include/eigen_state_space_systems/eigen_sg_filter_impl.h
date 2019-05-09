#ifndef   eigen_sg_filters_impl_201811280955
#define   eigen_sg_filters_impl_201811280955

// #include <eigen_state_space_systems/eigen_sg_filter.h>

namespace eigen_control_toolbox
{

inline SavitkyGolay::SavitkyGolay( const double& natural_frequency,
                            const double& sample_period,
                            const unsigned int& polynomial_order,
                            const unsigned int& output_size
)
{
  initSavitkyGolay(natural_frequency,sample_period,polynomial_order,output_size);
}

inline SavitkyGolay::SavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  initSavitkyGolay(window,sample_period,polynomial_order,output_size);
}

inline void SavitkyGolay::initSavitkyGolay(const double& natural_frequency, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  
  assert(sample_period>0);
  assert(natural_frequency>0);
  assert(output_size>(polynomial_order+1));
  m_sampling_period=sample_period;
  double shannon_freq=2*M_PI/sample_period;
  double adim_freq=natural_frequency/shannon_freq;
  
  // adim_freq~=(polynomial_order+1)/(3.2*window-4.6) from "On the frequency-domain properties of Savitzsky-Golay filters" R.W. Schafer, HP laboratories
  unsigned int half_window=std::ceil((4.6+(polynomial_order+1)/adim_freq)/3.2);
  unsigned int window=2*half_window+1;
  
  computeCoeffs(polynomial_order,window,output_size);
}

inline void SavitkyGolay::initSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  assert(window>polynomial_order);
  assert(sample_period>0);
  assert(output_size>(polynomial_order+1));
  computeCoeffs(polynomial_order,window,output_size);
}


inline void SavitkyGolay::computeCoeffs(const unsigned int& polynomial_order, const unsigned int& window, const unsigned int& output_size)
{
  Eigen::VectorXd time(window);
  unsigned int half_window=(window-1)/2;
  for (unsigned int idx=0;idx<window;idx++)
    time(idx)=(half_window-idx)*m_sampling_period;
  
  Eigen::MatrixXd regr(window,polynomial_order+1);
  for (unsigned int idx=0;idx<(polynomial_order+1);idx++)
    regr.col(idx)=time.array().pow(idx);
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(regr, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd coeffs=svd.solve(Eigen::MatrixXd::Identity(window,window));
  coeffs=coeffs.block(0,0,output_size,coeffs.cols());
  FirFilter::computeMatrices(coeffs);
  
}

inline CausalSavitkyGolay::CausalSavitkyGolay(const double& natural_frequency,
                                       const double& sample_period,
                                       const unsigned int& polynomial_order,
                                       const unsigned int& output_size
                                      )
{
  initCausalSavitkyGolay(natural_frequency,sample_period,polynomial_order,output_size);
}

inline CausalSavitkyGolay::CausalSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  initCausalSavitkyGolay(window,sample_period,polynomial_order,output_size);
}

inline void CausalSavitkyGolay::initCausalSavitkyGolay(const double& natural_frequency, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{

  assert(sample_period>0);
  assert(natural_frequency>0);
  assert(output_size>(polynomial_order+1));
  m_sampling_period=sample_period;
  double shannon_freq=2*M_PI/sample_period;
  double adim_freq=natural_frequency/shannon_freq;
  
  // adim_freq~=(polynomial_order+1)/(3.2*window-4.6) from "On the frequency-domain properties of Savitzsky-Golay filters" R.W. Schafer, HP laboratories
  unsigned int half_window=std::ceil((4.6+(polynomial_order+1)/adim_freq)/3.2);
  unsigned int window=2*half_window+1;
  
  computeCoeffs(polynomial_order,window,output_size);
}

inline void CausalSavitkyGolay::initCausalSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  assert(window>polynomial_order);
  assert(sample_period>0);
  assert(output_size>(polynomial_order+1));
  computeCoeffs(polynomial_order,window,output_size);
}

inline void CausalSavitkyGolay::computeCoeffs(const unsigned int& polynomial_order, const unsigned int& window, const unsigned int& output_size)
{
  Eigen::VectorXd time(window);
  for (unsigned int idx=0;idx<window;idx++)
    time(idx) = -(idx*m_sampling_period );
  
  Eigen::MatrixXd regr(window,polynomial_order+1);
  for (unsigned int idx=0;idx<(polynomial_order+1);idx++)
  {
    regr.col(idx)=time.array().pow(idx);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(regr, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd coeffs=svd.solve(Eigen::MatrixXd::Identity(window,window));
  coeffs=coeffs.block(0,0,output_size,coeffs.cols());
  FirFilter::computeMatrices(coeffs);
  
}


}
#endif
