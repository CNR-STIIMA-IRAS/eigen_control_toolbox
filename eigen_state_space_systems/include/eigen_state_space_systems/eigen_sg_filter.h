#ifndef   eigen_sg_filters_201805060814
#define   eigen_sg_filters_201805060814

#include <eigen_state_space_systems/eigen_fir_filters.h>

namespace eigen_control_toolbox
{
  
  class SavitkyGolay: public FirFilter
  {
  protected:
    void computeCoeffs(const unsigned int& polynomial_order,
                       const unsigned int& window,
                       const unsigned int& output_size);
    
    void initSavitkyGolay(
      const double& natural_frequency,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
    void initSavitkyGolay(
      const unsigned int& window,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
  public:
    SavitkyGolay(){};
    SavitkyGolay( const double& natural_frequency,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    SavitkyGolay( const unsigned int& window,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    
  };
  
  class CausalSavitkyGolay: public FirFilter
  {
  protected:
    void computeCoeffs(const unsigned int& polynomial_order,
                       const unsigned int& window,
                       const unsigned int& output_size);
    
    void initCausalSavitkyGolay(
      const double& natural_frequency,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
    void initCausalSavitkyGolay(
      const unsigned int& window,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
  public:
    CausalSavitkyGolay(){};
    CausalSavitkyGolay( const double& natural_frequency,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
                      );
    CausalSavitkyGolay( const unsigned int& window,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    
  };
  
}
#include <eigen_state_space_systems/eigen_sg_filter_impl.h>
#endif
