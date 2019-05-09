#ifndef   eigen_fir_filters_201805060814
#define   eigen_fir_filters_201805060814

#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{
  class FirFilter: public DiscreteStateSpace
  {
  protected:
  public: 
    FirFilter();
    FirFilter(Eigen::MatrixXd coeffs);
    void computeMatrices(Eigen::MatrixXd coeffs);
    
    /*
     * 
     * void setStateFromIO(const Eigen::Ref<Eigen::VectorXd> past_inputs,
     *                     const Eigen::Ref<Eigen::VectorXd> past_outputs);
     * 
     * Compute state from input output, past inputs and outputs during a time window large as the state
     */
    virtual void setStateFromIO(const Eigen::Ref< Eigen::VectorXd > past_inputs, const Eigen::Ref< Eigen::VectorXd > past_outputs);
    
    
    
    virtual Eigen::VectorXd update(const Eigen::Ref<Eigen::VectorXd> input);
    
  };
  
  
  
}

#include <eigen_state_space_systems/eigen_fir_filters_impl.h>
#endif
