#ifndef   eigen_state_space_systems_20180506011741
#define   eigen_state_space_systems_20180506011741


#include <ros/ros.h>
#include <eigen_state_space_systems/eigen_control_utils.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>

namespace eigen_control_toolbox
{

  
  class DiscreteStateSpace
  {
  protected:
    unsigned int m_order;
    unsigned int m_nin;
    unsigned int m_nout;
    double m_sampling_period;
    
    Eigen::VectorXd m_state;
    Eigen::VectorXd m_output;
    
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_C;
    Eigen::MatrixXd m_D;

    virtual void setMatrices( const Eigen::Ref<Eigen::MatrixXd> A, 
                      const Eigen::Ref<Eigen::MatrixXd> B, 
                      const Eigen::Ref<Eigen::MatrixXd> C, 
                      const Eigen::Ref<Eigen::MatrixXd> D);
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DiscreteStateSpace(){};
    DiscreteStateSpace( const Eigen::Ref<Eigen::MatrixXd> A, 
                        const Eigen::Ref<Eigen::MatrixXd> B, 
                        const Eigen::Ref<Eigen::MatrixXd> C, 
                        const Eigen::Ref<Eigen::MatrixXd> D);
    
    virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);
    
    Eigen::MatrixXd getAMatrix(){return m_A;};
    Eigen::MatrixXd getBMatrix(){return m_B;};
    Eigen::MatrixXd getCMatrix(){return m_C;};
    Eigen::MatrixXd getDMatrix(){return m_D;};
    Eigen::VectorXd getOutput(){return m_output;};
    Eigen::VectorXd getState(){return m_state;};
    unsigned int getOrder(){return m_order;};
    unsigned int getNumberOfInputs(){return m_nin;};
    unsigned int getNumberOfOutputs(){return m_nout;};
    
    void setState(const Eigen::Ref<Eigen::VectorXd> state);
    
    void print();

    void setSamplingPeriod(const double& sampling_period){m_sampling_period=sampling_period;};
    double getSamplingPeriod(){return m_sampling_period;};
    
    /*
     * 
     * void setStateFromIO(const Eigen::Ref<Eigen::VectorXd> past_inputs,
     *                     const Eigen::Ref<Eigen::VectorXd> past_outputs);
     * 
     * Compute state from input output, past inputs and outputs during a time window large as the state
     */
    virtual void setStateFromIO(const Eigen::Ref< Eigen::VectorXd > past_inputs, const Eigen::Ref< Eigen::VectorXd > past_outputs);
    
    /*
     * 
     * void setStateFromLastIO( const Eigen::Ref<Eigen::VectorXd> inputs,
     *                          const Eigen::Ref<Eigen::VectorXd> outputs);
     * 
     * Compute state from input output, consider past inputs and outputs equal to the last one during a time window large as the state
     */
    void setStateFromLastIO(const Eigen::Ref< Eigen::VectorXd > inputs, const Eigen::Ref< Eigen::VectorXd > outputs);
    
    /*
     *
     * void setStateFromLastIO( const double& input,
     *                          const double& output);
     *
     * Compute state from input output for SISO systems, consider past inputs and outputs equal to the last one during a time window large as the state
     */
    void setStateFromLastIO(const double& input, const double& output);

    
    
        
    virtual Eigen::VectorXd update(const Eigen::Ref<Eigen::VectorXd> input);
    virtual double update(const double& input);
    
    
    Eigen::MatrixXd computeObservatibilityMatrix( );
    
    Eigen::MatrixXd computeInputToOutputMatrix( );
    
  };
  
  using DiscreteStateSpacePtr = std::shared_ptr<DiscreteStateSpace>;
  
}

#include <eigen_state_space_systems/eigen_state_space_systems_impl.h>
#endif
