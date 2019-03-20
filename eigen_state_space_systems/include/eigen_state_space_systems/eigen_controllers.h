#ifndef eigen_controllers_201806131212
#define eigen_controllers_201806131212

#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{
  class Controller: public DiscreteStateSpace
  {
  protected:
    Eigen::MatrixXd m_Baw;
    
    
  public:
    Controller();   
    Controller(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D);
    void setMatrices(const Eigen::Ref< Eigen::MatrixXd > A, const Eigen::Ref< Eigen::MatrixXd > B, const Eigen::Ref< Eigen::MatrixXd > C, const Eigen::Ref< Eigen::MatrixXd > D);
    virtual void setAntiWindupMatrix(const Eigen::Ref< Eigen::MatrixXd > D);
    
    virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& controller_name);
    virtual Eigen::VectorXd update(const Eigen::Ref< Eigen::VectorXd > input);
    virtual double update(const double& input);
    void antiwindup(const Eigen::Ref< Eigen::VectorXd > saturated_output, Eigen::Ref< Eigen::VectorXd > unsaturated_output);
    void antiwindup(const double& saturated_output, const double& unsaturated_output);

    void setPI(const double& Kp, const double& Ki, const double& sampling_period);
    bool importPIFromParam(const ros::NodeHandle &nh, const std::string& name);
    bool importProportionalFromParam(const ros::NodeHandle &nh, const std::string &name);

  };  



}


#include <eigen_state_space_systems/eigen_controllers_impl.h>
#endif
