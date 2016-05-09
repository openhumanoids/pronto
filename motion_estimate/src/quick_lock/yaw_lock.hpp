#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>
#include <pronto_utils/pronto_math.hpp>

#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/bot_core/system_status_t.hpp>
#include <lcmtypes/bot_core/utime_t.hpp>

class YawLock{
  public:
    YawLock(boost::shared_ptr<lcm::LCM> &lcm_, boost::shared_ptr<ModelClient> &model_);

    ~YawLock(){
    }

    void setParameters(int correction_period_in, bool yaw_slip_detect_in, 
        double yaw_slip_threshold_degrees_in, double yaw_slip_disable_period_in){
        correction_period_ = correction_period_in;
        yaw_slip_detect_ = yaw_slip_detect_in;
        yaw_slip_threshold_degrees_ = yaw_slip_threshold_degrees_in;
        yaw_slip_disable_period_ = yaw_slip_disable_period_in;
    }

    void setControllerState(int last_controller_state_in){
      last_controller_state_ = last_controller_state_in;
    }

    void setJointState(std::vector<float> joint_position_in, 
                       std::vector<std::string> joint_name_in){
      joint_angles_init_ = true;
      joint_position_ = joint_position_in;
      joint_name_ = joint_name_in;
    }

    bool getJointAnglesInit(){
        return joint_angles_init_;
    }

    // Determine the actual yaw correction to be made.
    // Input is the current floating base
    // output bool is true if a correction should be made
    // and world_to_body_quat_correction holds the full orientation to be corrected to
    bool getCorrection(Eigen::Isometry3d world_to_body, int64_t body_utime, 
        Eigen::Quaterniond &world_to_body_quat_correction);
    

  private:
    
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    std::vector<std::string> joint_name_;
    std::vector<float> joint_position_;

    // Poses of the l and r feet when robot first became standing or manipulating:
    Eigen::Isometry3d world_to_l_foot_original_, world_to_r_foot_original_;
    bool joint_angles_init_; // have received some joint angles
    bool lock_init_; // is the yaw lock active

    // Used in foot slip detection:
    Eigen::Isometry3d l_foot_to_r_foot_original_;
    int64_t utime_disable_until_;


    int64_t counter_;

    int last_controller_state_;
    double current_yaw_;
    int correction_state_; // state in which we can apply the change

    // Parameters
    int correction_period_;
    bool yaw_slip_detect_;
    double yaw_slip_threshold_degrees_;
    double yaw_slip_disable_period_;

};