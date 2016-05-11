#ifndef RBIS_YAW_LOCK_LIB_UPDATE_HPP_
#define RBIS_YAW_LOCK_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <model-client/model-client.hpp>

#include <string>

#include <mav_state_est/mav-est-legodo/rbis_legodo_common.hpp>

#include <leg_estimate/leg_estimate.hpp>
#include <estimate_tools/torque_adjustment.hpp>

#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/pronto/controller_foot_contact_t.hpp>
#include <lcmtypes/bot_core/six_axis_force_torque_t.hpp>
#include <lcmtypes/bot_core/six_axis_force_torque_array_t.hpp>


namespace MavStateEst {
  
class YawLockHandler {
public:

  YawLockHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, ModelClient* model);
  RBISUpdateInterface * processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov);

  // Classes:
  YawLock* yaw_lock_;

  // Ancillary handler
  void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
 

  // Utilities
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_recv_boost;
  boost::shared_ptr<lcm::LCM> lcm_pub_boost;
  boost::shared_ptr<ModelClient> model_boost;
  
};


}
#endif /* RBIS_YAW_LOCK_LIB_UPDATE_HPP_ */