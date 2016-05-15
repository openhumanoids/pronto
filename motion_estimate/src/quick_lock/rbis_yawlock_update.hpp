#ifndef RBIS_YAWLOCK_LIB_UPDATE_HPP_
#define RBIS_YAWLOCK_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <model-client/model-client.hpp>

#include <string>

#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>

#include "yawlock.hpp"

#include <lcmtypes/bot_core/joint_state_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/pronto/behavior_t.hpp>


namespace MavStateEst {
  
class YawLockHandler {
public:

  YawLockHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames);
  RBISUpdateInterface * processMessage(const bot_core::joint_state_t *msg, RBIS state, RBIM cov);

  // Classes:
  YawLock* yaw_lock_;

  // Ancillary handler
  void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
  void robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg);

  // Utilities
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_recv_boost;
  boost::shared_ptr<lcm::LCM> lcm_pub_boost;
  boost::shared_ptr<ModelClient> model_boost;
  BotFrames* frames;

  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;
};


}
#endif /* RBIS_YAWLOCK_LIB_UPDATE_HPP_ */