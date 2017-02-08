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
#include <lcmtypes/bot_core/ins_t.hpp>
#include <lcmtypes/pronto/controller_status_t.hpp>
#include <lcmtypes/pronto/behavior_t.hpp>


namespace MavStateEst {
  
class YawLockHandler {
public:
  typedef enum {
    MODE_YAWBIAS, MODE_YAW, MODE_YAWBIAS_YAW
  } YawLockMode;

  YawLockHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames);
  RBISUpdateInterface * processMessage(const bot_core::joint_state_t *msg, MavStateEstimator* state_estimator);

  // Classes:
  YawLock* yaw_lock_;

  // Ancillary handler
  void insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg);

  void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::controller_status_t* msg);
  void robotBehaviorHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::behavior_t* msg);


  YawLockMode mode;

  // Utilities
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_recv_boost;
  boost::shared_ptr<lcm::LCM> lcm_pub_boost;
  boost::shared_ptr<ModelClient> model_boost;
  BotFrames* frames;

  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;

  BotTrans ins_to_body;
  double body_gyro[3];
  
  // Required because IHMC's 'standing' state is reported during walking
  // This is used to infer when it is truely standing
  int64_t last_ihmc_walking_utime;

};


}
#endif /* RBIS_YAWLOCK_LIB_UPDATE_HPP_ */