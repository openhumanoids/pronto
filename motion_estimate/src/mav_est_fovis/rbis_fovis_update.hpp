#ifndef RBIS_FOVIS_LIB_UPDATE_HPP_
#define RBIS_FOVIS_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <pronto_utils/pronto_vis.hpp>
#include <pronto_utils/pronto_conversions_lcm.hpp>
#include <pronto_utils/pronto_conversions_bot_core.hpp>
#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>

#include <lcmtypes/pronto/update_t.hpp>

namespace MavStateEst {

class FovisHandler {
public:
  // Typical mode is MODE_VELOCITY_ROT_RATE
  typedef enum {
    MODE_LIN_RATE, MODE_ROT_RATE, MODE_LIN_AND_ROT_RATE, MODE_LIN
  } FovisMode;

  FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
               BotParam * param, BotFrames * frames);

  RBISUpdateInterface * processMessage(const pronto::update_t  * msg, RBIS state, RBIM cov);

  FovisMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_fovis;
  
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  BotFrames* frames;
  pronto_vis* pc_vis_;
  
  // both duplicated in leg odom
  BotTrans getTransAsVelocityTrans(BotTrans msgT,
           int64_t utime, int64_t prev_utime);  
  
  void sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel);
  
  // Publish Debug Data e.g. velocities
  bool publish_diagnostics_;
  bool verbose_;  
  

  Eigen::Isometry3d prev_t0_body_;
  int64_t prev_t0_body_utime_;
};


}
#endif /* RBIS_FOVIS_LIB_UPDATE_HPP_ */

