#ifndef PRONTO_CONVERSIONS_LCM_HPP_
#define PRONTO_CONVERSIONS_LCM_HPP_

#include <pronto_utils/pronto_math.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>

namespace pronto
{

static Isometry3dTime getPoseAsIsometry3dTime(const bot_core::pose_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << pose->pos[0], pose->pos[1] , pose->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose->orientation[0], pose->orientation[1], 
                                               pose->orientation[2], pose->orientation[3]);
  pose_iso.rotate(quat);
  return Isometry3dTime(pose->utime, pose_iso);
}

static bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t tf;
  tf.utime = utime;
  tf.pos[0] = pose.translation().x();
  tf.pos[1] = pose.translation().y();
  tf.pos[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.orientation[0] = quat.w();
  tf.orientation[1] = quat.x();
  tf.orientation[2] = quat.y();
  tf.orientation[3] = quat.z();
  return tf;
}



// Scale a transform by elapsed time:
// was getTransAsVelocityTrans
static Eigen::Isometry3d getDeltaAsVelocity(Eigen::Isometry3d delta, int64_t dt){
  
  Eigen::Quaterniond quat(delta.rotation());
  double rpy[3];
  quat_to_euler(quat, rpy[0], rpy[1], rpy[2]);

  // NB: I don't believe this is the correct way of calculating this:
  double elapsed_time = (double) dt*1E-6;// ( (double) utime -  prev_utime)/1000000;
  double rpy_rate[3];
  rpy_rate[0] = rpy[0]/elapsed_time;
  rpy_rate[1] = rpy[1]/elapsed_time;
  rpy_rate[2] = rpy[2]/elapsed_time;
  
  /*
  if (verbose){
    std::stringstream ss;
    ss << utime << " delta: ";
    printTrans(delta, ss.str() );  
    std::cout << "Elapsed Time: " << elapsed_time  << " sec\n";
    std::cout << "RPY: " << rpy[0] << ", "<<rpy[1] << ", "<<rpy[2] <<" rad\n";
    std::cout << "RPY: " << rpy[0]*180/M_PI << ", "<<rpy[1]*180/M_PI << ", "<<rpy[2]*180/M_PI <<" deg\n";
    std::cout << "RPY: " << rpy_rate[0] << ", "
                         << rpy_rate[1] << ", "
                         << rpy_rate[2] << " rad/s | velocity scaled\n";
    std::cout << "RPY: " << rpy_rate[0]*180/M_PI << ", "
                         << rpy_rate[1]*180/M_PI << ", "
                         << rpy_rate[2]*180/M_PI << " deg/s | velocity scaled\n";
    std::cout << "XYZ: " << delta.trans_vec[0] << ", "
                         << delta.trans_vec[1] << ", "
                         << delta.trans_vec[2] << "\n";
  }
  */
  
  Eigen::Isometry3d vel;
  vel.setIdentity();
  vel.translation()  << delta.translation().x()/elapsed_time, delta.translation().y()/elapsed_time, delta.translation().z()/elapsed_time;
  Eigen::Quaterniond quat_vel = euler_to_quat(rpy_rate[0], rpy_rate[1], rpy_rate[2]);
  vel.rotate(quat_vel);

  /* 
  if (verbose){
    std::stringstream ss2;
    ss2 << " vel: ";
    printTrans(vel, ss2.str() );
    std::cout << "\n\n";
  }
  */
  
  return vel;
}


// Given an Isometry3d representing velocity,
// inserts the translation components as the velocity components in the pose (for visualization in signal scope)
static bot_core::pose_t getIsometry3dAsBotPoseVelocity(Eigen::Isometry3d pose_iso_velocity, int64_t utime ){
  bot_core::pose_t pose;
  pose.utime = utime;
  pose.pos[0] = 0;
  pose.pos[1] = 0;
  pose.pos[2] = 0;
  pose.orientation[0] = 0;
  pose.orientation[1] = 0;
  pose.orientation[2] = 0;
  pose.orientation[3] = 0;
  pose.vel[0] = pose_iso_velocity.translation().x();
  pose.vel[1] = pose_iso_velocity.translation().y();
  pose.vel[2] = pose_iso_velocity.translation().z();
  pose.rotation_rate[0] = 0;
  pose.rotation_rate[1] = 0;
  pose.rotation_rate[2] = 0;
  return pose;
}

}


#endif /* PRONTO_CONVERSIONS_LCM_HPP_ */
