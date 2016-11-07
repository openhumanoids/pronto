#ifndef PRONTO_CONVERSIONS_BOT_CORE_HPP_
#define PRONTO_CONVERSIONS_BOT_CORE_HPP_

#include <Eigen/Dense>
#include <Eigen/StdVector>


namespace pronto
{


static Eigen::Isometry3d getBotTransAsEigen(BotTrans trans){
  Eigen::Isometry3d transE;
  transE.setIdentity();
  transE.translation().x() = trans.trans_vec[0]; 
  transE.translation().y() = trans.trans_vec[1]; 
  transE.translation().z() = trans.trans_vec[2];

  Eigen::Quaterniond quat = Eigen::Quaterniond(trans.rot_quat[0], trans.rot_quat[1], 
                                               trans.rot_quat[2], trans.rot_quat[3]);
  transE.rotate(quat); 
  return transE;
}

static BotTrans getEigenAsBotTrans(Eigen::Isometry3d transE){
  BotTrans trans;
  memset(&trans, 0, sizeof(trans));
  trans.trans_vec[0] = transE.translation().x();
  trans.trans_vec[1] = transE.translation().y();
  trans.trans_vec[2] = transE.translation().z();

  Eigen::Quaterniond quat(transE.rotation());
  trans.rot_quat[0] = quat.w();
  trans.rot_quat[1] = quat.x();
  trans.rot_quat[2] = quat.y();
  trans.rot_quat[3] = quat.z();
  return trans;
}

}


#endif /* PRONTO_CONVERSIONS_BOT_CORE_HPP_ */
