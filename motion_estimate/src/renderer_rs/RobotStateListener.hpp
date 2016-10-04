#ifndef RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP
#define RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "GlKinematicBody.hpp"
#include "InteractableGlKinematicBody.hpp"
#include "file_access_utils.hpp"
#include "eigen_kdl_conversions.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/robot_urdf_t.hpp"
#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>

#include <Eigen/Dense>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListenerX
  {
    //--------fields
  public:  
    std::string _robot_name;
  private:  
    std::string _urdf_xml_string; 

    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;   

    //get rid of this
    BotViewer *_viewer;
    
    bool _urdf_parsed;
  
    std::vector<std::string> _jointdof_filter_list;
  public:  
    bool _urdf_subscription_on;
    int64_t _last_state_msg_sim_timestamp; 
    int64_t _last_state_msg_system_timestamp; 
    bot_core::robot_state_t _received_endpose;

    std::map<std::string,Eigen::Vector3f> ee_forces_map;
    std::map<std::string,Eigen::Vector3f> ee_torques_map;
   
    // Color of the robot rendering:
    float _robot_color[3];

    //----------------constructor/destructor
  public:
    RobotStateListenerX(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer, int operation_mode);
    ~RobotStateListenerX();

    boost::shared_ptr<visualization_utils::InteractableGlKinematicBodyX> _gl_robot;
    
    
    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const bot_core::robot_state_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  bot_core::robot_urdf_t* msg); 
    
    
   public:
   
  }; //class RobotStateListener



#endif //RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP
