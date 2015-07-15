#ifndef GL_KINEMATIC_BODY_HPP
#define GL_KINEMATIC_BODY_HPP

#include <boost/function.hpp>
#include <map>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>

#include <lcmtypes/bot_core.hpp>
#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>
#include <bot_frames/bot_frames.h>
#include <Eigen/Dense>
#include <iostream>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
// #include "gl_draw_utils.hpp"
#include "eigen_kdl_conversions.hpp"

namespace visualization_utils {

struct MeshStruct 
{
  GLuint displaylist;
  double span_x;
  double span_y;
  double span_z;
  double offset_x; // vertices are not always defined in local link frame. In the drc robot sdf, the vertices are defined in parent joint coordinates.
  double offset_y;
  double offset_z; 
};

struct  LinkFrameStruct
{
    LinkFrameStruct():frame(KDL::Frame::Identity()), future_frame(KDL::Frame::Identity()) { }
    std::string name;
    KDL::Frame frame;
    KDL::Frame future_frame;
}; 


struct  JointFrameStruct
{
    JointFrameStruct():frame(KDL::Frame::Identity()), future_frame(KDL::Frame::Identity()){ }
    std::string name;
    KDL::Frame frame;
    KDL::Frame future_frame;
    KDL::Vector axis; // in world frame
    KDL::Vector future_axis; // in future_world frame
    std::string parent_link_name;
    std::string child_link_name;
    int type;
}; 
  
class GlKinematicBody
{
  protected: 
    std::string _urdf_xml_string; 
    std::string _root_name;
    std::string _unique_root_geometry_name;
    std::vector<std::string> _joint_names_;
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;

    std::vector<std::string > _link_names; 
    std::vector<boost::shared_ptr<urdf::Geometry> > _link_shapes;
    std::vector<LinkFrameStruct> _link_tfs;    
    
    // adding support for multiple visual geometries for a given link 
    std::vector<std::string > _link_geometry_names; //_link name + id;
    std::vector<LinkFrameStruct> _link_geometry_tfs;
    
    std::vector<std::string > _joint_names; 
    std::vector<JointFrameStruct > _joint_tfs;


   // std::map<std::string, MeshStruct > _mesh_map; // associates link geometry name with meshstruct
  //Avoids loading the same mesh multiple times.
   // std::map<std::string, MeshStruct > _mesh_model_map; // associates file name with meshstruct

    std::map<std::string, boost::shared_ptr<MeshStruct> > _mesh_map; // associates link geometry name with meshstruct
  //Avoids loading the same mesh multiple times.
    std::map<std::string, boost::shared_ptr<MeshStruct> > _mesh_model_map; // associates file name with meshstruct
    
    bool initialized;
   
   public:
    // Constructors and destructor
   // GlKinematicBody( const GlKinematicBody& other );// copy constructor
    // GlKinematicBody(std::string urdfFilename); 
    GlKinematicBody(std::string &urdf_xml_string);
    //GlKinematicBody(boost::shared_ptr<urdf::Model> urdf_instance);
    ~ GlKinematicBody();
    
    //object state
    KDL::Frame _T_world_body; //store position in the world
    
    double _body_mass;
    KDL::Frame _T_world_com;
         
    std::string _mate_start_link;
    std::string _mate_end_link;
    
    //Also store current jointangles map.
    std::map<std::string, double> _current_jointpos;
    
    bool enforce_joint_limits;
    bool enforce_joint_limits_future_state;
    std::map<std::string, double> _jointlimit_min;
    std::map<std::string, double> _jointlimit_max;
    
    void disable_joint_limit_enforcement()
    {
      enforce_joint_limits = false;
    };
    void enable_joint_limit_enforcement()
    {
      enforce_joint_limits = true;
    };   
    void disable_joint_limit_enforcement_for_future_state()
    {
      enforce_joint_limits_future_state = false;
    };
    void enable_joint_limit_enforcement_for_future_state()
    {
      enforce_joint_limits_future_state = true;
    };  
    
    std::vector<KDL::Frame> _desired_body_motion_history;
    
    // mesh for drawing
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3i> triangles;
    bool isShowMeshSelected;
    bool isMateable;

    // state can be set via robot_state_t, or urdf::Model,  or affordance_state_t,  or otdf::ModelInterface;
    void set_state(const pronto::robot_state_t &msg);
    void run_fk_and_update_urdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in,const KDL::Frame &T_world_body, bool update_future_frame);
    
    bool get_state_as_lcm_msg(pronto::robot_state_t &msg_in)
    {
      pronto::position_3d_t pose;
      msg_in.utime = bot_timestamp_now();
      visualization_utils::transformKDLToLCM(_T_world_body,pose);
      msg_in.pose = pose;
      
      pronto::twist_t twist;
      msg_in.twist =twist;

      pronto::force_torque_t force_torque;   
      msg_in.force_torque =force_torque;
      
      msg_in.num_joints = 0;
      typedef std::map<std::string, double > jointpos_mapType;      
      for( jointpos_mapType::iterator it =  _current_jointpos.begin(); it!=_current_jointpos.end(); it++)
      { 
        msg_in.joint_name.push_back(it->first);
        msg_in.joint_position.push_back(it->second);
        msg_in.joint_velocity.push_back(0.0);
        msg_in.joint_effort.push_back(0.0);
        msg_in.num_joints++; 
      }
      return true;
    };
    
    void draw_link(boost::shared_ptr<urdf::Geometry> link,const std::string &nextTfname, const KDL::Frame &nextTfframe,float (&c)[3], float alpha);
    void draw_link_current_and_future(float (&c)[3], float alpha,int link_shape_index, const LinkFrameStruct &nextTf)
    {    
    
        boost::shared_ptr<urdf::Geometry> nextLink = _link_shapes[link_shape_index];
        draw_link(nextLink,nextTf.name, nextTf.frame,c,alpha);
    };
    
    void draw_body (float (&c)[3], float alpha)
    {

     if(alpha==0)
			return;

      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      double t;
     
      
      for(uint i = 0; i < _link_geometry_tfs.size(); i++)
      {
        LinkFrameStruct nextTf = _link_geometry_tfs[i];   
        draw_link_current_and_future(c,alpha,i,nextTf);
      }
      
    };
   
      
    void draw_body_in_frame (float (&c)[3], double alpha, const KDL::Frame &T_drawFrame_currentWorldFrame)
    {

      //glColor3f(c[0],c[1],c[2]);
      glColor4f(c[0],c[1],c[2],alpha);
      for(uint i = 0; i < _link_geometry_tfs.size(); i++)
      {
        KDL::Frame T_currentWorldFrame_link,T_drawFrame_link;
        LinkFrameStruct nextTf;
        
        nextTf = _link_geometry_tfs[i];
        T_currentWorldFrame_link = _link_geometry_tfs[i].frame;
        T_drawFrame_link = T_drawFrame_currentWorldFrame*T_currentWorldFrame_link;
        nextTf.frame = T_drawFrame_link;
        draw_link_current_and_future(c,alpha,i,nextTf);
      }

      
    };
    
    int get_num_joints()
    {
      int val = _joint_names_.size();
      return val;
    };

    std::vector<LinkFrameStruct> get_link_tfs()
    {
      return _link_tfs;
    };
    std::vector<std::string > get_link_names()
    {
      return _link_names;
    };
    std::vector<std::string > get_joint_names()
    {
      return _joint_names_;
    };
        
    
    std::map<std::string, boost::shared_ptr<urdf::Link> > get_links_map()
    {
      return _links_map;
    };
    
   
    std::vector<LinkFrameStruct> get_link_geometry_tfs()
    {
      return _link_geometry_tfs;
    };
    std::vector<std::string > get_link_geometry_names()
    {
      return _link_geometry_names;
    };
    
    std::string get_root_link_name(void) { return _root_name; };
    
    bool get_root_link_geometry_name(std::string &link_geometry_name) { 
      link_geometry_name = _root_name + "_0";
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) { 
        return true;
      }
      return false;
    };
    
    void get_body_mass(double &mass);
    void get_com_frame(KDL::Frame &T_world_com);
    bool get_link_frame(const std::string &link_name, KDL::Frame &T_world_link);
    bool get_link_geometry_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link);
    bool get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<urdf::Geometry> &link_geom);
    bool get_mesh_struct(const std::string &link_geometry_name, MeshStruct &mesh_struct);
    bool get_joint_info(const std::string &joint_name, JointFrameStruct &jointinfo_struct);
    bool get_current_joint_pos(const std::string &joint_name, double &pos);
    void get_whole_body_span_dims(Eigen::Vector3f &whole_body_span,Eigen::Vector3f &offset);
    bool get_link_geometry_span_dims(std::string &link_geometry_name, Eigen::Vector3f &whole_body_span_dims,Eigen::Vector3f &offset);
    
    bool get_associated_link_name(std::string &link_geometry_name,std::string &link_name)
    {
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) { // if doesnt exist then add*/
        unsigned int index = found - _link_geometry_names.begin();
        link_name = _link_names[index];
        return true;
      }
      return false;
    };
    
    bool get_joint_index(std::string &joint_name, unsigned int &index)
    {
      std::vector<std::string>::const_iterator found;
      found = std::find (_joint_names.begin(), _joint_names.end(), joint_name);
      if (found != _joint_names.end()) 
      {
        index = found - _joint_names.begin();   
        return true;
      }
      return false;
    };
    void draw_whole_body_bbox(); 
    void draw_link_geometry_bbox(std::string &link_geometry_name); 
    // Was protected: (mfallon changed this:
    std::string evalMeshFilePath(std::string file_path_expression, bool return_convex_hull_path =false);
  protected:    
    std::string exec(std::string cmd);
    std::string evalROSMeshFilePath(std::string file_path_expression);  
 };
 
} // end namespace 

#endif //GL_KINEMATIC_BODY_HPP
