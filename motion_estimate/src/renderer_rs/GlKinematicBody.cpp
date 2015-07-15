#include "GlKinematicBody.hpp"


using namespace std;
using namespace boost;
using namespace visualization_utils;

// copy constructor
/*GlKinematicBody::GlKinematicBody( const GlKinematicBody& other )
{
   //*this = other;

  urdf::Model model; 
  if (!model.initString(other._urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }

 _joint_names_= other._joint_names_;
 _jointlimit_min=other._jointlimit_min;
 _jointlimit_max=other._jointlimit_max;
 _current_jointpos= other._current_jointpos;
 _future_jointpos= other._future_jointpos;

  // get root link
  boost::shared_ptr<const urdf::Link> root_link=model.getRoot();
  _root_name = root_link->name;
  if(!root_link->inertial){
   cerr << "WARNING: root link has no inertia, Adding small inertia" << endl;
   model.root_link_->inertial.reset(new urdf::Inertial);
   model.root_link_->inertial->mass = 0.01;
   model.root_link_->inertial->ixx = 0.01;
   model.root_link_->inertial->iyy = 0.01;
   model.root_link_->inertial->izz = 0.01;
  }
  
  // Parse KDL tree
  KDL::Tree tree;
  //if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  if (!kdl_parser::treeFromUrdfModel(model,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
    return;
  }

  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  _links_map =  model.links_;
  _mesh_model_map = other._mesh_model_map;
  _mesh_map= other._mesh_map;
  _unique_root_geometry_name=other._unique_root_geometry_name;

} */

//constructor
GlKinematicBody::GlKinematicBody(string &urdf_xml_string): initialized(false),
_T_world_body(KDL::Frame::Identity()),
isShowMeshSelected(false),isMateable(false),enforce_joint_limits(true),enforce_joint_limits_future_state(true)
{  
  //cout << "GLKinematicBody Constructor" << endl;

  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model model; 
  if (!model.initString(urdf_xml_string))
  {
    cerr << "ERROR: Could not generate robot model" << endl;
  }
//  boost::shared_ptr<urdf::Model> model_ptr(new urdf::Model); 
//  model_ptr->initString(urdf_xml_string)
  
  //enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
  typedef map<string, shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = model.joints_.begin(); it!= model.joints_.end(); it++)
  { 

      string joint_name = it->first;
      size_t found = joint_name.find("mate::start"); 
      if (found!=std::string::npos){
       _mate_start_link = it->second->parent_link_name;
       isMateable = true;
       }
      found = joint_name.find("mate::end"); 
      if (found!=std::string::npos){
       _mate_end_link = it->second->child_link_name;
        isMateable = true;
       } 
  
  
    if(it->second->type!=urdf::Joint::FIXED){ // All joints that not of the type FIXED.
      _joint_names_.push_back(it->first);
      if((it->second->type==urdf::Joint::REVOLUTE)||(it->second->type==urdf::Joint::PRISMATIC)){
        if(it->second->limits){
          _jointlimit_min.insert(make_pair(it->first, it->second->limits->lower));
          _jointlimit_max.insert(make_pair(it->first, it->second->limits->upper));
        }
      }
      _current_jointpos.insert(make_pair(it->first, 0)); //initialize 
    }
  }
  
  // get root link
  boost::shared_ptr<const urdf::Link> root_link=model.getRoot();
  _root_name = root_link->name;
  if(!root_link->inertial){
   cerr << "WARNING: root link has no inertia, Adding small inertia" << endl;
   model.root_link_->inertial.reset(new urdf::Inertial);
   model.root_link_->inertial->mass = 0.01;
   model.root_link_->inertial->ixx = 0.01;
   model.root_link_->inertial->iyy = 0.01;
   model.root_link_->inertial->izz = 0.01;
  }
  
  // Parse KDL tree
  KDL::Tree tree;
  //if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  if (!kdl_parser::treeFromUrdfModel(model,tree))
  {
    cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
    return;
  }

  _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
  
  _links_map =  model.links_;
  
  typedef map<string, shared_ptr<urdf::Link> > links_mapType;
  for(links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
  { 
  
    _link_names.push_back(it->first);
    if(it->second->visual) // atleast one default visual tag exists
    {
      //cout << it->first << endl;
      
      typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
      visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
      for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
      {
         //int type = it->second->visual->geometry->type;
           vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
           int type = visuals[iv]->geometry->type;
    
      //enum {SPHERE, BOX, CYLINDER, MESH}; 

        if  (type == urdf::Geometry::MESH)
        {
          //shared_ptr<urdf::Mesh> mesh(dynamic_pointer_cast<urdf::Mesh>(it->second->visual->geometry));
          shared_ptr<urdf::Mesh> mesh(dynamic_pointer_cast<urdf::Mesh>(visuals[iv]->geometry));
          
          // READ AND STORE DISPLAY LISTS IN MEMORY ONCE
          string file_path = evalMeshFilePath(mesh->filename);
          //storing wavefront_model for different file_paths to prevent repeated creation of the same wavefront model  
          
	  MeshStruct mesh_struct;
	  shared_ptr<MeshStruct> mesh_struct_ptr;

          //typedef std::map<std::string, MeshStruct> mesh_model_map_type_;
	  typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_model_map_type_;
          mesh_model_map_type_::iterator mesh_model_it = _mesh_model_map.find(file_path);
          if(mesh_model_it==_mesh_model_map.end()) // does not exist
          {
            //cout <<"MESH:" << file_path << endl;
            BotWavefrontModel* wavefront_model;
            wavefront_model = bot_wavefront_model_create(file_path.c_str()); 
            
            GLuint dl;
            dl = glGenLists (1);
            glNewList (dl, GL_COMPILE);
            glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);
            //glEnable(GL_LIGHTING);
            bot_wavefront_model_gl_draw(wavefront_model);
            //glDisable(GL_LIGHTING);
            glEndList ();
           
            double minv[3];
            double maxv[3];
	    mesh_struct_ptr = shared_ptr<MeshStruct>(new MeshStruct);
            bot_wavefront_model_get_extrema(wavefront_model, minv, maxv);

	    mesh_struct_ptr->displaylist = dl;            
            mesh_struct_ptr->span_x =mesh->scale.x*(maxv[0] - minv[0]);
            mesh_struct_ptr->span_y =mesh->scale.y*(maxv[1] - minv[1]);
            mesh_struct_ptr->span_z =mesh->scale.z*(maxv[2] - minv[2]);
            mesh_struct_ptr->offset_x = mesh->scale.x*((maxv[0] + minv[0])/2.0);
            mesh_struct_ptr->offset_y = mesh->scale.y*((maxv[1] + minv[1])/2.0);
            mesh_struct_ptr->offset_z = mesh->scale.z*((maxv[2] + minv[2])/2.0);
            bot_wavefront_model_destroy(wavefront_model);
            // remember store in memory
            //_mesh_model_map.insert(make_pair(file_path, mesh_struct));

            _mesh_model_map.insert(make_pair(file_path, mesh_struct_ptr));
          }
          else
          {
          //recall from memory if the a wavefront model for a file path exists.
           //mesh_struct = mesh_model_it->second;
	   mesh_struct_ptr = mesh_model_it->second;
          }
          
          std::stringstream oss;
          oss << it->first << "_"<< iv; 
          string unique_geometry_name = oss.str();

            
         // populate data structures
         //typedef std::map<std::string, MeshStruct> mesh_map_type_;
 	 typedef std::map<std::string, shared_ptr<MeshStruct> > mesh_map_type_;
          mesh_map_type_::iterator mesh_map_it = _mesh_map.find(unique_geometry_name);
          if(mesh_map_it==_mesh_map.end()){
            //_mesh_map.insert(make_pair(unique_geometry_name, mesh_struct));
	    _mesh_map.insert(make_pair(unique_geometry_name, mesh_struct_ptr));
 	 }

        }//end  if  (type == MESH)
        
                  
       if(it->first==_root_name){
           std::stringstream oss;
            oss << it->first << "_"<< iv; 
          _unique_root_geometry_name = oss.str();
       }
      
      }// end for all visuals in a visual group
      
    } // end if(it->second->visual)
  } // end for

   std::map<std::string, double> jointpos_in;
   jointpos_in = _current_jointpos;
}

//destructor
GlKinematicBody::~GlKinematicBody()
{

}



// ========================================================================= 
void GlKinematicBody::set_state(const pronto::robot_state_t &msg)
{

  /*std::map<std::string, double> jointpos_in;
  for (uint i=0; i< (uint) msg.num_joints; i++) //cast to uint to suppress compiler warning
    jointpos_in.insert(make_pair(msg.joint_name[i], msg.joint_position[i])); 
  _current_jointpos.clear();
  _current_jointpos = jointpos_in;  */
  
  // dont clear old just update
   map<string, double>::iterator joint_it;
    for (uint i=0; i< (uint) msg.num_joints; i++){
       joint_it = _current_jointpos.find(msg.joint_name[i]);
       if(joint_it!=_current_jointpos.end())
          joint_it->second = msg.joint_position[i];
    }
    
  //TODO: STORE previous jointpos_in and T_world_body as private members?
  //InteractableGLKinematicBody will provide an method for interactive adjustment.  
    
  KDL::Frame T_world_body;
  T_world_body.p[0]= msg.pose.translation.x;
  T_world_body.p[1]= msg.pose.translation.y;
  T_world_body.p[2]= msg.pose.translation.z;		    
  T_world_body.M =  KDL::Rotation::Quaternion(msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z, msg.pose.rotation.w);
  
  _T_world_body  = T_world_body;  

  
  run_fk_and_update_urdf_link_shapes_and_tfs(_current_jointpos,_T_world_body,false);

  
}


   

// ========================================================================= 

// Shared code
void GlKinematicBody::run_fk_and_update_urdf_link_shapes_and_tfs(std::map<std::string, double> &jointpos_in, const KDL::Frame &T_world_body, bool update_future_frame)
{

   //clear stored data
   if (!update_future_frame){
    _link_names.clear();
    _link_geometry_names.clear();
    _link_tfs.clear();
    _link_geometry_tfs.clear();
    _link_shapes.clear();

    _joint_names.clear();
    _joint_tfs.clear();
    }
    
    // enforce joint limits    
    if((enforce_joint_limits&&!update_future_frame)||(enforce_joint_limits_future_state&&update_future_frame))
    {
      typedef map<string, double > jointpos_mapType;      
      for( jointpos_mapType::iterator it =  jointpos_in.begin(); it!=jointpos_in.end(); it++)
      { 
         map<string,double>::const_iterator joint_it;
         joint_it=_jointlimit_min.find(it->first);
         if(joint_it!=_jointlimit_min.end())
          it->second = std::max(joint_it->second,it->second);
         joint_it=_jointlimit_max.find(it->first);
         if(joint_it!=_jointlimit_max.end())
          it->second = std::min(joint_it->second,it->second); 
      }
    }
    
    std::map<std::string, KDL::Frame > cartpos_out;
    

    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                    //Otherwise returns relative transforms between joints. 

    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);

    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
      cerr << "Error: could not calculate forward kinematics!" <<endl;
      return;
    }

    
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;      
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
    {  		
    
       map<string, KDL::Frame>::const_iterator transform_it;
       transform_it=cartpos_out.find(it->first);
    
    // iterate over add all child joints and store joint origins and axes that are not of type fixed.
    //  std::vector<boost::shared_ptr<Joint> > child_joints;
      for (size_t i = 0; i < it->second->child_joints.size();i++)
      {
        if(it->second->child_joints[i]->type!=urdf::Joint::FIXED)
        {
          KDL::Frame T_parentlink_jointorigin;
          T_parentlink_jointorigin.p[0]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.x;
          T_parentlink_jointorigin.p[1]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.y;
          T_parentlink_jointorigin.p[2]= it->second->child_joints[i]->parent_to_joint_origin_transform.position.z;
          double x,y,z,w;       
          x= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.x;
          y= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.y;
          z= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.z; 
          w= it->second->child_joints[i]->parent_to_joint_origin_transform.rotation.w; 
          T_parentlink_jointorigin.M = KDL::Rotation::Quaternion(x,y,z,w);

          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {
            KDL::Frame T_body_parentlink =  transform_it->second;
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;
            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;
              jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis;
              jointInfo.axis.Normalize();
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                    _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                    _joint_tfs[index].future_axis.Normalize();
                }
              } // end if  (!update_future_frame)     
            
          }
          else if(it->first ==_root_name){
            KDL::Frame T_body_parentlink =  KDL::Frame::Identity();
            KDL::Frame T_world_jointorigin =T_world_body*T_body_parentlink*T_parentlink_jointorigin;

            JointFrameStruct jointInfo;
            jointInfo.name=it->second->child_joints[i]->name;
            jointInfo.parent_link_name=it->second->child_joints[i]->parent_link_name;
            jointInfo.child_link_name=it->second->child_joints[i]->child_link_name;
            if (!update_future_frame){
              jointInfo.frame=T_world_jointorigin;
              jointInfo.axis[0]=it->second->child_joints[i]->axis.x;
              jointInfo.axis[1]=it->second->child_joints[i]->axis.y;
              jointInfo.axis[2]=it->second->child_joints[i]->axis.z;
              jointInfo.axis= T_world_body.M*T_body_parentlink.M*jointInfo.axis;
              jointInfo.axis.Normalize();
              jointInfo.type=it->second->child_joints[i]->type;
              
              //store
              _joint_names.push_back(jointInfo.name);
              _joint_tfs.push_back(jointInfo);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_joint_names.begin(), _joint_names.end(), jointInfo.name);
                if (found != _joint_names.end()) {
                    unsigned int index = found - _joint_names.begin();
                    _joint_tfs[index].future_frame = T_world_jointorigin;
                    jointInfo.future_axis[0]=it->second->child_joints[i]->axis.x;
                    jointInfo.future_axis[1]=it->second->child_joints[i]->axis.y;
                    jointInfo.future_axis[2]=it->second->child_joints[i]->axis.z;
                    _joint_tfs[index].future_axis= T_world_body.M*T_body_parentlink.M*jointInfo.future_axis;
                    _joint_tfs[index].future_axis.Normalize();
                }
              } // end if  (!update_future_frame)    
          }
           
        }// end if (it->second->child_joints[i]->type!=urdf::Joint::FIXED)
      }// end for all child joints
    

      if(it->second->visual)
      {  

        typedef map<string, shared_ptr<vector<shared_ptr<urdf::Visual> > > >  visual_groups_mapType;
        visual_groups_mapType::iterator v_grp_it = it->second->visual_groups.find("default");
        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
          vector<shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
          
          urdf::Pose visual_origin = visuals[iv]->origin;
          KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_visual, T_world_parentjoint;


          //map<string, KDL::Frame>::const_iterator transform_it;
          //transform_it=cartpos_out.find(it->first);

          // usually find fails if base_link has a visual element.
          // Kdl based FK ignores the root link which must be set to body pose
          // manually.
          if(transform_it!=cartpos_out.end())// fk cart pos exists
          {

            T_body_parentjoint= transform_it->second;



            T_parentjoint_visual.p[0]=visual_origin.position.x;
            T_parentjoint_visual.p[1]=visual_origin.position.y;
            T_parentjoint_visual.p[2]=visual_origin.position.z;
            T_parentjoint_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

            T_body_visual  = T_body_parentjoint*T_parentjoint_visual;

            T_world_visual = T_world_body*T_body_visual;
            T_world_parentjoint = T_world_body*T_body_parentjoint;
             //T_world_visual  = T_world_camera*T_camera_body*T_body_visual;
             
             
            std::stringstream oss;
            oss << it->first << "_"<< iv; 
            string unique_geometry_name = oss.str();

            LinkFrameStruct geometry_state,link_state;	    
            if (!update_future_frame){
              geometry_state.name = unique_geometry_name;
              geometry_state.frame = T_world_visual;
              link_state.name = it->first;
              link_state.frame = T_world_parentjoint;
            }

              shared_ptr<urdf::Geometry> geom =  visuals[iv]->geometry;
              //---store
              if (!update_future_frame){
                /*std::vector<std::string>::const_iterator found;
                found = std::find (_link_names.begin(), _link_names.end(), it->first);
                if (found == _link_names.end()) { // if doesnt exist then add*/
                  _link_names.push_back(it->first);  
                  _link_tfs.push_back(link_state);;
                //}
                _link_geometry_names.push_back(unique_geometry_name);
                _link_shapes.push_back(geom);
                _link_geometry_tfs.push_back(geometry_state);
              }
              else{ // update future frame
                std::vector<std::string>::const_iterator found;
                found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
                if (found != _link_geometry_names.end()) {
                    unsigned int index = found - _link_geometry_names.begin();
                    _link_tfs[index].future_frame = T_world_parentjoint;
                    _link_geometry_tfs[index].future_frame = T_world_visual; 
                }
              }


            //cout << "translation  : " << endl;
            //cout << "\t .x  : " << state.tf.translation.x << endl;
            //cout << "\t .y  : " << state.tf.translation.y << endl;
            //cout << "\t .z  : " << state.tf.translation.z << endl;
            //cout << "quaternion" << endl;
            //cout << "\t .x  : " << state.tf.rotation.x << endl;
            //cout << "\t .y  : " << state.tf.rotation.y << endl;
            //cout << "\t .z  : " << state.tf.rotation.z << endl;
            //cout << "\t .w  : " << state.tf.rotation.w << endl;   
            //cout << "\n"<< endl;

          }//if(transform_it!=cartpos_out.end())
          else // root link has visual element (but is not part of fk output)
          {
              urdf::Pose visual_origin = visuals[iv]->origin;
              T_body_visual.p[0]=visual_origin.position.x;
              T_body_visual.p[1]=visual_origin.position.y;
              T_body_visual.p[2]=visual_origin.position.z;
              T_body_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);


              T_world_visual = T_world_body*T_body_visual;
              T_world_parentjoint = T_world_body;
              
              std::stringstream oss;
              oss << it->first << "_"<< iv; 
              string unique_geometry_name = oss.str();
              shared_ptr<urdf::Geometry> geom =  visuals[iv]->geometry;
              LinkFrameStruct geometry_state,link_state;	
              if (!update_future_frame){
                geometry_state.name = unique_geometry_name;
                geometry_state.frame = T_world_visual;
                link_state.name = it->first;
                link_state.frame = T_world_parentjoint;
              }

                //---store
                 if (!update_future_frame){
                 /* std::vector<std::string>::const_iterator found;
                  found = std::find (_link_names.begin(), _link_names.end(), it->first);
                  if (found == _link_names.end()) { // if doesnt exist then add*/
                    _link_names.push_back(it->first);  
                    _link_tfs.push_back(link_state);;
                  //}
                  _link_geometry_names.push_back(unique_geometry_name);  
                  _link_shapes.push_back(geom);
                  _link_geometry_tfs.push_back(geometry_state);
                }
                else{// update future frame
                  std::vector<std::string>::const_iterator found;
                  found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), unique_geometry_name);
                  if (found != _link_geometry_names.end()) {
                    unsigned int index = found - _link_geometry_names.begin();
                    _link_tfs[index].future_frame = T_world_parentjoint;
                    _link_geometry_tfs[index].future_frame = T_world_visual;
                  }
                }

    
         } //end if(transform_it!=cartpos_out.end())
           
        }// end for visual groups   

      }//if(it->second->visual)

    }//end for


}// end run_fk_and_update_urdf_link_shapes_and_tfs


//==============================

bool GlKinematicBody::get_link_geometry_frame(const std::string &link_geometry_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
      if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        state = _link_geometry_tfs[index];  
        T_world_link= state.frame;       
        return true;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       // std::cerr << "ERROR:"<< link_geometry_name << " not found in _link_geometry_names" << std::endl;
       return false;
      }
}  




bool GlKinematicBody::get_link_frame(const std::string &link_name, KDL::Frame &T_world_link)
{
      LinkFrameStruct state;	    

     // retrieve T_world_link from store
      std::vector<std::string>::const_iterator found;
      found = std::find (_link_names.begin(), _link_names.end(), link_name);
      if (found != _link_names.end()) {
        unsigned int index = found - _link_names.begin();
        state = _link_tfs[index];  
        T_world_link= state.frame; 
        return true;
      } 
      else 
      {  
       T_world_link = KDL::Frame::Identity();
       // std::cerr << "ERROR:"<< link_name << " not found in _link_names" << std::endl;
       return false;
      }
}  


bool GlKinematicBody::get_link_geometry(const std::string &link_geometry_name, boost::shared_ptr<urdf::Geometry> &link_geom)
{

    std::vector<std::string>::const_iterator found;
    found = std::find (_link_geometry_names.begin(), _link_geometry_names.end(), link_geometry_name);
    if (found != _link_geometry_names.end()) {
        unsigned int index = found - _link_geometry_names.begin();
        link_geom = _link_shapes[index]; 
        return true;
    }
    else 
     return false; 

}


bool GlKinematicBody::get_joint_info(const std::string &joint_name, JointFrameStruct &jointinfo_struct)
{
    std::vector<std::string>::const_iterator found;
    found = std::find (_joint_names.begin(), _joint_names.end(), joint_name);
    if (found != _joint_names.end()) {
        unsigned int index = found - _joint_names.begin();
        jointinfo_struct = _joint_tfs[index]; 
        return true;
    }
    else 
     return false; 
}

bool GlKinematicBody::get_current_joint_pos(const std::string &joint_name, double &pos)
{
    pos= 0;
    std::map<std::string, double >::const_iterator jointpos_map_it;
    jointpos_map_it = _current_jointpos.find(joint_name); 
    if(jointpos_map_it!=_current_jointpos.end()) 
    { 
      pos = jointpos_map_it->second;
      return true;
    }
    else 
     return false; 
}


bool GlKinematicBody::get_mesh_struct(const std::string &link_geometry_name, MeshStruct &mesh_struct) 
{
  //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
  std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
  mesh_map_it = _mesh_map.find(link_geometry_name);
  if(mesh_map_it!=_mesh_map.end()) 
  { 
   //mesh_struct = mesh_map_it->second;
    mesh_struct = *mesh_map_it->second;
    return true;
  }
  else 
   return false; 
}


//===============================================================================================
// DRAWING METHODS
//
void GlKinematicBody::draw_link(shared_ptr<urdf::Geometry> link, const std::string &nextTfname, const KDL::Frame &nextTfframe,float (&c)[3], float alpha)
{

  //--get rotation in angle/axis form
  double theta;
  double axis[3];
  double x,y,z,w;
  nextTfframe.M.GetQuaternion(x,y,z,w);
  double quat[4] = {w,x,y,z};
  bot_quat_to_angle_axis(quat, &theta, axis);
  
 GLUquadricObj* quadric = gluNewQuadric();
 gluQuadricDrawStyle(quadric, GLU_FILL);
 gluQuadricNormals(quadric, GLU_SMOOTH);
 gluQuadricOrientation(quadric, GLU_OUTSIDE);

 int type = link->type ;
  //enum {SPHERE, BOX, CYLINDER, MESH}; 
  if (type == urdf::Geometry::SPHERE)
    {
      shared_ptr<urdf::Sphere> sphere(dynamic_pointer_cast<urdf::Sphere>(link));	
      double radius = sphere->radius;
       glPushMatrix();
       glTranslatef(nextTfframe.p[0], nextTfframe.p[1], nextTfframe.p[2]);
      if(radius < 0.01)
       glutSolidSphere(radius,6,6);
      else
       glutSolidSphere(radius,36,36); 
	    //drawSphere(6,  radius);
       glPopMatrix();
    
    }
  else if  (type == urdf::Geometry::BOX)
    {
    shared_ptr<urdf::Box> box(dynamic_pointer_cast<urdf::Box>(link));
    double xDim = box->dim.x;
    double yDim = box->dim.y;
    double zDim = box->dim.z;
  //todo
    glPushMatrix();
        //size cuboid
    
        // move base up so that bottom face is at origin
     // glTranslatef(0,0.5,0.0); 
     glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);

     glRotatef(theta * 180/3.141592654, 
       	 axis[0], axis[1], axis[2]); 
     glScalef(xDim,yDim,zDim);
     bot_gl_draw_cube();
        //cube();
     //glColor4f(1.0,1.0,1.0,alpha);
     //bot_gl_draw_cube_frame();
     //glColor4f(c[0],c[1],c[2],alpha);
    glPopMatrix();
  

  }else if  (type == urdf::Geometry::CYLINDER){
    shared_ptr<urdf::Cylinder> cyl(dynamic_pointer_cast<urdf::Cylinder>(link));

    glPushMatrix();
    double v[] = {0,0, -cyl->length/2.0};
    double result[3];
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 

    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);

    glRotatef(theta * 180/3.141592654, 
    axis[0], axis[1], axis[2]); 

    gluCylinder(quadric,
      cyl->radius,
      cyl->radius,
      (double) cyl->length,
      36,
      1);

    //gluDeleteQuadric(quadric);
    glPopMatrix();

    // drawing two disks to make a SOLID cylinder
    glPushMatrix();  

    v[2] = -(cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
      glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();
    glPushMatrix(); 

    v[2] = (cyl->length/2.0);
    bot_quat_rotate_to(quat,v,result);

    // Translate tf origin to cylinder centre
    glTranslatef(result[0],result[1],result[2]); 
    glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
    glRotatef(theta * 180/3.141592654, 
      axis[0], axis[1], axis[2]); 
    gluDisk(quadric,
      0,
      cyl->radius,
      36,
      1);
    glPopMatrix();

    //cout << "CYLINDER"<< endl;
    //cout << "radius : "<<  cyl->radius << endl;
    //cout << "length : "<<  cyl->length << endl;
    // drawBox(radius,length, it->second -> visual->origin);
  }
  else if  (type == urdf::Geometry::MESH)
    {
    
    //shared_ptr<urdf::Mesh> mesh(dynamic_pointer_cast<urdf::Mesh>(link));
    
     /* size_t found1;
      found1=nextTf.link_name.find("r_");
    if (found1!=std::string::npos)
      {*/
        glPushMatrix();
        
        glTranslatef(nextTfframe.p[0],nextTfframe.p[1],nextTfframe.p[2]);
        glRotatef(theta * 180/3.141592654, 
        axis[0], axis[1], axis[2]); 


        //std::map<std::string, MeshStruct>::const_iterator mesh_map_it;
        std::map<std::string, shared_ptr<MeshStruct> >::const_iterator mesh_map_it;
        mesh_map_it=_mesh_map.find(nextTfname);
        if(mesh_map_it!=_mesh_map.end()) // exists in cache
        { 
         
	    glCallList (mesh_map_it->second->displaylist); 
	
        }

    glPopMatrix();

   // }// end if (found1!=std::string::npos)
  }
  else {
  //cout << "UNKNOWN"<< endl;
  }

  gluDeleteQuadric(quadric);
}
// ======================================================================================================


//===============================================================================================
// MISC. UTILITIES 
//
 std::string GlKinematicBody::evalMeshFilePath(std::string file_path_expression, bool return_convex_hull_path)
  {
    std::string result = "";
    std::string package_path = std::string(getModelsPath()) + "/mit_gazebo_models/"; // getModelsPath gives /drc/software/build/models/
    std::string token_str1 ("package://");
    std::string token_str2 (".");
    size_t found1, found2;
    std::string  file_path= "";
    found1=file_path_expression.find(token_str1);
    if (found1!=std::string::npos) // if string contains package:// as a substring 
    {  
    found2=file_path_expression.find(token_str2);
      if (return_convex_hull_path){
        file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+"_chull.obj"; 
      }else{
        file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+".obj"; 
      }
    }
    else
    file_path = file_path_expression;


    return file_path;
  }

//-------------------------------------------------------------------------------------
  std::string GlKinematicBody::exec(std::string cmd) 
  {
      FILE* pipe = popen(cmd.c_str(), "r");
      if (!pipe) return "ERROR";
      char buffer[128];
      std::string result = "";
      while(!feof(pipe)) {
      	if(fgets(buffer, 128, pipe) != NULL)
      		result += buffer;
      }
      pclose(pipe);
      return result;
  }
//-------------------------------------------------------------------------------------
  std::string GlKinematicBody::evalROSMeshFilePath(std::string file_path_expression) 
  {
      std::string result = "";
      std::string token_str1 ("package://");
      std::string token_str2 ("/");
      size_t found1, found2;
      std::string  file_path= "";
      found1=file_path_expression.find(token_str1);
      if (found1!=std::string::npos) // if string contains package:// as a substring 
      {
       	found2 = file_path_expression.find(token_str2,found1+token_str1.size()+1);
       	std::string package_name = file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size());

       	std::string cmd = "rospack find " + package_name;  
       	std::string  package_path = exec(cmd);

       	file_path = package_path.substr(0,package_path.size()-1) + 	    file_path_expression.substr(found2); 	
      }
      else
      	file_path = file_path_expression;

      return file_path;
  }
  
//-------------------------------------------------------------------------------------  

