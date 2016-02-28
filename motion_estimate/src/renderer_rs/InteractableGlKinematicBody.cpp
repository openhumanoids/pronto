#include "InteractableGlKinematicBody.hpp"
#include <algorithm> // using std::find

using namespace std;
using namespace boost;
using namespace visualization_utils;

// Copy constructors
InteractableGlKinematicBodyX::InteractableGlKinematicBodyX( const InteractableGlKinematicBodyX& other, string unique_name): 
  GlKinematicBody(other),_link_selection_enabled(other._link_selection_enabled),_unique_name(unique_name)
{ 
}

InteractableGlKinematicBodyX::InteractableGlKinematicBodyX(string urdf_xml_string,  bool enable_selection, string unique_name):
  GlKinematicBody(urdf_xml_string),   _link_selection_enabled(enable_selection) ,  _unique_name(unique_name)
{  
}

InteractableGlKinematicBodyX::~InteractableGlKinematicBodyX()
{
}

//======================================================================================================
void InteractableGlKinematicBodyX::set_state(const bot_core::robot_state_t &msg)
{
   GlKinematicBody::set_state(msg);  //code re-use
}


void InteractableGlKinematicBodyX::draw_body (float (&c)[3], float alpha)
{
  if(alpha==0)
    return;

  glColor4f(c[0],c[1],c[2],alpha);
  double t;

  for(uint i = 0; i < _link_geometry_tfs.size(); i++)
  {
    LinkFrameStruct nextTf=_link_geometry_tfs[i];
    std::stringstream oss;
    oss << _unique_name << "_"<< _link_geometry_tfs[i].name; 
    if((_link_selection_enabled)&&(selected_link == oss.str())) 
    {
      glColor4f(0.7,0.1,0.1,alpha);         
    }
    else {
        glColor4f(c[0],c[1],c[2],alpha);
    }

    GlKinematicBody::draw_link_current_and_future(c,alpha,i,nextTf);  
  }
  
}
