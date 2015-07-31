#include <stdio.h>
#include <glib.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <map>
#include <list>
#include <utility>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include <string>
#include <deque>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <lcm/lcm-cpp.hpp>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

#include "RobotStateListener.hpp"

using namespace std;
#define RENDERER_NAME "State"
#define PARAM_ALPHA "Alpha"

#define ERR(fmt, ...) do { \
  fprintf(stderr, "["__FILE__":%d Error: ", __LINE__); \
  fprintf(stderr, fmt, ##__VA_ARGS__); \
} while (0)


typedef struct
{
  BotRenderer renderer;
  BotViewer *viewer;
  BotGtkParamWidget *pw;  
  GtkWidget* vbox;   

    boost::shared_ptr<lcm::LCM> lcm;

  boost::shared_ptr<RobotStateListenerX> robotStateListener;
    // transparency of the model:
    float alpha;
 

} RendererState;

// ------------------------------ Drawing Functions ------------------------- //
static void _draw(BotViewer *viewer, BotRenderer *r){
  RendererState *self = (RendererState*)r;

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  //-draw 
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  //glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable (GL_RESCALE_NORMAL);

  float c[3] = {self->robotStateListener->_robot_color[0], 
                self->robotStateListener->_robot_color[1],
                self->robotStateListener->_robot_color[2]};
  glColor4f(c[0],c[1],c[2],self->alpha);
  
  float alpha = self->alpha;
  if(self->robotStateListener->_gl_robot)
  {
    self->robotStateListener->_gl_robot->draw_body (c,alpha);
  }
}



static void on_param_widget_changed(BotGtkParamWidget *pw, const char *param, void *user_data) {
  RendererState *self = (RendererState*) user_data;
  bot_viewer_request_redraw (self->viewer);

  self->alpha = bot_gtk_param_widget_get_double(self->pw, PARAM_ALPHA);
}

// ------------------------------ Up and Down ------------------------------- //
static void on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
  RendererState *self = (RendererState*)user_data;
  bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data){
  RendererState *self = (RendererState*)user_data;
  bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void _destroy(BotRenderer *r){
  if (!r) return;

  RendererState *self = (RendererState*)r->user;
  delete self;
}

BotRenderer *renderer_rs_new(BotViewer *viewer, int render_priority, lcm_t *lcm){

  //  RendererState *self = (RendererState*)calloc(1, sizeof(RendererState));
  RendererState *self = (RendererState*)new (RendererState);
  self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
  int   operation_mode = 0;

  self->robotStateListener = boost::shared_ptr<RobotStateListenerX>(new RobotStateListenerX(self->lcm, 
              viewer, operation_mode));  
  
  self->viewer = viewer;
  BotRenderer *r = &self->renderer;
  r->user = self;
  self->renderer.name = "State";
  self->renderer.draw = _draw;
  self->renderer.destroy = _destroy;
  self->alpha = 1;
  
  if (viewer) {
  self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

  self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
  self->vbox = vbox;
  gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
  gtk_widget_show (vbox);

  gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), 
            FALSE, TRUE, 0);  
  
   //   gtk_box_pack_start (GTK_BOX (self->renderer.widget), GTK_WIDGET (self->pw), 
   //             FALSE, TRUE, 0);

    
    bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER,
        0, 1, .01, self->alpha);
    
    gtk_widget_show (GTK_WIDGET (self->pw));
        
    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
  
    // save widget modes:
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);

  }

  //    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
  //    bot_viewer_add_event_handler(viewer, ehandler, render_priority);
  return r;
}

void rs_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm){
  bot_viewer_add_renderer(viewer, renderer_rs_new(viewer,
    render_priority, lcm), render_priority);
}
