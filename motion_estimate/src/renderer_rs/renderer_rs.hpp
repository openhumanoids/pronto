#ifndef RENDERER_STATE_HPP_
#define RENDERER_STATE_HPP_

/**
 * Linking: `pkg-config --libs renderer_status`
 * @{
 */

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>

void rs_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm);

#endif /* RENDERER_STATE_HPP_ */

