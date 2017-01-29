clear all
rpg_add_lcm_path()
lc = lcm.lcm.LCM.getSingleton();

disp( 'Bot Core')
status = bot_core.system_status_t();
status.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
status.system = 1;
status.importance = 1;
status.frequency  = 0;
status.value ='';
lc.publish('SYSTEM_STATUS', status);

disp( 'DRC')
q = drc.controller_status_t();
lc.publish('CONTROLLER_STATUS', q);

disp( 'Maps')
ms = maps.request_t();
ms.channel = 'dfds';
lc.publish('MAPS_REQUEST', ms);

disp( 'Vis')
vss = vs.object_t();
lc.publish('OBJECT_EXAMPLE', vss);
