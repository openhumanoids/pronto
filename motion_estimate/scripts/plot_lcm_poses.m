function plot_lcm(pos , quat, id, name, type, reset, draw_links, link_id)

% Supported Pose Type Values
% POSE=1, 
% TREE=2, 
% SQUARE=3, 
% POSE3D=4, 
% AXIS3D=5, 
% TAG=6, 
% CAMERA=7, 
% TRIANGLE=8, 
% HEXAGON=9, 
% SONARCONE=10;

%m = vs.utime_two_t();
%lc = lcm.lcm.LCM.getSingleton();
%m.utime_sim =10;
%m.utime_wall =20;
%lc.publish('OBJ_COLLECTION', m);


m = vs.object_collection_t();
m.objects = javaArray('vs.object_t', size(pos, 1));
for i=1:size(pos,1)
  msg2 = vs.object_t();
  msg2.id= i-1;
  msg2.x= pos(i,1);
  msg2.y= pos(i,2);
  msg2.z= pos(i,3);
  msg2.qw= quat(i,1);
  msg2.qx= quat(i,2);
  msg2.qy= quat(i,3);
  msg2.qz= quat(i,4);
  m.objects(i) =msg2;
end

m.id =id;
m.type =type;
m.name =name;
m.reset =logical(reset);
m.nobjects = size(pos,1);

lc = lcm.lcm.LCM.getSingleton();
lc.publish('OBJECT_COLLECTION', m);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% LINKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (draw_links)
link_col = vs.link_collection_t();
link_col.id = link_id;
link_col.name = [name ' (L)'] ;
link_col.reset =logical(reset);

link_col.links =javaArray('vs.link_t', size(pos-1, 1));
for i=2:size(pos,1)
  link = vs.link_t();
  link.id = i-1;
  link.collection1 = id;
  link.id1 = i-2;
  link.collection2 = id;
  link.id2 = i-1;
  link_col.links(i-1)=link;
end 
link_col.nlinks=size(pos,1)-1;

lc.publish('LINK_COLLECTION',link_col)
end