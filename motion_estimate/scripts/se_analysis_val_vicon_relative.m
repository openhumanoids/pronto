function main()
close all
clear all

global bot_
bot_ = bot;

load run3_summary
end_range = 21500%41500

%load run4_summary
%end_range = 10000




s.v.delta = get_delta_motion(s.v, end_range);
s.m.delta = get_delta_motion(s.m, end_range);
s.b.delta = get_delta_motion(s.b, end_range);
do_plotting(s,end_range)


function delta = get_delta_motion(motion, end_range)
global bot_
% 100 is 1sec
% 10 is 0.1sec
window = 10;


delta.t=[];
delta.trans_vec =[];
delta.rot_quat=[];
delta.rot_rpy=[];
for i=(window+1):end_range
  i_a = i-window;
  t_a.trans_vec=motion.trans_vec(i_a,:);
  t_a.rot_quat=motion.rot_quat(i_a,:);
  t_b.trans_vec=motion.trans_vec(i,:);
  t_b.rot_quat=motion.rot_quat(i,:);
    
  t_ab = transform_relative(t_a, t_b);
  t_ab.rot_rpy = bot_.quat_to_roll_pitch_yaw( t_ab.rot_quat );

  delta.t = [delta.t; motion.t(i)];
  delta.trans_vec = [delta.trans_vec;t_ab.trans_vec];
  delta.rot_quat = [delta.rot_quat;t_ab.rot_quat];
  delta.rot_rpy  = [delta.rot_rpy;t_ab.rot_rpy];
  
end

function do_plotting(s,end_range)
range =1:end_range;

figure; hold on
plot(s.v.trans_vec(range,1), s.v.trans_vec(range,2),'g')
plot(s.v.trans_vec(1,1), s.v.trans_vec(1,2),'c.')
%plot(s.b.rel_v.trans_vec(range,1), s.b.rel_v.trans_vec(range,2),'b')
%plot(s.m.rel_v.trans_vec(range,1), s.m.rel_v.trans_vec(range,2),'m')

plot_delta(s)
plot_delta_error(s)
plot_global(s, range)

function plot_delta(s)

figure
subplot(3,2,1), hold on, title('x')
plot(s.v.delta.t, s.v.delta.trans_vec(:,1),'g')
plot(s.m.delta.t, s.m.delta.trans_vec(:,1),'m')
plot(s.b.delta.t, s.b.delta.trans_vec(:,1),'b')


subplot(3,2,3), hold on, title('y')
plot(s.v.delta.t, s.v.delta.trans_vec(:,2),'g')
plot(s.m.delta.t, s.m.delta.trans_vec(:,2),'m')
plot(s.b.delta.t, s.b.delta.trans_vec(:,2),'b')


subplot(3,2,5), hold on, title('z')
plot(s.v.delta.t, s.v.delta.trans_vec(:,3),'g')
plot(s.m.delta.t, s.m.delta.trans_vec(:,3),'m')
plot(s.b.delta.t, s.b.delta.trans_vec(:,3),'b')


subplot(3,2,2), hold on, title('roll')
plot(s.v.delta.t, s.v.delta.rot_rpy(:,1)*180/pi,'g')
plot(s.m.delta.t, s.m.delta.rot_rpy(:,1)*180/pi,'m')
plot(s.b.delta.t, s.b.delta.rot_rpy(:,1)*180/pi,'b')


subplot(3,2,4), hold on, title('pitch')
plot(s.v.delta.t, s.v.delta.rot_rpy(:,2)*180/pi,'g')
plot(s.m.delta.t, s.m.delta.rot_rpy(:,2)*180/pi,'m')
plot(s.b.delta.t, s.b.delta.rot_rpy(:,2)*180/pi,'b')


subplot(3,2,6), hold on, title('yaw')
plot(s.v.delta.t, s.v.delta.rot_rpy(:,3)*180/pi,'g')
plot(s.m.delta.t, s.m.delta.rot_rpy(:,3)*180/pi,'m')
plot(s.b.delta.t, s.b.delta.rot_rpy(:,3)*180/pi,'b')

function plot_delta_error(s)

s.m.delta_error.trans_vec = abs(s.v.delta.trans_vec(:,:) - s.m.delta.trans_vec(:,:));
s.m.delta_error.rot_rpy = abs(s.v.delta.rot_rpy(:,:) - s.m.delta.rot_rpy(:,:));

s.b.delta_error.trans_vec = abs(s.v.delta.trans_vec(:,:) - s.b.delta.trans_vec(:,:));
s.b.delta_error.rot_rpy = abs(s.v.delta.rot_rpy(:,:) - s.b.delta.rot_rpy(:,:));

figure
subplot(3,2,1), hold on, title('x')
plot(s.m.delta.t, s.m.delta_error.trans_vec(:,1),'m')
%plot(s.b.delta.t, s.b.delta_error.trans_vec(:,1),'b')


subplot(3,2,3), hold on, title('y')
plot(s.m.delta.t, s.m.delta_error.trans_vec(:,2),'m')
%plot(s.b.delta.t, s.b.delta_error.trans_vec(:,2),'b')


subplot(3,2,5), hold on, title('z')
plot(s.m.delta.t, s.m.delta_error.trans_vec(:,3),'m')
%plot(s.b.delta.t, s.b.delta_error.trans_vec(:,3),'b')


subplot(3,2,2), hold on, title('roll')
plot(s.m.delta.t, s.m.delta_error.rot_rpy(:,1)*180/pi,'m')
%plot(s.b.delta.t, s.b.delta_error.rot_rpy(:,1)*180/pi,'b')


subplot(3,2,4), hold on, title('pitch')
plot(s.m.delta.t, s.m.delta_error.rot_rpy(:,2)*180/pi,'m')
%plot(s.b.delta.t, s.b.delta_error.rot_rpy(:,2)*180/pi,'b')


subplot(3,2,6), hold on, title('yaw')
plot(s.m.delta.t, s.m.delta_error.rot_rpy(:,3)*180/pi,'m')
%plot(s.b.delta.t, s.b.delta_error.rot_rpy(:,3)*180/pi,'b')

function plot_global(s, range)

figure
subplot(2,2,1), hold on
plot(s.v.t(range), s.v.trans_vec(range,1),'g')
title('Global x')

subplot(2,2,2), hold on
plot(s.v.t(range), s.v.trans_vec(range,2),'g')
title('Global y')

subplot(2,2,3), hold on
plot(s.v.t(range), s.v.trans_vec(range,3),'g')
title('Global z')

subplot(2,2,4)
plot(s.v.t(range), s.v.rot_rpy(range,3)*180/pi,'g')
title('Global yaw (k)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate ab = inv(a) * b
%pose_a.trans_vec = [1,0,0]; pose_a.rot_quat = [1,0,0,0];
%pose_b.trans_vec = [3,0,0]; pose_b.rot_quat = [1,0,0,0];
%pose_ab =transform_relative(pose_a, pose_b)
function pose_ab =transform_relative(pose_a, pose_b)
global bot_
pose_ab = bot_.trans_apply_trans( pose_b,bot_.trans_invert(pose_a));
