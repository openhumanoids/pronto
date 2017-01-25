close all; clear all

global bot_
bot_ = bot;

d =load('log_gt_se');
d(:,1) = (d(:,1) - d(1,1))*1e-6;


plot(d(:,2),d(:,3))
hold on 
plot(d(:,10),d(:,11),'r')
axis equal

offset = 44
dt = mean(diff(d(:,1)))*offset;



gt.trans_vec =  d(:,[2,3,4]);
gt.rot_quat =  d(:,[5,6,7,8]);
gt.distance_travelled = [0;sqrt(sum(diff(d(:,[2:4])).^2,2))]
gt.cum_dist_travelled = cumsum(gt.distance_travelled)
gt.rel = transform_relative_list(gt,offset);

se.trans_vec =  d(:,[10:12]);
se.rot_quat =  d(:,[13:16]);
se.distance_travelled = [0;sqrt(sum(diff(d(:,[10:12])).^2,2))]
se.cum_dist_travelled = cumsum(se.distance_travelled);
se.rel = transform_relative_list(se,offset);


figure; 
subplot(4,1,1);hold on; ylabel('X change, m')
plot(d(:,1), gt.rel.trans_vec(:,1))
plot(d(:,1), se.rel.trans_vec(:,1),'r')
subplot(4,1,2);hold on; ylabel('Y change, m')
plot(d(:,1), gt.rel.trans_vec(:,2))
plot(d(:,1), se.rel.trans_vec(:,2),'r')
subplot(4,1,3);hold on; ylabel('Z change, m')
plot(d(:,1), gt.rel.trans_vec(:,3))
plot(d(:,1), se.rel.trans_vec(:,3),'r')
subplot(4,1,4);hold on; ylabel('Yaw change, deg')
plot(d(:,1), gt.rel.rpy(:,3)*180/pi)
plot(d(:,1), se.rel.rpy(:,3)*180/pi,'r')
subplot(4,1,1);title('Blue: Ground Truth; Red: State Estimate')

drift.trans_vec = se.rel.trans_vec - gt.rel.trans_vec;
drift.rpy = se.rel.rpy - gt.rel.rpy;
figure; 
subplot(4,1,1);hold on; ylabel('X drift, m')
plot(d(:,1), drift.trans_vec(:,1),'r')
subplot(4,1,2);hold on; ylabel('Y drift, m')
plot(d(:,1), drift.trans_vec(:,2),'r')
subplot(4,1,3);hold on; ylabel('Z drift, m')
plot(d(:,1), drift.trans_vec(:,3),'r')
subplot(4,1,4);hold on; ylabel('Yaw drift, deg')
plot(d(:,1), drift.rpy(:,3)*180/pi,'r')
subplot(4,1,1);title(['Drift over the previous ' num2str(dt) ' seconds'])

drift.xyz = sqrt(sum((drift.trans_vec(:,:)).^2,2))
drift.xy = sqrt(sum((drift.trans_vec(:,1:2)).^2,2))
distance_travelled =  gt.rel.dist_travelled;%sqrt(sum(gt.rel.trans_vec.^2,2))
ddt.xyz = 100*drift.xyz ./ distance_travelled;
ddt.xy = 100*drift.xy ./ distance_travelled;

ddt.xy(isnan(ddt.xy)) = 0;
ddt.xyz(isnan(ddt.xyz)) = 0;

figure; 
subplot(5,1,1);hold on; ylabel('XYZ drift, m')
plot(d(:,1), drift.xyz,'r')

subplot(5,1,2);hold on; ylabel('XY drift, m')
plot(d(:,1), drift.xy,'r')

subplot(5,1,3);hold on; ylabel('distance travelled, m')
plot(d(:,1), distance_travelled,'r')
plot(d(:,1), gt.rel.dist_travelled,'k')


subplot(5,1,4);hold on; ylabel('ddt XYZ, percent')
plot(d(:,1), ddt.xyz,'r')
ylim([ 0 15])
title(['median percent drift per distance travelled: ' num2str(median(ddt.xyz)) ] )

subplot(5,1,5);hold on; ylabel('ddt XY, percent')
plot(d(:,1), ddt.xy,'r')
ylim([ 0 15])
title(['median percent drift per distance travelled: ' num2str(median(ddt.xy)) ] )
xlabel('Seconds')
subplot(5,1,1);title(['Drift over the previous ' num2str(dt) ' seconds'])



function rel = transform_relative_list(gt,offset)
global bot_
% choose suitable offset increment
rel.trans_vec = repmat([0,0,0]   ,size(gt.trans_vec,1), 1);
rel.rot_quat =  repmat([1,0,0,0] ,size(gt.trans_vec,1), 1);
rel.rpy = repmat([0,0,0]   ,size(gt.trans_vec,1), 1);
for i=(offset+1):size(gt.trans_vec,1)
    
  pose_a.trans_vec =  gt.trans_vec(i-offset,:);
  pose_a.rot_quat =  gt.rot_quat(i-offset,:);
    
  pose_b.trans_vec =  gt.trans_vec(i,:);
  pose_b.rot_quat =  gt.rot_quat(i,:);
  
  this_rel = transform_relative(pose_a, pose_b);
  rel.trans_vec(i,:) = this_rel.trans_vec;
  rel.rot_quat(i,:) = this_rel.rot_quat;
  
  rpy = bot_.quat_to_roll_pitch_yaw(this_rel.rot_quat);
  rel.rpy(i,:) = rpy;
  
  rel.dist_travelled(i,:) = gt.cum_dist_travelled(i) - gt.cum_dist_travelled(i-offset);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% given an initial vicon position and an initial estimate position (taken
% synchronously), determine the current position in the vicon frame
%function worldvicon_to_est =transform_relative(init_vicon, init_est, current_est)
%global bot_
%temp = bot_.trans_invert(init_est) ;
% how much have we moved since we started:

%estbody_zerotime_to_estbody_current = bot_.trans_apply_trans( current_est,temp   );

% ... applied to the initial vicon
%worldvicon_to_est = bot_.trans_apply_trans(estbody_zerotime_to_estbody_current, init_vicon );


function pose_ab = transform_relative(pose_a, pose_b)
global bot_
%  #print "xxxxxxxxxxx"
%  #print pose_a.trans_vec
%  print pose_b.trans_vec
pose_ab = bot_.trans_apply_trans( pose_b, bot_.trans_invert(pose_a));
end