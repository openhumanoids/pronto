function se_analysis_val_vicon()
% Steps to state estimation analysis
% 1. Run script to run sefusion lcmlogs or/and to convert output lcmlog to a matlab .mat
% se-batch-process-valkyrie.sh
%
% 2. Move the log into the results directory
% cd ~/Desktop/results ~/logs/valkyrie/lcm_logs/results/Valkyrie_100QuickSteps_NoArms-raw-2015-11-18-14-46-<short-summary>
%
% 3. Run matlab plotting to compare POSE_BDI/BODY and VICON
% se-analysis.m


close all
global bot_
bot_ = bot;

main_dir = [getenv('HOME')  '/Desktop/']
run_dir = 'results-pronto-vicon'
folder_path = [main_dir run_dir '/'];

logs = dir( [folder_path '*mat'])


settings.parse_async =0;
settings.plot_async = 0;
settings.parse_sync = 1;
settings.plot_sync = 1;
settings.save_raw_plots = 1;
settings.do_sync_comparison=1;
settings.vicon_median_filter =0;
settings.vicon_invalid_filter =0;

for i=1:size(logs,1)
  disp([ num2str(i) ': ' logs(i).name])
end

% all:
which_process= 1:size(logs,1)


%%%%%%%%%%%%%%%% DO WORK %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:size(which_process,2)
  disp(num2str(i))
  settings.folder_path  = folder_path
  settings.log_filename = logs( which_process(i) ).name
  summary(i) = file_analysis( settings  );
end


%save summary summary

if (settings.do_sync_comparison)
  h=figure('Position', [1, 1, 1700, 1200]);
  for i = 1:size(which_process,2)
    a = [summary(i).b.xyz_drift  summary(i).m.xyz_drift ];
    b = [summary(i).b.xy_drift  summary(i).m.xy_drift ];
    c = [summary(i).b.z_drift  summary(i).m.z_drift ];
    d = [summary(i).b.rpy_drift  summary(i).m.rpy_drift ];
    
    log_summary = [a;b;c;d];
    subplot(3,3,i); hold on; bar(log_summary,.75,'grouped')
    set(gca,'fontSize',7)

    ylabel(num2str(summary(i).b.t, '%2.0f sec'))
    fname = logs(which_process(i) ).name;
    %title( fname(1:31) )
    title( fname )
    set(gca,'XTick',[1,2,3,4]);set(gca,'XTickLabel',{'XYZ drift','XY drift','Z drift','Yaw drift'})
  end
  subplot(3,3,8)
  xlabel('Alt: Blue, Pronto: Magenta | Drift in dimensions')
end
png_fname = [folder_path 'summary.png'];
saveas( h, png_fname,'png');


function summary = file_analysis(settings)
%reads the data and does the parse-sync procedure
[a,s] = do_pre_process(settings)

%keyboard
%save run_summary a s settings

summary = do_plotting(a,s,settings)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [summary] = do_plotting(a,s,settings)
%%%% Plotting  %%%%%%%%%%%%%%%%%%
% s is the synced dataset

handles=[];
if (settings.plot_async==1)
  make_plots(a,settings.log_filename)
end
if (settings.plot_sync==1)
  %makes first figure of plots  
  handles_a=make_plots(s,settings.log_filename);
  handles = [handles;handles_a];
end

if (settings.do_sync_comparison)
  %only looks at yaw?  
  s.b.rpy_drift =  s.v.rot_rpy(:,3)  - s.b.rel_v.rot_rpy(:,3);
  s.m.rpy_drift =  s.v.rot_rpy(:,3)  - s.m.rel_v.rot_rpy(:,3);
  s.b.xyz_drift =  sqrt(sum((s.v.trans_vec - s.b.rel_v.trans_vec).^2,2));
  %column of euclidean differences for each point
  s.m.xyz_drift =  sqrt(sum((s.v.trans_vec - s.m.rel_v.trans_vec).^2,2));
  s.b.xy_drift =  sqrt(sum((s.v.trans_vec(:,1:2) - s.b.rel_v.trans_vec(:,1:2) ).^2,2));
  s.m.xy_drift =  sqrt(sum((s.v.trans_vec(:,1:2) - s.m.rel_v.trans_vec(:,1:2) ).^2,2));
  s.b.z_drift =  sqrt(sum((s.v.trans_vec(:,3) - s.b.rel_v.trans_vec(:,3) ).^2,2));
  s.m.z_drift =  sqrt(sum((s.v.trans_vec(:,3) - s.m.rel_v.trans_vec(:,3) ).^2,2));
  
  handles_b=make_plots_synced(s, settings.log_filename);
  handles = [handles;handles_b];
  
  
  summary.b.xy_drift  = s.b.xy_drift(end);
  summary.b.xyz_drift = s.b.xyz_drift(end);
  summary.b.z_drift   = s.b.z_drift(end);
  summary.b.rpy_drift = s.b.rpy_drift(end);
  summary.b.t         = s.b.t(end);
  summary.m.xy_drift  = s.m.xy_drift(end);
  summary.m.xyz_drift = s.m.xyz_drift(end);
  summary.m.z_drift   = s.m.z_drift(end);
  summary.m.rpy_drift = s.m.rpy_drift(end);
  summary.m.t         = s.m.t(end);
  
end



if (settings.save_raw_plots)
  % save plots to file:
  for j=1:size(handles,1)
    png_fname = [settings.folder_path settings.log_filename(1:end-4) '-' num2str(j) '.png'];
    saveas( handles(j), png_fname,'png');
  end
  %close all
  
end



function [a,s] = do_pre_process(settings)
a=[]; b=[];
load([settings.folder_path settings.log_filename]);
raw = [ 0*ones(size(POSE_VICON,1),1) , POSE_VICON ];
raw = [raw; 1*ones(size(POSE_BODY_ALT,1),1) , POSE_BODY_ALT];
raw = [raw; 2*ones(size(POSE_BODY,1),1) , POSE_BODY];
res = sortrows(raw, 2);
% convert to mins from zero
res(:,2) = (res(:,2) - res(1,2))*1E-6;

clip_start =0;
if (clip_start)
  disp('Clipping first 15 seconds')
  idx_keep = res(:,2) > 15;
  res = res(idx_keep,:);
  res(:,2) = res(:,2) - res(1,2); % rezero
end


%%%%% Parseing %%%%%%%%%%%%%%%%%%
if (settings.parse_async)
  [a] = parse_async(res);
end
if (settings.parse_sync)
  [s] = parse_sync(res);
end

% filter out invalid points.
% 'filtering' is just setting them to the most recent values
% but removing the samples entirely would be a better approach ;) of course

%THIS NEVER HAPPENS WITH OUR LOGS ANYWAY
if (settings.vicon_invalid_filter)
  %z smaller than 0.2...? if this is the case, replace the point with most recent value  
  s.v.invalid = (s.v.trans_vec(:,3)  < 0.2);
  
  good_idx = 1;
  for i=1:size(s.v.invalid,1)
    if (s.v.invalid(i) ~= 1)
      good_idx = i;
    end  
    use_idx(i) = good_idx;
  end
  
  temp = s.v.trans_vec(use_idx,:);
  s.v.trans_vec = temp;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function a=parse_async(res)
% extract the vicon, alt or pronto estimates:
i_vicon = res(:,1) ==0;
i_alt =  res(:,1) ==1;
i_pronto =  res(:,1) ==2;
a.v = split_data(res,i_vicon);
a.b = split_data(res,i_alt);
a.m = split_data(res,i_pronto);

% Transform the Asynchronous Log into the vicon frame
a.v.init.trans_vec = a.v.trans_vec(1,:);
a.v.init.rot_quat = a.v.rot_quat(1,:);
disp('b -> v [async]')
a.b=transform_est_to_vicon(a.v.init, a.b,1);
disp('m -> v [async]')
a.m=transform_est_to_vicon(a.v.init, a.m,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s=parse_sync(res)
%parse_sync works by only selecting pose_body points at time steps closest
%and before pose_vicon time steps. it then retains only timestamps,
%position, quaternion and rpy equivalent. also, it represents all data
%relative to the initial vicon estimate, so that all data starts from the
%same location and is relative to the same initial starting point and is
%thus comparable

% Synchronize the log:

%get first vicon
last_b = res(  find(res(:,1) == 1 ,1) , :);

%get first pronto
last_m = res(  find(res(:,1) == 2 ,1) , :);

%get first initial input - also vicon in this case
last_v = res(  find(res(:,1) == 0 ,1) , :);

n_cols = size(res(1,:),2);
%number of data points in the initial input - vicon in this case
n_rows = sum(res(:,1) == 0);

res_sync.v = zeros(n_rows,n_cols);
res_sync.b = zeros(n_rows,n_cols);
res_sync.m = zeros(n_rows,n_cols);

res_sync.b(1,:) = last_b;
res_sync.v(1,:) = last_v;
res_sync.m(1,:) = last_m;

%for each time stamp in the initial input (vicon)
%search within each of the data sets and collect the data point closest to
%and before the time stamp in the initial input
counter=1;
for i=1:size(res,1)
  if ( res(i,1)  ==0 )
    counter=counter+1;
    res_sync.v(counter,:) = res(i,:);
    res_sync.m(counter,:) = last_m;
    res_sync.b(counter,:) = last_b;
  elseif ( res(i,1)  ==1 )
    last_b = res(i,:);
  elseif ( res(i,1)  ==2 )
    last_m = res(i,:);
  end
end

%res_sync has 3 arrays of the same size with time points as close to each
%other as possible

%get only the timestamps, positions and rpy orientations
s.b = split_data(res_sync.b , 1:size(res_sync.b,1));
s.m = split_data(res_sync.m , 1:size(res_sync.m,1));
s.v = split_data(res_sync.v , 1:size(res_sync.v,1));



% Transform the Synchronous Log into the vicon frame
s.v.init.trans_vec =s.v.trans_vec(1,:);
s.v.init.rot_quat = s.v.rot_quat(1,:);

%also add representations of the data relative to the initial position and
%orientation of the input (vicon)
disp('b -> v [sync]')
s.b=transform_est_to_vicon(s.v.init, s.b,0);
disp('m -> v [sync]')
s.m=transform_est_to_vicon(s.v.init, s.m, 0);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handles= make_plots_synced(s,log_filename)
handles=figure('Position', [1, 1, 1700, 900]);

subplot(2,3,1); hold on
plot(s.b.t,s.b.rpy_drift*180/pi,'b','MarkerSize',2);
p1 = plot(s.m.t,s.m.rpy_drift*180/pi,'m');
title('Yaw Drift [deg]')


subplot(2,3,2); hold on
plot(s.b.t,s.b.xyz_drift,'b');
p2 = plot(s.m.t,s.m.xyz_drift,'m');
title('XYZ Drift')


subplot(2,3,3); hold on
plot(s.b.t,s.b.xy_drift,'b');
p3 = plot(s.m.t,s.m.xy_drift,'m');
title('XY Drift')


subplot(2,3,4); hold on
plot(s.b.t,s.b.z_drift,'b')
plot(s.m.t,s.m.z_drift,'m')
title('Z Drift')

diff_val = 10; % time between samples of distance travelled from vicon
% 100 is 1Hz | 10 is 10Hz
cum_dist = cumsum(sqrt(sum((diff ( s.v.trans_vec(1:10:end,:)  )).^2,2)));
time_temp = s.v.t(1:10:end);
t_cum_dist = time_temp(2:end);

subplot(2,3,5); hold on
plot(t_cum_dist, cum_dist)
cum_dist(end)
t_cum_dist(end)

title('Vicon Distance Travelled [samples at 10Hz]')
xlabel('Time [seconds]')
ylabel('Distance [meters]')

drawnow
pause(0.1)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot generic details - either sync or asyc
function handles=make_plots(d,log_filename)
handles=figure('Position', [1, 1, 1700, 900]);

subplot(2,3,1)
hold on
plot(d.v.trans_vec(:,1), d.v.trans_vec(:,2),'g')
plot(d.b.trans_vec(:,1), d.b.trans_vec(:,2),'b')
plot(d.m.trans_vec(:,1), d.m.trans_vec(:,2),'m')
axis equal
title('unaligned x and y')

%plotting relative to initial vicon position
subplot(2,3,2)
hold on
plot(d.v.trans_vec(:,1), d.v.trans_vec(:,2),'g')
plot(d.b.rel_v.trans_vec(:,1), d.b.rel_v.trans_vec(:,2),'b')
plot(d.m.rel_v.trans_vec(:,1), d.m.rel_v.trans_vec(:,2),'m')
axis equal
title('aligned x and y')
xlabel(log_filename)

subplot(2,3,3)
hold on
plot(d.v.t(:), d.v.trans_vec(:,3),'g')
plot(d.b.rel_v.t(:), d.b.rel_v.trans_vec(:,3),'b')
plot(d.m.rel_v.t(:), d.m.rel_v.trans_vec(:,3),'m')
title('aligned z and time')

subplot(2,3,4)
hold on
plot(d.v.t(:), d.v.rot_rpy(:,1)*180/pi,'g')
plot(d.b.rel_v.t(:), d.b.rel_v.rot_rpy(:,1)*180/pi,'b')
plot(d.m.rel_v.t(:), d.m.rel_v.rot_rpy(:,1)*180/pi,'m')
title('aligned roll (deg) and time')

subplot(2,3,5)
hold on
plot(d.v.t(:), d.v.rot_rpy(:,2)*180/pi,'g')
plot(d.b.rel_v.t(:), d.b.rel_v.rot_rpy(:,2)*180/pi,'b')
plot(d.m.rel_v.t(:), d.m.rel_v.rot_rpy(:,2)*180/pi,'m')
title('aligned pitch (deg) and time')

subplot(2,3,6)
hold on
plot(d.v.t(:), d.v.rot_rpy(:,3)*180/pi,'g')
plot(d.b.rel_v.t(:), d.b.rel_v.rot_rpy(:,3)*180/pi,'b')
plot(d.m.rel_v.t(:), d.m.rel_v.rot_rpy(:,3)*180/pi,'m')
legend('g VICON','b Alt','m Pronto')
xlabel('Time [seconds]')
ylabel('Yaw [degrees]')
title('Aligned Yaw and Time')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [mode] = split_data(res, i)
global bot_;
%this function just returns the timestamps, positions and rpy orientation
%for all data points

%get all times
mode.t    = res(i,2);
%get all positions
mode.trans_vec  = res(i,3:5);
%get all rotations
mode.rot_quat = res(i,9:12);
% TODO:VECTORIZED THIS
for i=1:size(mode.rot_quat,1)
  mode.rot_rpy(i,:) = bot_.quat_to_roll_pitch_yaw( mode.rot_quat(i,:) );
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function e=transform_est_to_vicon(v_init, e, use_subset)
%I think the purpose of this is to make everything relative to the initial
%position and orientation of the input (vicon) so that numbers are
%comparable

global bot_
init.trans_vec = e.trans_vec(1,:);
init.rot_quat = e.rot_quat(1,:);

if (use_subset)
  indices = round(linspace(1, size(e.trans_vec,1),1000));
else
  indices = 1:size(e.trans_vec,1); % use all data
end

for i=1:size(indices,2)
  e_current.trans_vec =  e.trans_vec( indices(i),:);
  e_current.rot_quat =  e.rot_quat( indices(i),:);
  
  e_current_rel_v = transform_relative(v_init, init, e_current);
  
  %store an estimate of the motion travelled relative to the initial
  %position and orientation of the input (vicon)
  e.rel_v.trans_vec(i,:) = e_current_rel_v.trans_vec;
  e.rel_v.rot_quat(i,:) = e_current_rel_v.rot_quat;
  
  e.rel_v.rot_rpy(i,:) = bot_.quat_to_roll_pitch_yaw( e_current_rel_v.rot_quat );
  e.rel_v.t(i) = e.t(indices(i));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% given an initial vicon position and an initial estimate position (taken
% synchronously), determine the current position in the vicon frame
function worldvicon_to_est =transform_relative(init_vicon, init_est, current_est)
global bot_
temp = bot_.trans_invert(init_est) ;
% how much have we moved since we started:
estbody_zerotime_to_estbody_current = bot_.trans_apply_trans( current_est,temp   );

% ... applied to the initial vicon
worldvicon_to_est = bot_.trans_apply_trans(estbody_zerotime_to_estbody_current, init_vicon );
