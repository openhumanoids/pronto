global bot_
bot_ = bot;
rpg_add_lcm_path()

%1 234 567 is t xyz rpy (in degrees)
filepath  = '/home/mfallon/2017-rss-simona/data/plots/imu-lo-vo-aicp-microstrain/log2_trot';
gt_filename = [ filepath '/viconpos.txt' ];
gt_data= load(gt_filename);
se_filename = [ filepath '/prontopos.txt' ];
se_data= load(se_filename);

figure; hold on
plot(gt_data(:,2),gt_data(:,3)); 
plot(se_data(:,2),se_data(:,3),'r'); 
axis equal

gt_input = convert_to_quat(gt_data);
se_input_full = convert_to_quat(se_data);

gt_idx = round(linspace(1, size(gt_input,1), 100));
se_idx = round(linspace(1, size(se_input_full,1), 100));
plot_lcm_poses(gt_input(gt_idx,2:4) , gt_input(gt_idx,5:8), 1, 'gt', 5, 1, 1, 2);
plot_lcm_poses(se_input_full(se_idx,2:4) , se_input_full(se_idx,5:8), 3, 'se', 5, 1, 1, 4);



% synchronse: drop se from 1kHz to 100Hz
disp('synchronise')
se_input = zeros(size(gt_input));
for i=1:size(gt_input,1)
    if mod(i,1000)==0
        i
        size(gt_input,1)
    end
  [a,b] = min(abs(se_input_full(:,1) - gt_input(i,1)));
  se_input(i,:) = se_input_full(b,:);
end

disp('save')
output = [gt_input,se_input];
output_filename = [ filepath '/vicon_se_sync.txt' ];
csvwrite(output_filename ,output);

function d_out = convert_to_quat(d1)
global bot_
d_out = zeros(size(d1,1) , 8);
d_out(:,1:4) = d1(:,1:4);
d_out(:,5:8) = bot_.roll_pitch_yaw_to_quat ( d1(:,5:7)*pi/180.0 );
end