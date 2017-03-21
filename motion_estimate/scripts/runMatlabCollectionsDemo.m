% demonstration of Collections to LCM
global bot_
bot_ = bot;
rpg_add_lcm_path()

% 2. Plot a set of axes - and connect them:
pos= rand(4,3) 
pos(:,1) = pos(:,1) - 2
rpy= rand(size(pos))/5
plot_lcm_poses(pos, bot_.roll_pitch_yaw_to_quat(rpy) , 8, 'Triads', 5, 1, 1, 9)



% 2. Plot a set of points and lines
pos= rand(12,3)
cols=repmat( [1,0,0], size(pos)) % red

plot_lcm_points(pos, cols, 10, 'Points', 1, 1)
plot_lcm_points(pos, cols, 11, 'Line', 2, 1)
plot_lcm_points(pos, cols, 12, 'Triangles', 7, 1)


% 3. Plot an image:
%mit=imread('mit_logo.png');
%image(mit)
%x = repmat(1:32, 32,1)/10;
%y = repmat(1:32, 1,32)/10;
%z = rand(32,32)/10;
%pos = [x(:), y(:), z(:)];
%pos(:,1) = -3-pos(:,1);
%cols = reshape( double(mit)/255 ,32*32,3);
%plot_lcm_points(pos, cols, 13, 'MIT Logo', 1, 1)


% 4. Foot steps - dont connect them
pos = [0.5  , 0 ,0;
       0  , 1.05 ,0;
       0.6, 2.05 ,0;
       0.1, 2.9,0;
       0.66 3.8 , 0 ;
       0  , 4.7 ,0;
       0.7, 5.9 ,0;
       0.15, 6.3,0       ];
pos(:,1)= pos(:,1) + 2;
rpy= zeros(size(pos));
rpy(:,1) = ( 90-10 + 20*rand( size(pos,1) ,1) )*pi/180;
plot_lcm_poses(pos, bot_.roll_pitch_yaw_to_quat(rpy) , 14, 'Foot Steps', 5, 1, 0, -1)


% 5. A polygon:
pos = [0.5  , 0 ,0;
       0  , 1.05 ,0;
       0.6, 2.05 ,0;
       0.1, 2.9,0;
       0.66 3.8 , 0 ;
       0  , 4.7 ,0;
       0.7, 5.9 ,0;
       0.15, 6.3,0       ];
pos(:,1)= pos(:,1) + 3;
cols=repmat( [0,1,0], size(pos)); % red
plot_lcm_points(pos, cols, 15, 'Polygon', 10, 1)



