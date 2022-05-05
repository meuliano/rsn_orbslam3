%% Odometry txt file conversion
% converts odometry data from ROS bag (recorded in txt file)
% to TUM format for Evo tool evaluation

odom_in = readmatrix("odom_files/odom_dual_loop_closure.txt");

% remove frame_id column (full of NaNs) and other not needed data
odom_in(:,2:4) = [];

% convert timestamp from ns to seconds
odom_in(:,1) = odom_in(:,1) / 1.0e9;

% output odometry to txt file
writematrix(odom_in,'odom_files/odom_dual_loop_closure_out.txt','Delimiter','space');