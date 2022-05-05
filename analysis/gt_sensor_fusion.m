%% IMU + GPS Sensor Fusion for ground truth
% uses an EKF from Matlab's sensor fusion toolbox
% to combine the measurements for the ground truth
% modeled from this example: 
% https://www.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html


%% input data
bag = rosbag('/home/curtis/datasets/nuance/morning_gps_imu_gt.bag');
imubSel = select(bag,'Topic','/imu/imu');
gpsbSel = select(bag,'Topic','/vehicle/gps/fix');

imumsgStructs = readMessages(imubSel,'DataFormat','struct');
gpsmsgStructs = readMessages(gpsbSel,'DataFormat','struct');

linAcc = cellfun(@(m) struct(m.LinearAcceleration),imumsgStructs);
angVel = cellfun(@(m) struct(m.AngularVelocity),imumsgStructs);
orient = cellfun(@(m) struct(m.Orientation),imumsgStructs);

linAccArray = [linAcc.X; linAcc.Y; linAcc.Z]';
angVelArray = [angVel.X; angVel.Y; angVel.Z]';
% quaternion put in matlab format, with the real part (W) at index 1
quatArray = [orient.W; orient.X; orient.Y; orient.Z]';
quatObject = quaternion(quatArray);

lat = cellfun(@(m) double(m.Latitude),gpsmsgStructs);
lon = cellfun(@(m) double(m.Longitude),gpsmsgStructs);
alt = cellfun(@(m) double(m.Altitude),gpsmsgStructs);

gpsLLA = [lat, lon, alt];
gpsUTM = getUTM(gpsLLA);
gpsVel = getVelFromGPS(gpsUTM, gpsLLA);

% get timeseries data
[tsImu timeStampIMU] = getTimeSeries(imumsgStructs);
[tsGps timeStampGPS] = getTimeSeries(gpsmsgStructs);


%% Fusion

% initialization
imuFs = 100;
gpsFs = 1; 

% Define where on the Earth this simulated scenario takes place using the
% latitude, longitude and altitude.
localOrigin = gpsLLA(1,:);


% Validate that the |gpsFs| divides |imuFs|. This allows the sensor sample 
% rates to be simulated using a nested for loop without complex sample rate
% matching.

imuSamplesPerGPS = (size(imumsgStructs)/size(gpsmsgStructs));
%assert(imuSamplesPerGPS == fix(imuSamplesPerGPS), ...
%    'GPS sampling rate must be an integer factor of IMU sampling rate.');

% set up initial state vector consisting of 16 states
initstate = zeros(16,1);
initstate(1:4) = compact(meanrot(quatObject(1:100))); 
% gyro bias
initstate(5:7) = [0.000638712916426128, 0.000756813427481811, 0.0006913442777512];
% set initial position to UTM position or local origin?
%initstate(8:10) = [gpsUTM(1,1), gpsUTM(1,2), gpsLLA(1,3)];
initstate(8:10) = [0.0, 0.0, gpsLLA(1,3)];

initstate(11:13) = [gpsVel(1,1), gpsVel(1,2), gpsVel(1,3)];
% acc bias
initstate(14:16) = [0.0125622794947136, 0.0116115161825542, 0.0170527491375191];


filt = insfilterNonholonomic('ReferenceFrame','ENU');
filt.IMUSampleRate = imuFs;
filt.ReferenceLocation = localOrigin;
filt.State = initstate;
%filt.StateCovariance = 1e-9*eye(16);
filt.StateCovariance = eye(16);


filt.ReferenceLocation = localOrigin;

% need to figure out calculations for these values
% noise values in (m/s^2)^2 and (rad/s)^2 
%filt.AccelerometerBiasNoise =  2e-4;
filt.AccelerometerNoise = 2; 
%filt.GyroscopeBiasNoise = 1e-16;
filt.GyroscopeNoise =  1e-5;

filt.AccelerometerBiasDecayFactor = 0;
filt.GyroscopeBiasDecayFactor = 0;


% fusion loop!
% position and velocity covariances
% post-process
Rpos = eye(3) .* 0.0000100;
Rvel = eye(3) .* 0.000100;

numIMUSamples = size(linAccArray,1);
estOrient = quaternion.ones(numIMUSamples,1);
estPos = zeros(numIMUSamples,3);
    
gpsIdx = 1;



for idx = 1:numIMUSamples
    predict(filt,linAccArray(idx,:),angVelArray(idx,:));       %Predict filter state
    
    if (mod(idx,imuSamplesPerGPS) < 1)                   %Correct filter state
        fusegps(filt,gpsLLA(gpsIdx,:),Rpos,gpsVel(gpsIdx,:),Rvel);
        gpsIdx = gpsIdx + 1;
    end
    
    [estPos(idx,:),estOrient(idx,:)] = pose(filt);        %Log estimated pose
end

%% Plots
close all;

firstEast = gpsUTM(1,1);
firstNorth = gpsUTM(1,2);

localEast = gpsUTM(:,1) - firstEast;
localNorth = gpsUTM(:,2) - firstNorth;

figure(1)
scatter(estPos(:,1),estPos(:,2))
hold on;
scatter(localEast,localNorth)
legend('EKF Estimated Pose','GPS UTM Position')
xlabel('Local Displacement - Easting (m)')
ylabel('Local Displacement - Northing (m)')
title('IMU+GPS Estimated Pose vs Local UTM Position')
axis('equal')
print -depsc gtAccuracy

figure(2)
ax = gca;
geoscatter(lat,lon,[],tsGps,'filled')
%geoscatter(lat(1:120),lon(1:120),[],gps_ts(1:120),'filled')

title('GPS Data and Route Overlay - 01FEB2020 Data Collect')
c = colorbar;
c.Label.String = 'Time Elapsed (s)';
hold on;
geoscatter(lat(1),lon(1),'magenta','p','filled','SizeData',300,'LineWidth',2)
hold on;
geoscatter(lat(end),lon(end),'red','x','SizeData',300,'LineWidth',2)
legend('GPS Data','Start Point','End Point', Location='northwest')
set(gca, 'ActivePositionProperty', 'position')
print -depsc route1


%% Export groundtruth to TUM .txt file
% NOTE: May have to convert to camera frame if offset in gt/pose
est_quat = compact(estOrient); % W, X, Y, Z
tum_mat = [timeStampIMU, estPos, est_quat(:,2), est_quat(:,3), est_quat(:,4), est_quat(:,1)];
writematrix(tum_mat,'gt_morning.txt','Delimiter','space');
%type 'gt_morning.txt';

function [ts, unix_ns] = getTimeSeries(msgStruct)
    % get timestamps from headers with nanosecond precision
    Sec = cellfun(@(m) uint64(m.Header.Stamp.Sec), msgStruct)*1e9;
    Nsec = cellfun(@(m) uint64(m.Header.Stamp.Nsec),msgStruct);
    unix_ns = Sec + Nsec;
    
    timeElapsed = double((unix_ns - min(unix_ns)))/1.0e9;
    unix_ns = double(unix_ns) / 1.0e9;
    ts = linspace(0,max(timeElapsed),length(timeElapsed));
end

function gpsUTM = getUTM(lla)
    % adapted from: 
    % https://www.mathworks.com/matlabcentral/answers/312772-latitude-and-longitude-data-to-universal-transverse-mercator-utm
    p1 = lla(1,1:2);
    z1 = utmzone(p1);
    [ellipsoid,estr] = utmgeoid(z1);
    utmstruct = defaultm('utm');
    utmstruct.zone = z1;
    utmstruct.geoid = ellipsoid;
    utmstruct = defaultm(utmstruct);
    [x,y] = projfwd(utmstruct,lla(:,1),lla(:,2));
    gpsUTM = [x,y];
end

function gpsVel = getVelFromGPS(gpsUTM, lla)
    gpsPos = [gpsUTM, lla(:,3)];
    vel = diff(gpsPos);
    % add a row to the first position so that array dimensions
    % remain the same
    gpsVel = [vel(1,:);vel];
end