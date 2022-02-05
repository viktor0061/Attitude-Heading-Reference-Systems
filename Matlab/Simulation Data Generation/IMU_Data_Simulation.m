close all
delta_T = 7e-3;
L = 4200;
time = 0 : delta_T : (L - 1) * delta_T;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Generate Ground Truth Data %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Generate Euler angles
testRoll = zeros(1,L);
testPitch = zeros(1,L);
testYaw = zeros(1,L); 
testRoll(602 : 1800) = pi/3 * sin(2*pi*0.2381*time(602 : 1800)); %[rad]
testPitch(1802 : 3000) = pi/3 * sin(2*pi*0.2381*time(1802 : 3000)); %[rad]
testYaw(3002 : L) = pi/3 * sin(2*pi*0.2381*time(3002 : L)); %[rad]

%Differentiate angles
AngVel = zeros(3, L); 
for i = 2:L
    AngVel(1, i) = (testRoll(i) - testRoll(i - 1))/delta_T; %[rad/s]
    AngVel(2, i) = (testPitch(i) - testPitch(i - 1))/delta_T; %[rad/s]
    AngVel(3, i) = (testYaw(i) - testYaw(i - 1))/delta_T; %[rad/s]
end
AngVel = transpose(AngVel);

%Generate magnetic and gravity vector
RefGravitationalField = [0; 0; 9.81]; %[m/s2]
RefMagneticField = [22.76; 0; 40.72]; % [uT] / Data based on the magnetic field calculator of the
                                    %National Centers for Envireomental Information
                                    %National Oceanic and Atmospheric Administration
                                    %Lat :45.614922 Long: 20.035061 [deg] SeaLvL : 77[m]
                                    
%%%%%%%%%%%%%%%%%%%% Transform field vectors %%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% Determine Orientation in Rotation Matrices & Quaternions %%%%%%%%%%%%%%%
eulerAngles = zeros(L, 3);
eulerAngles(:, 1) = testRoll;
eulerAngles(:, 2) = testPitch;
eulerAngles(:, 3) = testYaw;
rotationMatrices_XYZ = eul2rotm(eulerAngles, 'XYZ');
% rotationMatrices_ZYX = eul2rotm(eulerAngles, 'ZYX');
%quaternions = eul2quat(eulerAngles, 'XYZ');

BodyGravity = zeros(3, L);
BodyMagField = zeros(3, L);
for i = 1 : L
    BodyGravity(:, i) = rotationMatrices_XYZ(:, :, i) * RefGravitationalField; %[m/s2]
    BodyMagField(:, i) = rotationMatrices_XYZ(:, :, i) * RefMagneticField; %[uT]
%    Rx = [1 0 0; 0 cos(testRoll(i)) -sin(testRoll(i)); 0 sin(testRoll(i)), cos(testRoll(i))];
%    Ry = [cos(testPitch(i)) 0 sin(testPitch(i)); 0 1 0; -sin(testPitch(i)) 0 cos(testPitch(i))];
%    Rz = [cos(testYaw(i)) -sin(testYaw(i)) 0; sin(testYaw(i)) cos(testYaw(i)) 0; 0 0 1];
%    R = Rx * Ry * Rz;
%    partCalcBodyGravity = R * RefGravitationalField;
%    partCalcBodyMagField = R * RefMagneticField;
%    BodyGravity(1, i) = partCalcBodyGravity(1, 1); %[m/s2]
%    BodyGravity(2, i) = partCalcBodyGravity(2, 1); %[m/s2]
%    BodyGravity(3, 1) = partCalcBodyGravity(3, 1); %[m/s2]
%    BodyMagField(1, i) = partCalcBodyMagField(1, 1); %[uT]
%    BodyMagField(2, i) = partCalcBodyMagField(2, 1); %[uT]
%    BodyMagField(3, 1) = partCalcBodyMagField(3, 1); %[uT]
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Generate IMU Object %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IMU = imuSensor('accel-gyro-mag');
IMU.SampleRate = 1/delta_T;
IMU.MagneticField = transpose(RefMagneticField);

%Set Gyroscope parameters
IMU.Gyroscope.AccelerationBias = [0.0025 0.00235 0.0027]; IMU.Gyroscope.ConstantBias = [0.1 0.11 0.05];
IMU.Gyroscope.BiasInstability = [0.01 0.01 0.01]; IMU.Gyroscope.AxesMisalignment = [2, 4, 2.5]; IMU.Gyroscope.NoiseDensity = [0.00061, 0.00062, 0.00095];

%Set Accelerometer parameters
IMU.Accelerometer.ConstantBias = [0.05 0.06 0.055];IMU.Accelerometer.BiasInstability = [0.01 0.01 0.01]; 
IMU.Accelerometer.AxesMisalignment = [4, 3, 5]; IMU.Accelerometer.NoiseDensity = [0.05, 0.06, 0.09];

%Set Magnetometer parameters
IMU.Magnetometer.ConstantBias = [0.1 0.16 0.09];IMU.Magnetometer.BiasInstability = [0.001 0.001 0.001]; 
IMU.Magnetometer.AxesMisalignment = [3, 5, 4]; IMU.Magnetometer.NoiseDensity = [0.0096, 0.014, 0.01];

%Generate measurements with IMU model and Ground Truth data
MagField = zeros(L, 3);
Accel = zeros(L, 3);
for i = L
   Accel(i, :) = transpose(RefGravitationalField);
end
[Accel, AngVel, MagField] = IMU(Accel, AngVel, rotationMatrices_XYZ);




% figure
% hold on
% title('Magnetic Field in Body Frame')
% plot(time, BodyMagField(1, :), '-', 'Color', 'blue')
% plot(time, BodyMagField(2, :), '--', 'Color', 'red')
% plot(time, BodyMagField(3, :), '-.', 'Color', 'black')
% grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
% figure
% hold on
% title('Gravitational Field in Body Frame')
% plot(time, BodyGravity(1, :), '-', 'Color', 'blue')
% plot(time, BodyGravity(2, :), '--', 'Color', 'red')
% plot(time, BodyGravity(3, :), '-.', 'Color', 'black')
% grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
% figure
% hold on
% plot(time, testYaw, 'Color', 'green')
% plot(time, AngVel(:, 3), '-.', 'Color', 'magenta')
% grid, legend('Yaw', 'dYaw')
% figure
% hold on
% plot(time, testPitch, 'Color', 'red')
% plot(time, AngVel(:, 2), '-.', 'Color', 'black')
% grid, legend('Pitch', 'dPitch')
% figure
% hold on
% plot(time, testRoll, 'Color', 'blue')
% plot(time, AngVel(:, 1), '-.', 'Color', 'black')
% grid, legend('Roll', 'dRoll')
% figure
% title('Simulated Accelerometer Data'), hold on
% plot(time, Accel(:, 1), '-', 'Color', 'blue')
% plot(time, Accel(:, 2), '--', 'Color', 'red')
% plot(time, Accel(:, 3), '-.', 'Color', 'black')
% grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
% figure
% title('Simulated Magnetometer Data'), hold on
% plot(time, MagField(:, 1), '-', 'Color', 'blue')
% plot(time, MagField(:, 2), '--', 'Color', 'red')
% plot(time, MagField(:, 3), '-.', 'Color', 'black')
% grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
% figure
% title('Simulated Gyroscope Data'), hold on
% plot(time, AngVel(:, 1), '-', 'Color', 'blue')
% plot(time, AngVel(:, 2), '--', 'Color', 'red')
% plot(time, AngVel(:, 3), '-.', 'Color', 'black')
% grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')