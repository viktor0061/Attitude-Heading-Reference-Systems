%%%%%% Call Ground Truth Data Generator %%%%%%%

AHRS_Ground_Truth_Data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Generate IMU Object %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IMU = imuSensor('accel-gyro-mag');
IMU.SampleRate = 1/delta_T;
IMU.MagneticField = transpose(RefMagneticField);

%%%%%%%%%% Set Gyroscope parameters %%%%%%%%%%
IMU.Gyroscope.AccelerationBias = [0.0025 0.00235 0.0027]; IMU.Gyroscope.ConstantBias = [0.1 0.11 0.05];
IMU.Gyroscope.BiasInstability = [0.01 0.01 0.01]; IMU.Gyroscope.AxesMisalignment = [2, 4, 2.5]; IMU.Gyroscope.NoiseDensity = [0.00061, 0.00062, 0.00095];

%%%%%%%%%% Set Accelerometer parameters %%%%%%%%%%
IMU.Accelerometer.ConstantBias = [0.05 0.06 0.055];IMU.Accelerometer.BiasInstability = [0.01 0.01 0.01]; 
IMU.Accelerometer.AxesMisalignment = [4, 3, 5]; IMU.Accelerometer.NoiseDensity = [0.05, 0.06, 0.09];

%%%%%%%%%% Set Magnetometer parameters %%%%%%%%%%
IMU.Magnetometer.ConstantBias = [0.1 0.16 0.09];IMU.Magnetometer.BiasInstability = [0.001 0.001 0.001]; 
IMU.Magnetometer.AxesMisalignment = [3, 5, 4]; IMU.Magnetometer.NoiseDensity = [0.0096, 0.014, 0.01];

%%%%%%%%%% Generate measurements with IMU model and Ground Truth data %%%%%%%%%%
MagField = zeros(L, 3);
Accel = zeros(L, 3);
for i = L
   Accel(i, :) = transpose(RefGravitationalField);
end
[Accel, AngVel, MagField] = IMU(Accel, AngVel, rotationMatrices_XYZ); %[m/s2]; [rad/s]; [uT]




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