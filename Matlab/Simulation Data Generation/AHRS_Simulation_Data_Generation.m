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

%Transform field vectors
BodyGravity = zeros(L, 3);
BodyMagField = zeros(L, 3);
for i = 1 : L
   Rx = [1 0 0; 0 cos(testRoll(i)) -sin(testRoll(i)); 0 sin(testRoll(i)), cos(testRoll(i))];
   Ry = [cos(testPitch(i)) 0 sin(testPitch(i)); 0 1 0; -sin(testPitch(i)) 0 cos(testPitch(i))];
   Rz = [cos(testYaw(i)) -sin(testYaw(i)) 0; sin(testYaw(i)) cos(testYaw(i)) 0; 0 0 1];
   R = Rx * Ry * Rz;
   partCalcBodyGravity = R * RefGravitationalField;
   partCalcBodyMagField = R * RefMagneticField;
   BodyGravity(i, 1) = partCalcBodyGravity(1, 1); %[m/s2]
   BodyGravity(i, 2) = partCalcBodyGravity(2, 1); %[m/s2]
   BodyGravity(i, 3) = partCalcBodyGravity(3, 1); %[m/s2]
   BodyMagField(i, 1) = partCalcBodyMagField(1, 1); %[uT]
   BodyMagField(i, 2) = partCalcBodyMagField(2, 1); %[uT]
   BodyMagField(i, 3) = partCalcBodyMagField(3, 1); %[uT]
end




figure
hold on
title('Magnetic Field in Body Frame')
plot(time, BodyMagField(:, 1), '-', 'Color', 'blue')
plot(time, BodyMagField(:, 2), '--', 'Color', 'red')
plot(time, BodyMagField(:, 3), '-.', 'Color', 'black')
grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
figure
hold on
title('Gravitational Field in Body Frame')
plot(time, BodyGravity(:, 1), '-', 'Color', 'blue')
plot(time, BodyGravity(:, 2), '--', 'Color', 'red')
plot(time, BodyGravity(:, 3), '-.', 'Color', 'black')
grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
figure
hold on
plot(time, testYaw, 'Color', 'green')
plot(time, AngVel(:, 3), '-.', 'Color', 'magenta')
grid, legend('Yaw', 'dYaw')
figure
hold on
plot(time, testPitch, 'Color', 'red')
plot(time, AngVel(:, 2), '-.', 'Color', 'black')
grid, legend('Pitch', 'dPitch')
figure
hold on
plot(time, testRoll, 'Color', 'blue')
plot(time, AngVel(:, 1), '-.', 'Color', 'black')
grid, legend('Roll', 'dRoll')
figure
title('Simulated Accelerometer Data'), hold on
plot(time, Accel(:, 1), '-', 'Color', 'blue')
plot(time, Accel(:, 2), '--', 'Color', 'red')
plot(time, Accel(:, 3), '-.', 'Color', 'black')
grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
figure
title('Simulated Magnetometer Data'), hold on
plot(time, MagField(:, 1), '-', 'Color', 'blue')
plot(time, MagField(:, 2), '--', 'Color', 'red')
plot(time, MagField(:, 3), '-.', 'Color', 'black')
grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')
figure
title('Simulated Gyroscope Data'), hold on
plot(time, AngVel(:, 1), '-', 'Color', 'blue')
plot(time, AngVel(:, 2), '--', 'Color', 'red')
plot(time, AngVel(:, 3), '-.', 'Color', 'black')
grid, legend('X - body frame axis', 'Y - body frame axis', 'Z - body frame axis')