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
testYaw(3002 : L) = 10 * pi/3 * sin(2*pi*0.2381*time(3002 : L)); %[rad]

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
                                    
%%%%%%%%%%%%%%% Determine Orientation in Rotation Matrices & Quaternions %%%%%%%%%%%%%%%
eulerAngles = zeros(L, 3);
eulerAngles(:, 1) = testRoll; %[rad]
eulerAngles(:, 2) = testPitch; %[rad]
eulerAngles(:, 3) = testYaw; %[rad]
rotationMatrices_XYZ = eul2rotm(eulerAngles, 'XYZ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Transform field vectors %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BodyGravity = zeros(3, L);
BodyMagField = zeros(3, L);
for i = 1 : L
    BodyGravity(:, i) = rotationMatrices_XYZ(:, :, i) * RefGravitationalField; %[m/s2]
    BodyMagField(:, i) = rotationMatrices_XYZ(:, :, i) * RefMagneticField; %[uT]
end