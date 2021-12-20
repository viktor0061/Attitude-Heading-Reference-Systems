close all
format long
%Flags
TEST = 1;
POS_OBSERVER = 0;
DO_CALIBRATION = 1;
%transform data so we can process it
if TEST == 0
    Accel = table2array(Acceleration);
    AngVel = table2array(AngularVelocity);
    MagField = table2array(MagneticField);
end
if TEST
    AHRS_Simulation_Data_Generation
end

accelX = Accel(:, 1);
accelY = Accel(:, 2);
accelZ = Accel(:, 3);

angularX = AngVel(:, 1);
angularY = AngVel(:, 2);
angularZ = AngVel(:, 3);

magX = MagField(:, 1);
magY = MagField(:, 2);
magZ = MagField(:, 3);
%Calibration
if DO_CALIBRATION
    CalSamples = 200;
    CalibrationAx = sum(accelX(1:CalSamples)) / CalSamples;
    CalibrationAy = sum(accelY(1:CalSamples)) / CalSamples;
    CalibrationAz = sum(accelZ(1:CalSamples) - 9.81) / CalSamples;
%     CalibrationAz = 0;        
    CalibrationGx = sum(angularX(1:CalSamples)) / CalSamples;
    CalibrationGy = sum(angularY(1:CalSamples)) / CalSamples;
    CalibrationGz = sum(angularZ(1:CalSamples)) / CalSamples;
else
    CalibrationAx = 0;
    CalibrationAy = 0;
    CalibrationAz = 0;
    CalibrationGx = 0;
    CalibrationGy = 0;
    CalibrationGz = 0;
end
calibratedAccelX = accelX - CalibrationAx;
calibratedAccelY = accelY - CalibrationAy;
calibratedAccelZ = accelZ - CalibrationAz;
calibratedAngularX = angularX - CalibrationGx;
calibratedAngularY = angularY - CalibrationGy;
calibratedAngularZ = angularZ - CalibrationGz;

%Define constants and time vector
delta_T = 7e-3;
L = length(angularX) - 1;
if TEST
   L = length(angularX);
end
time = 0 : delta_T : (L - 1) * delta_T;
rad2deg = 57.29578;
deg2rad = 0.0174532925;

%Calculate angles from accelero- and magnetometer
accelRollInRads = atan2(calibratedAccelY, sqrt((calibratedAccelX .* calibratedAccelX) + (calibratedAccelZ .* calibratedAccelZ)));
%the atan2 functions get a minus sign
%so CCW rotation will correspond to negative change in the angles
accelPitchInRads = -atan2(calibratedAccelX, sqrt((calibratedAccelY .* calibratedAccelY) + (calibratedAccelZ .* calibratedAccelZ)));
magYawInRads = atan2(magY, magX);
accelRoll = -accelRollInRads * rad2deg;
accelPitch = -accelPitchInRads * rad2deg;
magYaw = magYawInRads * rad2deg;
% magYawCal = sum(magYaw(1:300)) / 300;
% magYaw = magYaw - magYawCal;

%Transform magnetic readings from inertial system to frame of reference
rotatedMagX = zeros(1, L);
rotatedMagY = zeros(1, L);
rotatedMagZ = zeros(1, L);
partialRotatedMag = zeros(3, 1);
for i = 1 : L
    partialRotatedMag = [magX(i); magY(i); magZ(i)];
    Rx = [1, 0, 0; 
        0, cos(accelRollInRads(i)), -sin(accelRollInRads(i));
        0, sin(accelRollInRads(i)), cos(accelRollInRads(i))];
    Ry = [cos(accelPitchInRads(i)), 0, sin(accelPitchInRads(i));
        0, 1, 0;
        -sin(accelPitchInRads(i)), 0, cos(accelPitchInRads(i))];
    partialRotatedMag = Ry * Rx * partialRotatedMag;
    rotatedMagX(1, i) = partialRotatedMag(1, 1);
    rotatedMagY(1, i) = partialRotatedMag(2, 1);
    rotatedMagZ(1, i) = partialRotatedMag(3, 1);
    
%     rotatedMagX(i) = magX(i) * cos(accelPitchInRads(i)) + magZ(i) * sin(accelPitchInRads(i));
%     rotatedMagY(i) = magY(i) * cos(accelRollInRads(i)) + magX(i) * sin(accelRollInRads(i)) * sin(accelPitchInRads(i)) - magZ(i) * sin(accelRollInRads(i)) * cos(accelPitchInRads(i));
%     rotatedMagZ(i) = magY(i) * sin(accelRollInRads(i)) - magX(i) * sin(accelPitchInRads(i)) * cos(accelRollInRads(i)) + magZ(i) * cos(accelPitchInRads(i)) * cos(accelRollInRads(i));
end
transformedMagYawInRads = atan2(rotatedMagY, rotatedMagX);
transformedMagYaw = transformedMagYawInRads * rad2deg;

%Make the output of atan2 funcs continuous
last = zeros(1, 3);
for i = 2 : L
   while transformedMagYaw(i) < last(1, 3) - 180 
      transformedMagYaw(i) = transformedMagYaw(i) + 360;
   end
   while transformedMagYaw(i) > last(1, 3) + 180 
      transformedMagYaw(i) = transformedMagYaw(i) - 360;
   end
   last(1, 3) = transformedMagYaw(i);
end

%Calculate angles from gyro w/ dead reackoning and convert from rad to deg
gyroRoll = zeros(1, L);
gyroPitch = zeros(1, L);
gyroYaw = zeros(1, L);
gyroRollInitial = sum(accelRoll(1:300)) / 300;
gyroPitchInitial = sum(accelPitch(1:300)) / 300;
gyroYawInitial = sum(transformedMagYaw(1:300)) / 300;
for i = 1:L
    if i == 1
        gyroRoll(i) = +calibratedAngularX(i) * delta_T;
        gyroPitch(i) = + calibratedAngularY(i) * delta_T;
        gyroYaw(i) = +calibratedAngularZ(i) * delta_T;
    else
        gyroRoll(i) = gyroRoll(i - 1) + calibratedAngularX(i) * delta_T;
        gyroPitch(i) = gyroPitch(i - 1) + calibratedAngularY(i) * delta_T;
        gyroYaw(i) = gyroYaw(i - 1) + (calibratedAngularZ(i) * delta_T);
    end
end
gyroRoll = gyroRoll * rad2deg;
gyroPitch = gyroPitch * rad2deg;
gyroYaw = gyroYaw * rad2deg;

%Complementary Filter(sensor fusion)
%Calculates Euler angles
Alpha = 0.98;
Beta = 1 - Alpha;
Roll = zeros(1,L);
Pitch = zeros(1,L);
Yaw = zeros(1,L);
for i = 1:L
   if i == 1
       Roll(i) = Alpha * (gyroRollInitial + calibratedAngularX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
       Pitch(i) = Alpha * (gyroPitchInitial + calibratedAngularY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
       Yaw(i) = Alpha * (gyroYawInitial + calibratedAngularZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
   else    
       Roll(i) =  Alpha * (Roll(i - 1) + calibratedAngularX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
       Pitch(i) = Alpha * (Pitch(i - 1) + calibratedAngularY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
       Yaw(i) = Alpha * (Yaw(i - 1) + calibratedAngularZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
   end
end
rotatedGyroRoll = zeros(1, L);
rotatedGyroPitch = zeros(1, L);
rotatedGyroYaw = zeros(1, L);

%Filter acceleration readings and transform to reference frame
bodyAccel = zeros(3, 1);
partialRefAccel = zeros(3, 1);
partialRefVelocity= zeros(3, 1);
partialRefPosition= zeros(3, 1);
if POS_OBSERVER
    filteredAccelX = conv(Num3, calibratedAccelX);
    filteredAccelY = conv(Num3, calibratedAccelY);
    filteredAccelZ = conv(Num3, calibratedAccelZ);
end
if POS_OBSERVER == 0
    filteredAccelX = zeros(1, L);
    filteredAccelY = zeros(1, L);
    filteredAccelZ = zeros(1, L);
end
L2 = length(filteredAccelX);
time2 = 0 :delta_T: (L2 - 1) * delta_T;
refAccel = zeros(3, L2);
refVelocity= zeros(3, L2);
refPosition= zeros(3, L2);
for i = 1 : L
%The signs of the sinus functions in Ry and Rx was inverted -compared to the basic,
%right handed rotation matrices- because of the not usual
%coordinate systems and signs of the phone IMU and the code
Rx = [1, 0, 0; 
    0, cos(Roll(i) * deg2rad), sin(Roll(i) * deg2rad);
    0, -sin(Roll(i) * deg2rad), cos(Roll(i) * deg2rad)];
Ry = [cos(Pitch(i) * deg2rad), 0, -sin(Pitch(i) * deg2rad);
    0, 1, 0;
    sin(Pitch(i) * deg2rad), 0, cos(Pitch(i) * deg2rad)];
Rz = [cos(Yaw(i) * deg2rad), -sin(Yaw(i) * deg2rad), 0;
    sin(Yaw(i) * deg2rad), cos(Yaw(i) * deg2rad), 0;
    0, 0, 1];
R = Rz * Ry * Rx;
%    Rx = [1, 0, 0; 
%         0, cos(Roll(i) * deg2rad), -sin(Roll(i) * deg2rad);
%         0, sin(Roll(i) * deg2rad), cos(Roll(i) * deg2rad)];
%    Ry = [cos(Pitch(i) * deg2rad), 0, sin(Pitch(i) * deg2rad);
%         0, 1, 0;
%         -sin(Pitch(i) * deg2rad), 0, cos(Pitch(i) * deg2rad)];
%    bodyGravity = Rx * Ry * Rz * [0; 0; 9.81];
bodyAccel = [filteredAccelX(i); filteredAccelY(i); filteredAccelZ(i)];% - bodyGravity;
partialRefAccel = R * bodyAccel;
%Calculate displacement w/ deadreackoning
partialRefVelocity = partialRefVelocity + partialRefAccel * delta_T;
partialRefPosistion = partialRefPosition + partialRefVelocity * delta_T;

refAccel(1, i) = partialRefAccel(1, 1);
refAccel(2, i) = partialRefAccel(2, 1);
refAccel(3, i) = partialRefAccel(3, 1);

refVelocity(1, i) = partialRefVelocity(1, 1);
refVelocity(2, i) = partialRefVelocity(2, 1);
refVelocity(3, i) = partialRefVelocity(3, 1);

refPosition(1, i) = partialRefPosition(1, 1);
refPosition(2, i) = partialRefPosition(2, 1);
refPosition(3, i) = partialRefPosition(3, 1);
end
if TEST
   testRoll = testRoll * rad2deg;
   testPitch = testPitch * rad2deg;
   testYaw = testYaw * rad2deg;
   errorRateRoll = (-testRoll + Roll)./testRoll;
   errorRatePitch= (-testPitch + Pitch)./testPitch;
   errorRateYaw= (-testYaw + Yaw)./testYaw;
end

%Plot the results
figure
subplot(3, 1, 1), plot(time, Roll(1:L), "red", time, accelRoll(1:L), "green", time, gyroRoll(1:L), "blue"), legend("Roll", "Accel Roll", "Gyro Roll"), title("Roll"), grid
subplot(3, 1, 2), plot(time, Pitch(1:L), "red", time, accelPitch(1:L), "green", time, gyroPitch(1:L), "blue"), legend("Pitch", "Accel Pitch", "Gyro Pitch"), title("Pitch"), grid
subplot(3, 1, 3), plot(time, Yaw(1:L), "red", time, transformedMagYaw(1:L), "green", time, gyroYaw(1:L), "blue"), legend("Yaw", "Mag Yaw", "Gyro Yaw"), title("Yaw"), grid
figure
subplot(3, 1, 1), plot(time2, refAccel(1, 1:L2)), grid, title("Accel x in reference frame")
subplot(3, 1, 2), plot(time2, refAccel(2, 1:L2)), grid, title("Accel y in reference frame")
subplot(3, 1, 3), plot(time2, refAccel(3, 1:L2)), grid, title("Accel z in reference frame")
figure
subplot(3, 1, 1), plot(time, Roll(1:L)), grid, title("Roll")
subplot(3, 1, 2), plot(time, Pitch(1:L)), grid, title("Pitch")
subplot(3, 1, 3), plot(time, Yaw(1:L)), grid, title("Yaw")
figure
subplot(3, 1, 1), plot(time, gyroRoll(1:L)), grid, title("Gyro Roll"), legend("Gyro Roll")
subplot(3, 1, 2), plot(time, gyroPitch(1:L)), grid, title("Gyro Pitch"), legend("Gyro Pitch")
subplot(3, 1, 3), plot(time, gyroYaw(1:L)), grid, title("Gyro Yaw"), legend("Gyro Yaw")
figure
subplot(3, 1, 1), plot(time, accelRoll(1:L)), grid, title("Inclination X")
subplot(3, 1, 2), plot(time, accelPitch(1:L)), grid, title("Inclination Y")
subplot(3, 1, 3), plot(time, magYaw(1:L), time, transformedMagYaw), grid, title("Inclination Z"), legend("Mag Yaw", "Rotated Mag Yaw")
% figure
% subplot(1, 3, 1), plot(time, CFrotatedMagX(1:L)), grid, title("CF Rotated Magnetic Field X")
% subplot(1, 3, 2), plot(time, CFrotatedMagY(1:L)), grid, title("CF Rotated Magnetic Field Y")
% subplot(1, 3, 3), plot(time, CFrotatedMagZ(1:L)), grid, title("CF Rotated Magnetic Field Z")
figure
subplot(3, 1, 1), plot(time, rotatedMagX(1:L)), grid, title("Rotated Magnetic Field X")
subplot(3, 1, 2), plot(time, rotatedMagY(1:L)), grid, title("Rotated Magnetic Field Y")
subplot(3, 1, 3), plot(time, rotatedMagZ(1:L)), grid, title("Rotated Magnetic Field Z")
figure
subplot(3, 1, 1), plot(time, magX(1:L)), grid, title("Magnetic Field X")
subplot(3, 1, 2), plot(time, magY(1:L)), grid, title("Magnetic Field Y")
subplot(3, 1, 3), plot(time, magZ(1:L)), grid, title("Magnetic Field Z")
figure
subplot()
subplot(3, 1, 1), plot(time, calibratedAngularX(1:L), "red"), grid, title("Angular Velocity X"), legend("GyroX")
subplot(3, 1, 2), plot(time, calibratedAngularY(1:L), "red"), grid, title("Angular Velocity Y"), legend("GyroY")
subplot(3, 1, 3), plot(time, calibratedAngularZ(1:L), "red"), grid, title("Angular Velocity Z"), legend("GyroZ")
figure
subplot(3, 1, 1), plot(time, calibratedAccelX(1:L), time, filteredAccelX(1:L)), grid, title("Acceleration X"), legend("Calibrated", "Filtered")
subplot(3, 1, 2), plot(time, calibratedAccelY(1:L), time, filteredAccelY(1:L)), grid, title("Acceleration Y"), legend("Calibrated", "Filtered")
subplot(3, 1, 3), plot(time, calibratedAccelZ(1:L), time, filteredAccelZ(1:L)), grid, title("Acceleration Z"), legend("Calibrated", "Filtered")
if TEST
    subplot(3, 1, 1), plot(time, testRoll, time, Roll, time, errorRateRoll), grid, title('Roll Error Rate')
    subplot(3, 1, 2), plot(time, testPitch, time, Pitch, time, errorRatePitch), grid, title('Pitch Error Rate')
    subplot(3, 1, 3), plot(time, testYaw, time, Yaw, time, errorRateYaw), grid, title('Yaw Error Rate')
end