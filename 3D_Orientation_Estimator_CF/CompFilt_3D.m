close all
%transform data so we can process it
Accel = table2array(Acceleration);
AngVel = table2array(AngularVelocity);
MagField = table2array(MagneticField);

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
DO_CALIBRATION = 1;
if DO_CALIBRATION
    CalSamples = 200;
    CalibrationAx = sum(accelX(1:CalSamples)) / CalSamples;
    CalibrationAy = sum(accelY(1:CalSamples)) / CalSamples;
    CalibrationAz = 0;
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
time = 0 : delta_T : (L - 1) * delta_T;
rad2deg = 57.29578;
deg2rad = 0.0174532925;
%Calculate absolute angles from accelero- and magnetometer
accelRollInRads = atan2(calibratedAccelY, accelZ);
accelPitchInRads = -atan2(calibratedAccelX, accelZ); %the atan2 functions get a minus sign
                                                    %so CCW rotation will correspond to negative change in the angles
magYawInRads = atan2(magY, magX);
accelRoll = -accelRollInRads * rad2deg;
accelPitch = -accelPitchInRads * rad2deg;
magYaw = magYawInRads * rad2deg;
magYawCal = sum(magYaw(1:300)) / 300;
magYaw = magYaw - magYawCal;

%Transform magnetic readings from inertial system to frame of reference
rotatedMagX = zeros(1, L);
rotatedMagY = zeros(1, L);
rotatedMagZ = zeros(1, L);
for i = 1 : L
    rotatedMagX(i) = magX(i) * cos(accelPitchInRads(i)) + magZ(i) * sin(accelPitchInRads(i));
    rotatedMagY(i) = magY(i) * cos(accelRollInRads(i)) + magX(i) * sin(accelRollInRads(i)) * sin(accelPitchInRads(i)) - magZ(i) * sin(accelRollInRads(i)) * cos(accelPitchInRads(i));
    rotatedMagZ(i) = magY(i) * sin(accelRollInRads(i)) - magX(i) * sin(accelPitchInRads(i)) * cos(accelRollInRads(i)) + magZ(i) * cos(accelPitchInRads(i)) * cos(accelRollInRads(i));
end
transformedMagYaw = atan2(rotatedMagY, rotatedMagX);
transformedMagYaw = transformedMagYaw * rad2deg;
% transformedMagYawCal = sum(transformedMagYaw(1:300)) / 300;
transformedMagYawCal = 0;
transformedMagYaw = transformedMagYaw - transformedMagYawCal;

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
        gyroRoll(i) = calibratedAngularX(i) * delta_T;
        gyroPitch(i) = - calibratedAngularY(i) * delta_T;
        gyroYaw(i) = calibratedAngularZ(i) * delta_T;
    else
        gyroRoll(i) = gyroRoll(i - 1) - calibratedAngularX(i) * delta_T;
        gyroPitch(i) = gyroPitch(i - 1) - calibratedAngularY(i) * delta_T;
        gyroYaw(i) = gyroYaw(i - 1) -  (calibratedAngularZ(i) * delta_T);
    end
end
gyroRoll = gyroRoll * rad2deg;
gyroPitch = gyroPitch * rad2deg;
gyroYaw = gyroYaw * rad2deg;

%Complementary Filter(sensor fusion)
%Calculates Euler angles in frame of reference
ROTATE_GYRO = 0;
Alpha = 0.98;
Beta = 1 - Alpha;
Roll = zeros(1,L);
Pitch = zeros(1,L);
Yaw = zeros(1,L);
gyroInstantaneous= zeros(3, 1);
gyroRotated = zeros(3, 1);
rotatedGyroX = zeros(1, L);
rotatedGyroY = zeros(1, L);
rotatedGyroZ = zeros(1, L);
if ROTATE_GYRO == 0
    for i = 1:L
       if i == 1
           Roll(i) = Alpha * (gyroRollInitial + calibratedAngularX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
           Pitch(i) = Alpha * (gyroPitchInitial - calibratedAngularY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
           Yaw(i) = Alpha * (gyroYawInitial - calibratedAngularZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
       else    
       Roll(i) =  Alpha * (Roll(i - 1) + calibratedAngularX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
       Pitch(i) = Alpha * (Pitch(i - 1) - calibratedAngularY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
       Yaw(i) = Alpha * (Yaw(i - 1) - calibratedAngularZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
       end
    end
end
if ROTATE_GYRO == 1
    for i = 1:L
        gyroInstantaneous(1, 1) = calibratedAngularX(i);
        gyroInstantaneous(2, 1) = calibratedAngularY(i);
        gyroInstantaneous(3, 1) = calibratedAngularZ(i);
        if i == 1
            Rz = [cos(0), -sin(0), 0; sin(0), cos(0), 0; 0, 0, 1];
            Ry = [cos(0), 0, -sin(0); 0, 1, 0; sin(0), 0, cos(0)];
            Rx = [1, 0, 0; 0, cos(0), -sin(0); 0, sin(0), cos(0)];
            gyroRotated = Rz * gyroInstantaneous;
            gyroRotated = Ry * gyroRotated;
            gyroRotated = Rx * gyroRotated;
            rotatedGyroX(i) = gyroRotated(1, 1);
            rotatedGyroY(i) = gyroRotated(2, 1);
            rotatedGyroZ(i) = gyroRotated(3, 1);
            Roll(i) = Alpha * (gyroRollInitial - rotatedGyroX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
            Pitch(i) = Alpha * (gyroPitchInitial - rotatedGyroY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
            Yaw(i) = Alpha * (gyroYawInitial + rotatedGyroZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
        else
            Rz = [cos(Yaw(i - 1) * deg2rad), -sin(Yaw(i - 1) * deg2rad), 0; sin(Yaw(i - 1) * deg2rad), cos(Yaw(i - 1) * deg2rad), 0; 0, 0, 1];
            Ry = [cos(Pitch(i - 1) * deg2rad), 0, -sin(Pitch(i - 1) * deg2rad); 0, 1, 0; sin(Pitch(i - 1) * deg2rad), 0, cos(Pitch(i - 1) * deg2rad)];
            Rx = [1, 0, 0; 0, cos(Roll(i - 1) * deg2rad), -sin(Roll(i - 1) * deg2rad); 0, sin(Roll(i - 1) * deg2rad), cos(Roll(i - 1) * deg2rad)];
            gyroRotated = Rz * gyroInstantaneous;
            gyroRotated = Ry * gyroRotated;
            gyroRotated = Rx * gyroRotated;
            rotatedGyroX(i) = gyroRotated(1, 1);
            rotatedGyroY(i) = gyroRotated(2, 1);
            rotatedGyroZ(i) = gyroRotated(3, 1);
            Roll(i) =  Alpha * (Roll(i - 1) - rotatedGyroX(i) * rad2deg * delta_T) + Beta * accelRoll(i);
            Pitch(i) = Alpha * (Pitch(i - 1) - rotatedGyroY(i) * rad2deg * delta_T) + Beta * accelPitch(i);
            Yaw(i) = Alpha * (Yaw(i - 1) + rotatedGyroZ(i) * rad2deg * delta_T) + Beta * transformedMagYaw(i);
        end
    end
end
rotatedGyroRoll = zeros(1, L);
rotatedGyroPitch = zeros(1, L);
rotatedGyroYaw = zeros(1, L);
for i = 1:L
    if i == 1
        rotatedGyroRoll(i) = - rotatedGyroX(i) * delta_T * rad2deg;
        rotatedGyroPitch(i) = - rotatedGyroY(i) * delta_T * rad2deg;
        rotatedGyroYaw(i) = + rotatedGyroZ(i) * delta_T * rad2deg;
    else
        rotatedGyroRoll(i) = rotatedGyroRoll(i - 1) - rotatedGyroX(i) * delta_T * rad2deg;
        rotatedGyroPitch(i) = rotatedGyroPitch(i - 1) - rotatedGyroY(i) * delta_T * rad2deg;
        rotatedGyroYaw(i) = rotatedGyroYaw(i - 1) + rotatedGyroZ(i) * delta_T * rad2deg;
    end
end

% for i = 1 : L
%     rotatedMagX(i) = magX(i) * cos(Pitch(i) * deg2rad) + magZ(i) * sin(Pitch(i)  * deg2rad);
%     rotatedMagY(i) = magY(i) * cos(Roll(i) * deg2rad) + magX(i) * sin(Roll(i) * deg2rad) * sin(Pitch(i) * deg2rad) - magZ(i) * sin(Roll(i) * deg2rad) * cos(Pitch(i) * deg2rad);
%     rotatedMagZ(i) = magY(i) * sin(Roll(i) * deg2rad) - magX(i) * sin(Pitch(i) * deg2rad) * cos(Roll(i) * deg2rad) + magZ(i) * cos(Pitch(i) * deg2rad) * cos(Roll(i) * deg2rad);
% end
% transformedMagYaw = atan2(rotatedMagY, rotatedMagX);
% transformedMagYaw = transformedMagYaw * rad2deg;
% gyroYawInitial = sum(transformedMagYaw(1:300)) / 300;
% for i = 1:L
%    if i == 1 
%        Yaw(i) = Alpha * (gyroYawInitial - angularZ(i) * delta_T) + Beta * transformedMagYaw(i);
%    else
%        Yaw(i) = Alpha * (Yaw(i - 1) + angularZ(i) * delta_T) + Beta * transformedMagYaw(i);
%    end
% end

%Rotate magnetic readings with the output of CF
% CFrotatedMagX = zeros(1, L);
% CFrotatedMagY = zeros(1, L);
% CFrotatedMagZ = zeros(1, L);
% for i = 1 : L
% CFrotatedMagX(i) = magX(i) * cos(Roll(i) * deg2rad) + magZ(i) * sin(Pitch(i) * deg2rad);
% CFrotatedMagY(i) = magY(i) * cos(Roll(i) * deg2rad) + magX(i) * sin(Roll(i) * deg2rad) * sin(Pitch(i) * deg2rad) - magZ(i) * sin(Roll(i) * deg2rad) * cos(Pitch(i) * deg2rad);
% CFrotatedMagZ(i) = magY(i) * sin(Roll(i) * deg2rad) - magX(i) * sin(Pitch(i) * deg2rad) * cos(Roll(i) * deg2rad) + magZ(i) * cos(Pitch(i) * deg2rad) * cos(Roll(i) * deg2rad);
% end

%Plot the results
figure(8)
subplot(1, 3, 1), plot(time, Roll(1:L), "red", time, accelRoll(1:L), "green", time, gyroRoll(1:L), "blue", time, rotatedGyroRoll), legend("Roll", "Accel Roll", "Gyro Roll", "EulerX"), title("Roll"), grid
subplot(1, 3, 2), plot(time, Pitch(1:L), "red", time, accelPitch(1:L), "green", time, gyroPitch(1:L), "blue", time, rotatedGyroPitch), legend("Pitch", "Accel Pitch", "Gyro Pitch", "EulerY"), title("Pitch"), grid
subplot(1, 3, 3), plot(time, Yaw(1:L), "red", time, transformedMagYaw(1:L), "green", time, gyroYaw(1:L), "blue", time, rotatedGyroYaw), legend("Yaw", "Mag Yaw", "Gyro Yaw", "EulerZ"), title("Yaw"), grid
figure(6)
subplot(1, 3, 1), plot(time, Roll(1:L)), grid, title("Roll")
subplot(1, 3, 2), plot(time, Pitch(1:L)), grid, title("Pitch")
subplot(1, 3, 3), plot(time, Yaw(1:L)), grid, title("Yaw")
figure(5)
subplot(1, 3, 1), plot(time, gyroRoll(1:L), time, rotatedGyroRoll), grid, title("Gyro Roll"), legend("Gyro Roll", "EulerX")
subplot(1, 3, 2), plot(time, gyroPitch(1:L), time, rotatedGyroPitch), grid, title("Gyro Pitch"), legend("Gyro Pitch", "EulerY")
subplot(1, 3, 3), plot(time, gyroYaw(1:L), time, rotatedGyroYaw), grid, title("Gyro Yaw"), legend("Gyro Yaw", "EulerZ")
figure(4)
subplot(1, 3, 1), plot(time, accelRoll(1:L)), grid, title("Inclination X")
subplot(1, 3, 2), plot(time, accelPitch(1:L)), grid, title("Inclination Y")
subplot(1, 3, 3), plot(time, magYaw(1:L), time, transformedMagYaw), grid, title("Inclination Z"), legend("Mag Yaw", "Rotated Mag Yaw")
% figure(9)
% subplot(1, 3, 1), plot(time, CFrotatedMagX(1:L)), grid, title("CF Rotated Magnetic Field X")
% subplot(1, 3, 2), plot(time, CFrotatedMagY(1:L)), grid, title("CF Rotated Magnetic Field Y")
% subplot(1, 3, 3), plot(time, CFrotatedMagZ(1:L)), grid, title("CF Rotated Magnetic Field Z")
figure(7)
subplot(1, 3, 1), plot(time, rotatedMagX(1:L)), grid, title("Rotated Magnetic Field X")
subplot(1, 3, 2), plot(time, rotatedMagY(1:L)), grid, title("Rotated Magnetic Field Y")
subplot(1, 3, 3), plot(time, rotatedMagZ(1:L)), grid, title("Rotated Magnetic Field Z")
figure(3)
subplot(1, 3, 1), plot(time, magX(1:L)), grid, title("Magnetic Field X")
subplot(1, 3, 2), plot(time, magY(1:L)), grid, title("Magnetic Field Y")
subplot(1, 3, 3), plot(time, magZ(1:L)), grid, title("Magnetic Field Z")
figure(2)
subplot()
subplot(1, 3, 1), plot(time, calibratedAngularX(1:L), time, rotatedGyroX), grid, title("Angular Velocity X"), legend("GyroX", "EulerRateX")
subplot(1, 3, 2), plot(time, calibratedAngularY(1:L), time, rotatedGyroY), grid, title("Angular Velocity Y"), legend("GyroY", "EulerRateY")
subplot(1, 3, 3), plot(time, calibratedAngularZ(1:L), time, rotatedGyroZ), grid, title("Angular Velocity Z"), legend("GyroZ", "EulerRateZ")
figure(1)
subplot(1, 3, 1), plot(time, calibratedAccelX(1:L)), grid, title("Acceleration X")
subplot(1, 3, 2), plot(time, calibratedAccelY(1:L)), grid, title("Acceleration Y")
subplot(1, 3, 3), plot(time, calibratedAccelZ(1:L)), grid, title("Acceleration Z")