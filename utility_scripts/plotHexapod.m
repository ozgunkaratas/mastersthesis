set(groot,'defaultAxesTickLabelInterpreter','latex');  

%hexapod values
plot(recordedTime,ax)
title('Acceleration X-axis (Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,ay)
title('Acceleration Y-axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,az)
title('Acceleration Z-axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,wbx)
title('Angular Velocity X axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,wby)
title('Angular Velocity Y axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,wbz)
title('Angular Velocity Z axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)




plot(recordedTime,vx)
title('Velocity X-axis (Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,vy)
title('Velocity Y-axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,vz)
title('Velocity Z-axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,posX)
title('Position X axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position(mm)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,posY)
title('Position Y axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position(mm)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(recordedTime,posZ)
title('Position Z axis(Hex)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position (mm)', 'Interpreter', 'latex', 'Fontsize', 16)




















plot(hexEpochpyTime-hexEpochpyTime(1),hexaccX)
title('Acceleration X axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime-hexEpochpyTime(1),hexaccY)
title('Acceleration Y axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime-hexEpochpyTime(1),hexaccZ)
title('Acceleration Z axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Accel (ms-2)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime-hexEpochpyTime(1),hexgyrX)
title('Angular Velocity  X axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime-hexEpochpyTime(1),hexgyrY)
title('Angular Velocity Y axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime-hexEpochpyTime(1),hexgyrZ)
title('Angular Velocity Z axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('ang vel(rads-1)', 'Interpreter', 'latex', 'Fontsize', 16)









plot(hexEpochpyTime(1:52418)-hexEpochpyTime(1),hexvx(1:52418))
title('Velocity X axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime(1:52418)-hexEpochpyTime(1),hexvy(1:52418))
title(' Velocity Y axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime(1:52418)-hexEpochpyTime(1),hexvz(1:52418))
title('Velocity Z axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Velocity (ms-1)', 'Interpreter', 'latex', 'Fontsize', 16)


plot(hexEpochpyTime(1:52417)-hexEpochpyTime(1),hexpx(1:52417))
title('Position X axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position (mm)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime(1:52417)-hexEpochpyTime(1),hexpy(1:52417))
title('Position Y axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position (mm)', 'Interpreter', 'latex', 'Fontsize', 16)

plot(hexEpochpyTime(1:52417)-hexEpochpyTime(1),hexpz(1:52417))
title('Position Z axis(IMU)', 'Interpreter', 'latex', 'Fontsize', 16)
xlabel('Time (s)', 'Interpreter', 'latex', 'Fontsize', 16)
ylabel('Position (mm)', 'Interpreter', 'latex', 'Fontsize', 16)


