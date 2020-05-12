clear all; close all; clc;

pos = readmatrix("position");
t = pos(:,1);
pos_x = pos(:,2);
ref_pos_x = pos(:,3);
pos_y = pos(:,4);
ref_pos_y = pos(:,5);
pos_z = pos(:,6);
ref_pos_z = pos(:,7);

figure(1)
plot3(pos_x, pos_y, pos_z, 'linewidth', 2); hold on;
plot3(ref_pos_x, ref_pos_y, ref_pos_z, "--", 'linewidth', 2);
scatter3(pos_x(1), pos_y(1), pos_z(1), 'filled');
scatter3(pos_x(20000), pos_y(20000), pos_z(20000), 'filled');
title("Position tracking");
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
set(gcf,'position',[500,500,700,400])

att = readmatrix("attitude");
t = att(:,1);
roll = att(:,2);
roll_ref = att(:,3);
roll_cmd = att(:,4);
pitch = att(:,5);
pitch_ref = att(:,6);
pitch_cmd = att(:,7);

figure(2)
subplot(2,1,1)
plot(t, roll * 180/pi, "linewidth", 1); hold on
plot(t, roll_cmd * 180/pi, "--", "linewidth", 1)
ylabel("Roll [deg]")
xlabel("time [s]")
xline(20, "--")
ylim([-25,25])
grid on
subplot(2,1,2)
plot(t, pitch * 180/pi, "linewidth", 1); hold on
plot(t, pitch_cmd * 180/pi, "--", "linewidth", 1)
xlabel("time [s]")
ylabel("Pitch [deg]")
xline(20, "--")
ylim([-25,25])
grid on
set(gcf,'position',[500,500,700,400])

% Closeup
figure(3)
subplot(2,1,1)
plot(t, roll * 180/pi, "linewidth", 1); hold on
plot(t, roll_ref * 180/pi, "--", "linewidth", 1); hold on
plot(t, roll_cmd * 180/pi, "--", "linewidth", 1)
ylabel("Roll [deg]")
xlabel("time [s]")
xline(20, "--")
xlim([32 38])
ylim([-25,25])
grid on
subplot(2,1,2)
plot(t, pitch * 180/pi, "linewidth", 1); hold on
plot(t, pitch_ref * 180/pi, "--", "linewidth", 1); hold on
plot(t, pitch_cmd * 180/pi, "--", "linewidth", 1)
xlabel("time [s]")
ylabel("Pitch [deg]")
xline(20, "--")
xlim([32 38])
ylim([-25,25])
grid on
set(gcf,'position',[500,500,700,400])

inputs = readmatrix("inputs");
t = inputs(:,1);
tau_x = inputs(:,2);
limit_x = inputs(:,3);
tau_y = inputs(:,4);
limit_y = inputs(:,5);

figure(4)
subplot(2,1,1)
plot(t, tau_x, "linewidth", 1); hold on
plot(t, limit_x, "--"); hold on
plot(t, -limit_x, "--");
xlabel("time [s]")
ylabel("Input torque x [Nm]")
ylim([-5,5])
grid on
subplot(2,1,2)
plot(t, tau_x, "linewidth", 1); hold on
plot(t, limit_x, "--"); hold on
plot(t, -limit_x, "--");
xlabel("time [s]")
ylabel("Input torque y [Nm]")
ylim([-5,5])
grid on
set(gcf,'position',[500,500,700,400])

adaptive_params = readmatrix("adaptive_params");
t = adaptive_params(:,1);
tau_dist_x = adaptive_params(:,2);
tau_dist_y = adaptive_params(:,3);
tau_dist_z = adaptive_params(:,4);

figure(5)
subplot(2,1,1)
plot(t, tau_dist_x, "linewidth", 1); hold on
xlabel("time [s]")
ylabel("Est. disturbance x [Nm]")
ylim([-5,5])
xline(20, "--")
grid on
subplot(2,1,2)
plot(t, tau_dist_y, "linewidth", 1); hold on
xlabel("time [s]")
ylabel("Est. disturbance y [Nm]")
ylim([-5,5])
xline(20, "--")
grid on
set(gcf,'position',[500,500,700,400])
