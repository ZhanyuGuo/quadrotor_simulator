%% test quadrotor motion equation

%% clear before running
close all; clear; clc;

%% add path
addpath(genpath('./controller'));

%% initialization
% time
time   = 0;      % current time
s_time = 0;      % time of save
t_step = 0.002;  % Time step for solving equations of motion
s_step = 0.01;   % save interval
c_step = 0.01;   % control time
t_M    = 10;     % total simulate time


params = quadModel_readonly();
start  = [0; 0; 0];
true_s = init_state(start);

F = params.mass*params.grav;
M = [0; 0; 0];
% External disturbance
params.Fd = zeros(3, 1);
params.Md = zeros(3, 1);

t_list = [];  % time span
s_list = [];  % state
u_list = [];  % input
a_list = [];  % angle
d_list = [];  % disturbance

%% simulation
disp('Start Simulation ...');
while (1)
    if time > t_M
        disp('Finished!');
        break
    end

    % impuse F
%     if time >= t_M / 2 && time <= t_M / 2 + s_step
%         F = 1e6;
%     else
%         F = params.mass*params.grav;
%     end

    % step F
%     if time >= t_M / 2
%         F = 1.1*params.mass*params.grav;
%     else
%         F = params.mass*params.grav;
%     end
    
    % sin F
%     F = sin(time) + params.mass*params.grav;

    % impuse Mx
%     if time >= t_M / 2 && time <= t_M / 2 + s_step
%         M(1) = 1e6;
%     else
%         M(1) = 0;
%     end

    % step Mx
%     if time >= t_M / 2
%         M(1) = 0.005;
%     else
%         M(1) = 0;
%     end

    % sin Mx
%     M(1) = 0.005 * sin(time);

    % impuse Mz
%     if time >= t_M / 2 && time <= t_M / 2 + s_step
%         M(3) = 5;
%     else
%         M(3) = 0;
%     end

    % step Mz
%     if time >= t_M / 2
%         M(3) = 0.005;
%     else
%         M(3) = 0;
%     end

    % sin Mz
%     M(3) = 0.005 * sin(time);

    % impuse Fdx
%     if time >= t_M / 2 && time <= t_M / 2 + s_step
%         params.Fd(1) = 1e6;
%     else
%         params.Fd(1) = 0;
%     end

    % step Fdx
%     if time >= t_M / 2
%         params.Fd(1) = 1;
%     else
%         params.Fd(1) = 0;
%     end

    % sin Fdx
%     params.Fd(1) = sin(time);

    timeint = time: t_step: time + c_step;
    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F, M, params), timeint', true_s);
    true_s = xsave(end, :)';
    
    if time - s_time >= s_step
        s_time = time;
        t_list = [t_list; time];
        s_list = [s_list; true_s'];
        u_list = [u_list; F, M'];
        a = RotToRPY_ZXY_wrapper(QuatToRot(true_s(7:10)))';
        a_list = [a_list; a];
        d_list = [d_list; params.Fd', params.Md'];
    end
    time = time + c_step;
end

%% visualization
f1 = figure(1); sgtitle('Quadrotor Simulation');
subplot(4, 2, 1), hold on, grid on;
plot(t_list, s_list(:, 1), 'Color', 'r');
plot(t_list, s_list(:, 2), 'Color', 'g');
plot(t_list, s_list(:, 3), 'Color', 'b');
xlabel('t (s)'), ylabel('Position (m)');
legend('x', 'y', 'z');
hold off;

subplot(4, 2, 2), hold on, grid on;
plot(t_list, u_list(:, 1));
xlabel('t (s)'), ylabel('Force (N)');
hold off;

subplot(4, 2, 3), hold on, grid on;
plot(t_list, s_list(:, 4), 'Color', 'r');
plot(t_list, s_list(:, 5), 'Color', 'g');
plot(t_list, s_list(:, 6), 'Color', 'b');
xlabel('t (s)'), ylabel('Velocity (m/s)');
legend('vx', 'vy', 'vz');
hold off;

subplot(4, 2, 4), hold on, grid on;
plot(t_list, u_list(:, 2), 'Color', 'r');
plot(t_list, u_list(:, 3), 'Color', 'g');
plot(t_list, u_list(:, 4), 'Color', 'b');
xlabel('t (s)'), ylabel('Moment (N*m)');
legend('Mx', 'My', 'Mz');
hold off;

subplot(4, 2, 5), hold on, grid on;
plot(t_list, a_list(:, 1), 'Color', 'r');
plot(t_list, a_list(:, 2), 'Color', 'g');
plot(t_list, a_list(:, 3), 'Color', 'b');
xlabel('t (s)'), ylabel('Angle (rad)');
legend('\phi', '\theta', '\psi');
hold off;

subplot(4, 2, 6), hold on, grid on;
plot(t_list, d_list(:, 1), 'Color', 'r');
plot(t_list, d_list(:, 2), 'Color', 'g');
plot(t_list, d_list(:, 3), 'Color', 'b');
xlabel('t (s)'), ylabel('Disturbe Force (N)');
legend('Fdx', 'Fdy', 'Fdz');
hold off;

subplot(4, 2, 7), hold on, grid on;
plot(t_list, s_list(:, 11), 'Color', 'r');
plot(t_list, s_list(:, 12), 'Color', 'g');
plot(t_list, s_list(:, 13), 'Color', 'b');
xlabel('t (s)'), ylabel('Angular Velocity (rad/s)');
legend('$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$', 'Interpreter', 'latex');
hold off;

subplot(4, 2, 8), hold on, grid on;
plot(t_list, d_list(:, 4), 'Color', 'r');
plot(t_list, d_list(:, 5), 'Color', 'g');
plot(t_list, d_list(:, 6), 'Color', 'b');
xlabel('t (s)'), ylabel('Disturbe Moment (N*m)');
legend('Mdx', 'Mdy', 'Mdz');
hold off;