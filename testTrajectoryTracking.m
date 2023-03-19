%% test quadrotor trajectory generation and tracking in 3d

clc; clear; close all;

% path
addpath(genpath('./trajectory_generation/'), genpath('./controller'));

% configurations
display_ratio   = 1.25;
figure_width    = 1920 / display_ratio;
figure_height   = 1080 / display_ratio;
figure_size     = 800 / display_ratio;
figure_position = [
    0.5*(figure_width - figure_size), ...
    0.5*(figure_height - figure_size), ...
    figure_size, ...
    figure_size];

f1 = figure(1);
set(f1, 'position', figure_position, 'Renderer', 'painters');
axis ([-5, 5, -5, 5, -1, 4]); grid on; hold on;

t_M         = 30;   % total time
c_step      = 0.01; % control time

% set points
waypoints = setPoints3(f1);

% get minimum snap trajectory
[poly_coef_x, poly_coef_y, poly_coef_z, ts, n_order, n_seg] = getMinimumSnap3(waypoints, t_M);

% extract from polynomial
k = 1;
for i = 0: n_seg - 1
    Pxi = flipud(poly_coef_x((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    Pyi = flipud(poly_coef_y((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    Pzi = flipud(poly_coef_z((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    for t = 0: c_step: ts(i + 1)
        x_des(k) = polyval(Pxi, t);
        y_des(k) = polyval(Pyi, t);
        z_des(k) = polyval(Pzi, t);
%         z_des(k) = 0.1 * k * cstep;
        dx_des(k) = polyval(polyder(Pxi), t);
        dy_des(k) = polyval(polyder(Pyi), t);
        dz_des(k) = polyval(polyder(Pzi), t);
%         dz_des(k) = 0.1;
        k = k + 1;
    end
end

% plot desired trajectory
trj_1 = plot3(x_des, y_des, z_des, 'Color', 'g', 'LineWidth', 2);

% sensor parameters
fnoise  = 0.1;         % Standard deviation of gaussian noise for external disturbance (N)
ifov    = 90;          % Camera field of view  

% Initialize simulation
global params;
params  = quadModel_readonly(); % Quad model
x0      = zeros(13, 1);
yaw0    = 0 * pi / 180;
pitch0  = 0 * pi / 180;
roll0   = 0 * pi / 180;
x0(1)   = x_des(1) + 1.0; % x
x0(2)   = y_des(1) - 1.0; % y
x0(3)   = z_des(1) - 0.5; % z
% x0(1)   = 0.0; % x
% x0(2)   = 0.0; % y
% x0(3)   = 0.0; % z
x0(4)   = 0; % xdot
x0(5)   = 0; % ydot
x0(6)   = 0; % zdot
Quat0   = R_to_quaternion(yrp_to_R([yaw0 roll0 pitch0])');
x0(7)   = Quat0(1); % qw
x0(8)   = Quat0(2); % qx
x0(9)   = Quat0(3); % qy
x0(10)  = Quat0(4); % qz
x0(11)  = 0;        % p
x0(12)  = 0;        % q
x0(13)  = 0;        % r
true_s  = x0;       % true state
F       = params.mass * params.grav;
M       = [0; 0; 0];

% time
time        = 0;      % current time
vis_time    = 0;      % Time of last visualization
% t_step      = 0.002;  % Time step for solving equations of motion
v_step      = 0.05;   % visualization interval

% Visualization
vis_init   = false;

% h1
thtraj     = [];
thprop1    = [];
thprop2    = [];
thprop3    = [];
thprop4    = [];
tharm1     = [];
tharm2     = [];
thfov1     = [];
thfov2     = [];
thfov3     = [];
thfov4     = [];
ehtraj     = [];
ehprop1    = [];
ehprop2    = [];
ehprop3    = [];
ehprop4    = [];
eharm1     = [];
eharm2     = [];
ehmap      = [];
ehwindow   = [];

% Start Simulation run_trajectory_readonly
disp('Start Simulation ...');
while (1)
    % External disturbance
    Fd = randn(3, 1) * fnoise;

    % Run simulation for cstep
%     timeint = time: t_step: time + c_step;
    timeint = [time, time + c_step];
%     [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F, M, Fd), timeint', true_s);
    [~, xsave] = ode45(@(t, s) quadEOM(t, s, F, M, Fd), timeint', true_s);
    true_s = xsave(end, :)';
    time = time + c_step;
    
%     des_s = circle_trajectory(time, true_s);
    des_s = zeros(13, 1);
    k = floor(time / c_step);
    des_s(1) = x_des(k);
    des_s(2) = y_des(k);
    des_s(3) = z_des(k);
    des_s(4) = dx_des(k);
    des_s(5) = dy_des(k);
    des_s(6) = dz_des(k);
    des_yaw  = mod(0.1 * pi * time, 2 * pi);
    yrp = [des_yaw, 0.0, 0.0];
    Rot = yrp_to_R(yrp);
    des_q = R_to_quaternion(Rot);
    des_s(7:10) = des_q;

    [F, M] = controller(time, true_s, des_s);

    if time >= t_M
        break;
    end

    if time - vis_time > v_step
%         if ~vis_init
%             grid on;
%             axis equal;
%             axis ([-5, 5, -5, 5, -1, 4]);
%         end
        ll = 0.175;
        rr = 0.1;
        ff = 0.3;
        nprop = 40;
        propangs = linspace(0,2*pi,nprop);
        tR = QuatToRot(true_s(7:10))';
        tpoint1 = tR*[ll;0;0];
        tpoint2 = tR*[0;ll;0];
        tpoint3 = tR*[-ll;0;0];
        tpoint4 = tR*[0;-ll;0];
        tproppts = rr*tR*[cos(propangs);sin(propangs);zeros(1,nprop)];
        twp1 = true_s(1:3) + tpoint1;
        twp2 = true_s(1:3) + tpoint2;
        twp3 = true_s(1:3) + tpoint3;
        twp4 = true_s(1:3) + tpoint4;
        tprop1 = tproppts + twp1*ones(1,nprop);
        tprop2 = tproppts + twp2*ones(1,nprop);
        tprop3 = tproppts + twp3*ones(1,nprop);
        tprop4 = tproppts + twp4*ones(1,nprop);
        tfov0 = true_s(1:3);
        tfov1 = tR * [ff;  ff * tan(ifov*pi/180/2);  ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov2 = tR * [ff;  ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2);  ff * tan(ifov*pi/180/2)] + true_s(1:3);
        eR = QuatToRot(des_s(7:10))';
        epoint1 = eR*[ll;0;0];
        epoint2 = eR*[0;ll;0];
        epoint3 = eR*[-ll;0;0];
        epoint4 = eR*[0;-ll;0];
        eproppts = rr*eR*[cos(propangs);sin(propangs);zeros(1,nprop)];
        ewp1 = des_s(1:3) + epoint1;
        ewp2 = des_s(1:3) + epoint2;
        ewp3 = des_s(1:3) + epoint3;
        ewp4 = des_s(1:3) + epoint4;
        eprop1 = eproppts + ewp1*ones(1,nprop);
        eprop2 = eproppts + ewp2*ones(1,nprop);
        eprop3 = eproppts + ewp3*ones(1,nprop);
        eprop4 = eproppts + ewp4*ones(1,nprop);
        if ~vis_init
            thtraj = plot3(true_s(1), true_s(2), true_s(3), 'b-','LineWidth',3);
            tharm1 = line([twp1(1),twp3(1)],[twp1(2),twp3(2)],[twp1(3),twp3(3)],'Color','b');
            tharm2 = line([twp2(1),twp4(1)],[twp2(2),twp4(2)],[twp2(3),twp4(3)],'Color','b');
            thprop1 = plot3(tprop1(1,:),tprop1(2,:),tprop1(3,:),'r-');
            thprop2 = plot3(tprop2(1,:),tprop2(2,:),tprop2(3,:),'b-');
            thprop3 = plot3(tprop3(1,:),tprop3(2,:),tprop3(3,:),'b-');
            thprop4 = plot3(tprop4(1,:),tprop4(2,:),tprop4(3,:),'b-');
            thfov1 = line([tfov0(1) tfov1(1) tfov2(1)], [tfov0(2) tfov1(2) tfov2(2)], [tfov0(3) tfov1(3) tfov2(3)],'Color','k');
            thfov2 = line([tfov0(1) tfov2(1) tfov3(1)], [tfov0(2) tfov2(2) tfov3(2)], [tfov0(3) tfov2(3) tfov3(3)],'Color','k');
            thfov3 = line([tfov0(1) tfov3(1) tfov4(1)], [tfov0(2) tfov3(2) tfov4(2)], [tfov0(3) tfov3(3) tfov4(3)],'Color','k');
            thfov4 = line([tfov0(1) tfov4(1) tfov1(1)], [tfov0(2) tfov4(2) tfov1(2)], [tfov0(3) tfov4(3) tfov1(3)],'Color','k');
            ehtraj = plot3(des_s(1), des_s(2), des_s(3), 'g-','LineWidth',3);
            eharm1 = line([ewp1(1),ewp3(1)],[ewp1(2),ewp3(2)],[ewp1(3),ewp3(3)],'Color','g');
            eharm2 = line([ewp2(1),ewp4(1)],[ewp2(2),ewp4(2)],[ewp2(3),ewp4(3)],'Color','g');
            ehprop1 = plot3(eprop1(1,:),eprop1(2,:),eprop1(3,:),'m-');
            ehprop2 = plot3(eprop2(1,:),eprop2(2,:),eprop2(3,:),'g-');
            ehprop3 = plot3(eprop3(1,:),eprop3(2,:),eprop3(3,:),'g-');
            ehprop4 = plot3(eprop4(1,:),eprop4(2,:),eprop4(3,:),'g-');
        else
            set(thtraj, 'XData', [get(thtraj, 'XData') true_s(1)]);
            set(thtraj, 'YData', [get(thtraj, 'YData') true_s(2)]);
            set(thtraj, 'ZData', [get(thtraj, 'ZData') true_s(3)]);
            set(thprop1,'XData',tprop1(1,:));
            set(thprop1,'YData',tprop1(2,:));
            set(thprop1,'ZData',tprop1(3,:));
            set(thprop2,'XData',tprop2(1,:));
            set(thprop2,'YData',tprop2(2,:));
            set(thprop2,'ZData',tprop2(3,:));
            set(thprop3,'XData',tprop3(1,:));
            set(thprop3,'YData',tprop3(2,:));
            set(thprop3,'ZData',tprop3(3,:));
            set(thprop4,'XData',tprop4(1,:));
            set(thprop4,'YData',tprop4(2,:));
            set(thprop4,'ZData',tprop4(3,:));
            set(tharm1,'XData',[twp1(1),twp3(1)]);
            set(tharm1,'YData',[twp1(2),twp3(2)]);
            set(tharm1,'ZData',[twp1(3),twp3(3)]);
            set(tharm2,'XData',[twp2(1),twp4(1)]);
            set(tharm2,'YData',[twp2(2),twp4(2)]);
            set(tharm2,'ZData',[twp2(3),twp4(3)]);
            set(thfov1,'XData',[tfov0(1) tfov1(1) tfov2(1)]);
            set(thfov1,'YData',[tfov0(2) tfov1(2) tfov2(2)]);
            set(thfov1,'ZData',[tfov0(3) tfov1(3) tfov2(3)]);
            set(thfov2,'XData',[tfov0(1) tfov2(1) tfov3(1)]);
            set(thfov2,'YData',[tfov0(2) tfov2(2) tfov3(2)]);
            set(thfov2,'ZData',[tfov0(3) tfov2(3) tfov3(3)]);
            set(thfov3,'XData',[tfov0(1) tfov3(1) tfov4(1)]);
            set(thfov3,'YData',[tfov0(2) tfov3(2) tfov4(2)]);
            set(thfov3,'ZData',[tfov0(3) tfov3(3) tfov4(3)]);
            set(thfov4,'XData',[tfov0(1) tfov4(1) tfov1(1)]);
            set(thfov4,'YData',[tfov0(2) tfov4(2) tfov1(2)]);
            set(thfov4,'ZData',[tfov0(3) tfov4(3) tfov1(3)]);
            set(ehtraj, 'XData', [get(ehtraj, 'XData') des_s(1)]);
            set(ehtraj, 'YData', [get(ehtraj, 'YData') des_s(2)]);
            set(ehtraj, 'ZData', [get(ehtraj, 'ZData') des_s(3)]);
            set(ehprop1,'XData',eprop1(1,:));
            set(ehprop1,'YData',eprop1(2,:));
            set(ehprop1,'ZData',eprop1(3,:));
            set(ehprop2,'XData',eprop2(1,:));
            set(ehprop2,'YData',eprop2(2,:));
            set(ehprop2,'ZData',eprop2(3,:));
            set(ehprop3,'XData',eprop3(1,:));
            set(ehprop3,'YData',eprop3(2,:));
            set(ehprop3,'ZData',eprop3(3,:));
            set(ehprop4,'XData',eprop4(1,:));
            set(ehprop4,'YData',eprop4(2,:));
            set(ehprop4,'ZData',eprop4(3,:));
            set(eharm1,'XData',[ewp1(1),ewp3(1)]);
            set(eharm1,'YData',[ewp1(2),ewp3(2)]);
            set(eharm1,'ZData',[ewp1(3),ewp3(3)]);
            set(eharm2,'XData',[ewp2(1),ewp4(1)]);
            set(eharm2,'YData',[ewp2(2),ewp4(2)]);
            set(eharm2,'ZData',[ewp2(3),ewp4(3)]);
        end
        drawnow;
        vis_time = time;
        vis_init = true;
    end
end

