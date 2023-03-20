function [F, M] = controller(s, s_des, params)
%%
% @brief: simple linear controller
%%
m = params.mass;
g = params.grav;
I = params.I;

% current State
x    = s(1);
y    = s(2);
z    = s(3);
dx   = s(4);
dy   = s(5);
dz   = s(6);
quat = s(7: 10);
p    = s(11);
q    = s(12);
r    = s(13);

[phi, theta, psi] = RotToRPY_ZXY(QuatToRot(quat));

R_ab = [cos(theta), 0, -cos(phi) * sin(theta);
    0, 1, sin(phi);
    sin(theta), 0, cos(phi) * cos(theta)];

drpy   = R_ab' * [p; q; r];
dphi   = drpy(1);
dtheta = drpy(2);
dpsi   = drpy(3);

% desire State
x_des    = s_des(1);
y_des    = s_des(2);
z_des    = s_des(3);
dx_des   = s_des(4);
dy_des   = s_des(5);
dz_des   = s_des(6);
quat_des = s_des(7: 10);
[~, ~, psi_des] = RotToRPY_ZXY(QuatToRot(quat_des));
ddx_des    = 0;
ddy_des    = 0;
ddz_des    = 0;
dphi_des   = 0;
dtheta_des = 0;
dpsi_des   = 0;

% PD Parameter
Kd_x = 10;
Kp_x = 10;
Kd_y = 10;
Kp_y = 10;
Kd_z = 15;
Kp_z = 30;

Kp_phi   = 900;
Kd_phi   = 50;
Kp_theta = 900;
Kd_theta = 50;
Kp_psi   = 900;
Kd_psi   = 50;

% position Control
ddx = ddx_des + Kd_x * (dx_des - dx) + Kp_x * (x_des - x);
ddy = ddy_des + Kd_y * (dy_des - dy) + Kp_y * (y_des - y);
ddz = ddz_des + Kd_z * (dz_des - dz) + Kp_z * (z_des - z);
u_1 = m * (g + ddz);

phi_des   = 1 / g * (ddx * sin(psi) - ddy * cos(psi));
theta_des = 1 / g * (ddx * cos(psi) + ddy * sin(psi));

% roll
e_phi = phi_des - phi;
e_phi = atan2(sin(e_phi), cos(e_phi));

% pitch
e_theta = theta_des - theta;
e_theta = atan2(sin(e_theta), cos(e_theta));

% yaw
e_psi = psi_des - psi;
e_psi = atan2(sin(e_psi), cos(e_psi));

% if e_psi <= -pi
%     e_psi = e_psi + 2*pi;
% elseif e_psi >= pi
%     e_psi = e_psi - 2*pi;
% end
%     if(((psi_des - psi) > -pi) && ((psi_des - psi) < pi))
%         e_psi = psi_des - psi;
%     elseif((psi_des - psi) <= -pi)
%         e_psi = psi_des - psi + 2 * pi;
%     elseif((psi_des - psi) >= pi)
%         e_psi = psi_des - psi - 2 * pi;
%     end

% pitch
% if(((theta_des-theta) > -pi) && ((theta_des - theta) < pi))
%     e_theta = theta_des - theta;
% elseif((theta_des-theta) <= -pi)
%     e_theta = theta_des - theta + 2 * pi;
% elseif((theta_des - theta) >= pi)
%     e_theta = theta_des - theta - 2 * pi;
% end

% roll
% if(((phi_des - phi) > -pi) && ((phi_des - phi) < pi))
%     e_phi = phi_des - phi;
% elseif((phi_des - phi) <= -pi)
%     e_phi = phi_des - phi + 2 * pi;
% elseif((phi_des-phi) >= pi)
%     e_phi = phi_des - phi - 2 * pi;
% end

% attitude Control
ddphiC    = Kp_phi * (e_phi) + Kd_phi * (dphi_des - dphi);
ddthetaC  = Kp_theta * (e_theta) + Kd_theta * (dtheta_des - dtheta);
ddvarphiC = Kp_psi * (e_psi) + Kd_psi * (dpsi_des - dpsi);

u_2 = I * [ddphiC; ddthetaC; ddvarphiC] + cross([p; q; r], I * [p; q; r]);
F = u_1;
M = u_2;
end
