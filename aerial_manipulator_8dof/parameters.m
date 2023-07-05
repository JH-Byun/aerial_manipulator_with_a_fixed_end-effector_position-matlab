% masses
param.m_B = 2.153; % multirotor body
param.m_S1 = 0.1852; % 1st servo motor (XW540)
param.m_L1 = 0.001; % 1st link
param.m_S2 = 0.0821; % 2nd servo motor (XH430)
param.m_L2 = 0.001; % 2nd link
param.m_S3 = 0.05723; % 3rd servo motor (XL430-W250-T)
param.m_E = 0.002; % end-effector (gripper)

% length
param.dx = 0.05; % X distance from multirotor c.o.m to 1st servo
param.dy = 0.005; % Y distance from multirotor c.o.m to 1st servo
param.dz = 0.05; % Z distance from multirotor c.o.m to 1st servo
param.l_1 = 0.25; % 1st link length
param.l_2 = 0.3; % 2nd link

% moment of inertia (MOI)
param.J_Bx = 0.03123; param.J_By = 0.03324; param.J_Bz = 0.03221; % multirotor
param.J_S1x = 83.85e-6; param.J_S1y = 69.391e-6; param.J_S1z = 69.391e-6; % 1st servo motor
param.J_L1x = (1/12)*param.m_L1*param.l_1^2; param.J_L1y = (1/12)*param.m_L1*param.l_1^2; param.J_L1z = 0.0; % 1st link
param.J_S2x = 22.098e-6; param.J_S2y = 19.944e-6; param.J_S2z = 12.842e-6; % 2nd servo motor
param.J_L2x = (1/12)*param.m_L2*param.l_2^2; param.J_L2y = (1/12)*param.m_L2*param.l_2^2; param.J_L2z = 0.0; % 2nd link
param.J_S3x = 15.415e-6; param.J_S3y = 13.192e-6; param.J_S3z = 8.958e-6; % 3rd servo motor
param.J_Ex = 0; param.J_Ey = 0; param.J_Ez = 0; % end-effector

% etc
param.g = 9.81; % gravitational acceleration [m/s^2]