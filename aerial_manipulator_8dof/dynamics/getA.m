function A = getA(param, q)
%A
%    A = A(DX,DY,DZ,GMA_1,GMA_2,L_1,L_2,PHI_X,PHI_Y,PHI_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-May-2021 21:24:31
%% Parameters
% masses
m_B = param.m_B; % multirotor body
m_S1 = param.m_S1; % 1st servo motor (XW540)
m_L1 = param.m_L1; % 1st link
m_S2 = param.m_S2; % 2nd servo motor (XH430)
m_L2 = param.m_L2; % 2nd link
m_S3 = param.m_S3; % 3rd servo motor (XL430-W250-T)
m_E = param.m_E; % end-effector (gripper)

% length
dx = param.dx; % X distance from multirotor c.o.m to 1st servo
dy = param.dy; % Y distance from multirotor c.o.m to 1st servo
dz = param.dz; % Z distance from multirotor c.o.m to 1st servo
l_1 = param.l_1; % 1st link length
l_2 = param.l_2; % 2nd link

% moment of inertia (MOI)
J_Bx = param.J_Bx; J_By = param.J_By; J_Bz = param.J_Bz;
J_S1x = param.J_S1x; J_S1y = param.J_S1y; J_S1z = param.J_S1z;
J_L1x = param.J_L1x; J_L1y = param.J_L1y; J_L1z = param.J_L1z;
J_S2x = param.J_S2x; J_S2y = param.J_S2y; J_S2z = param.J_S2z;
J_L2x = param.J_L2x; J_L2y = param.J_L2y; J_L2z = param.J_L2z;
J_S3x = param.J_S3x; J_S3y = param.J_S3y; J_S3z = param.J_S3z;
J_Ex = param.J_Ex; J_Ey = param.J_Ey; J_Ez = param.J_Ez;

% etc
g = param.g; % gravitational acceleration [m/s^2]

%% State distribution
p_x = q(1);
p_y = q(2);
p_z = q(3);
phi_x = q(4);
phi_y = q(5);
phi_z = q(6);
gma_1 = q(7);
gma_2 = q(8);

%%
t2 = cos(gma_1);
t3 = cos(phi_x);
t4 = cos(phi_y);
t5 = cos(phi_z);
t6 = sin(gma_1);
t7 = sin(phi_x);
t8 = sin(phi_y);
t9 = sin(phi_z);
t10 = gma_1+gma_2;
t11 = cos(t10);
t12 = l_1.*t2;
t13 = sin(t10);
t14 = l_1.*t6;
t15 = t3.*t5;
t16 = t3.*t9;
t17 = t5.*t7;
t18 = t7.*t9;
t19 = l_2.*t11;
t20 = l_2.*t13;
t21 = t8.*t18;
t22 = t8.*t15;
t23 = t8.*t16;
t24 = t8.*t17;
t25 = -t23;
t26 = -t24;
t27 = t12+t19;
t28 = t14+t20;
t31 = t15+t21;
t32 = t18+t22;
t29 = dz+t27;
t30 = dx+t28;
t33 = t16+t26;
t34 = t17+t25;
A = reshape([1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,dy.*t32+t29.*t33,-dy.*t34-t29.*t31,dy.*t3.*t4-t4.*t7.*t29,dy.*t4.*t17-t5.*t8.*t30+t4.*t15.*t29,dy.*t4.*t18-t8.*t9.*t30+t4.*t16.*t29,-t4.*t30-dy.*t7.*t8-t3.*t8.*t29,-dy.*t31+t29.*t34-t4.*t9.*t30,-dy.*t33+t29.*t32+t4.*t5.*t30,0.0,-t28.*t32+t4.*t5.*t27,t28.*t34+t4.*t9.*t27,-t8.*t27-t3.*t4.*t28,-t20.*t32+t4.*t5.*t19,t20.*t34+t4.*t9.*t19,-t8.*t19-t3.*t4.*t20],[3,8]);
