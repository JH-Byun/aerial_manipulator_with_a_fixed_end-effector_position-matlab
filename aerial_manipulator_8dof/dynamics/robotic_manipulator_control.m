function u_r = robotic_manipulator_control(q_d, q_ddot, q_d2dot, q, qdot, u, del)
global MODE

%% actual parameters
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

%%
M_ = getM(param,q);
C_ = getC(param,q,qdot);
G_ = getG(param,q);
J_tau_ = getJ_tau(param,q);
A_ = getA(param,q);
Adot_ = getAdot(param,q,qdot);

M_chi = M_(1:6,1:6);
M_chigma = M_(1:6,7:8);
M_gma = M_(7:8,7:8);
C_chi = C_(1:6,1);
C_gma = C_(7:8,1);
G_chi = G_(1:6,1);
G_gma = G_(7:8,1);
J_u = J_tau_(1:4,1:6);
Achi = A_(1:3,1:6);
Agma = A_(1:3,7:8);
Achidot = Adot_(1:3,1:6);
Agmadot = Adot_(1:3,7:8);

%%
K_Pr = diag([100 100]);
K_Dr = diag([100 100]);
if MODE == 1
    M = M_gma - M_chigma.'*inv(M_chi)*M_chigma;
    h = (C_gma-M_chigma.'*inv(M_chi)*C_chi)+(G_gma-M_chigma.'*inv(M_chi)*G_chi)...
        +M_chigma.'*inv(M_chi)*(J_u.'*u+del(1:6))-del(7:8);
elseif MODE == 2
    J_ = [-A_(1:3,4:8);eye(5)];
    Jdot_ = [-Adot_(1:3,4:8);zeros(5,5)];
    M_r = J_.'*M_*J_;
    h_r = J_.'*M_*Jdot_*qdot(4:8)+J_.'*(C_+G_);
    del_r = J_.'*del;
    J_tau_r = J_tau_*J_;
    Jac = J_tau_r.';
    
    M = M_r(4:5,4:5)-M_r(1:3,4:5).'*inv(M_r(1:3,1:3))*M_r(1:3,4:5);
    h = h_r(4:5) - M_r(1:3,4:5).'*inv(M_r(1:3,1:3))*h_r(1:3)...
        +M_r(1:3,4:5).'*inv(M_r(1:3,1:3))*(Jac(1:3,1:4)*u+del_r(1:3))-Jac(4:5,1)*u(1)-del_r(4:5);
else
    fprintf('Mode selection error! \n');
end

u_r = M*(q_d2dot(7:8)+K_Dr*(q_ddot(7:8)-qdot(7:8))+K_Pr*(q_d(7:8)-q(7:8)))+h;
u_r(1,1) = constraint(u_r(1,1), -9.5, 9.5);
u_r(2,1) = constraint(u_r(2,1), -9.5, 9.5);
end

