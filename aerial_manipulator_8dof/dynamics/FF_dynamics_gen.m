%% FF_dynamics_gen
clear all; close all; clc;
addpath('../utils/');

%% NOTES
    % Description: actual symbolic matrices M, C, G, ... for the aerial
    % manipulator on free-flight mode.
    % Date: 2021/05/30

%% parameters 
syms m_B m_S1 m_L1 m_S2 m_L2 m_S3 m_E

syms J_Bx J_By J_Bz...
    J_S1x J_S1y J_S1z J_L1x J_L1y J_L1z...
    J_S2x J_S2y J_S2z J_L2x J_L2y J_L2z...
    J_S3x J_S3y J_S3z J_Ex J_Ey J_Ez;
J_B = diag([J_Bx J_By J_Bz]); 
J_S1 = diag([J_S1x J_S1y J_S1z]);
J_L1 = diag([J_L1x J_L1y J_L1z]);
J_S2 = diag([J_S2x J_S2y J_S2z]);
J_L2 = diag([J_L2x J_L2y J_L2z]);
J_S3 = diag([J_S3x J_S3y J_S3z]);
J_E = diag([J_Ex J_Ey J_Ez]);

syms dx dy dz l_1 l_2

syms g

%% states
syms p_x p_y p_z
syms phi_x phi_y phi_z
syms gma_1 gma_2

syms p_xdot p_ydot p_zdot
syms phi_xdot phi_ydot phi_zdot
syms gma_1dot gma_2dot

p_IB = [p_x p_y p_z].';
eta = [phi_x phi_y phi_z].';
gma = [gma_1 gma_2].';

pI_IBdot = [p_xdot p_ydot p_zdot].';
etadot = [phi_xdot phi_ydot phi_zdot].';
gmadot = [gma_1dot gma_2dot].';

q = [p_IB;eta;gma];
qdot = [pI_IBdot;etadot;gmadot];

%% Kinematics
n = length(gma);

R_IB = Rz(phi_z)*Ry(phi_y)*Rx(phi_x);
Q = [1 0 sin(phi_y);...
    0 cos(phi_x) sin(phi_x)*cos(phi_y);...
    0 -sin(phi_x) cos(phi_x)*cos(phi_y)];
T = R_IB*Q;

p_BS1 = [dx dy dz].';
p_BL1 = p_BS1 + [(l_1/2)*sin(gma_1);0;(l_1/2)*cos(gma_1)];
p_BS2 = p_BL1 + [(l_1/2)*sin(gma_1);0;(l_1/2)*cos(gma_1)];
p_BL2 = p_BS2 + [(l_2/2)*sin(gma_1+gma_2);0;(l_2/2)*cos(gma_1+gma_2)];
p_BS3 = p_BL2 + [(l_2/2)*sin(gma_1+gma_2);0;(l_2/2)*cos(gma_1+gma_2)];
p_BE = p_BS3;

R_BS1 = eye(3);
R_BL1 = R_BS1*Ry(gma_1);
R_BS2 = R_BL1*eye(3);
R_BL2 = R_BS2*Ry(gma_2);
R_BS3 = R_BL2*eye(3);
R_BE = R_BS3*eye(3);

p_IS1 = p_IB + R_IB*p_BS1;
p_IL1 = p_IB + R_IB*p_BL1;
p_IS2 = p_IB + R_IB*p_BS2;
p_IL2 = p_IB + R_IB*p_BL2;
p_IS3 = p_IB + R_IB*p_BS3;
p_IE = p_IB + R_IB*p_BE;

omB_BS1 = zeros(3,1);
omB_BL1 = [0;gma_1dot;0]; 
omB_BS2 = omB_BL1;
omB_BL2 = omB_BS2 + Ry(gma_2)*[0;gma_2dot;0];
omB_BS3 = omB_BL2;
omB_BE = omB_BS3;

omI_IB = T*etadot;
omI_IS1 = omI_IB + R_IB*omB_BS1;
omI_IL1 = omI_IB + R_IB*omB_BL1;
omI_IS2 = omI_IB + R_IB*omB_BS2;
omI_IL2 = omI_IB + R_IB*omB_BL2;
omI_IS3 = omI_IB + R_IB*omB_BS3;
omI_IE = omI_IB + R_IB*omB_BE;

R_IS1 = R_IB*R_BS1;
R_IL1 = R_IB*R_BL1;
R_IS2 = R_IB*R_BS2;
R_IL2 = R_IB*R_BL2;
R_IS3 = R_IB*R_BS3;
R_IE = R_IB*R_BE;

omB_IB = R_IB.'*omI_IB;
omS1_IS1 = R_IS1.'*omI_IS1;
omL1_IL1 = R_IL1.'*omI_IL1;
omS2_IS2 = R_IS2.'*omI_IS2;
omL2_IL2 = R_IL2.'*omI_IL2;
omS3_IS3 = R_IS3.'*omI_IS3;
omE_IE = R_IE.'*omI_IE;

%% Jacobians
for i = 1:3
    for j = 1:n+6
        J_tB(i,j) = diff(p_IB(i), q(j));
        J_tS1(i,j) = diff(p_IS1(i), q(j));
        J_tL1(i,j) = diff(p_IL1(i), q(j));
        J_tS2(i,j) = diff(p_IS2(i), q(j));
        J_tL2(i,j) = diff(p_IL2(i), q(j));
        J_tS3(i,j) = diff(p_IS3(i), q(j));
        J_tE(i,j) = diff(p_IE(i), q(j));
        
        J_rB(i,j) = diff(omB_IB(i), qdot(j));
        J_rS1(i,j) = diff(omS1_IS1(i), qdot(j));
        J_rL1(i,j) = diff(omL1_IL1(i), qdot(j));
        J_rS2(i,j) = diff(omS2_IS2(i), qdot(j));
        J_rL2(i,j) = diff(omL2_IL2(i), qdot(j));
        J_rS3(i,j) = diff(omS3_IS3(i), qdot(j));
        J_rE(i,j) = diff(omE_IE(i), qdot(j));
    end
end

%% mass matrix, M
M = m_B*J_tB.'*J_tB + m_S1*J_tS1.'*J_tS1 + m_L1*J_tL1.'*J_tL1...
    + m_S2*J_tS2.'*J_tS2 + m_L2*J_tL2.'*J_tL2...
    + m_S3*J_tS3.'*J_tS3 + m_E*J_tE.'*J_tE...
    + J_rB.' * J_B * J_rB + J_rS1.' * J_S1 * J_rS1 + J_rL1.'*J_L1*J_rL1...
    + J_rS2.' * J_S2 * J_rS2 + J_rL2.'*J_L2*J_rL2...
    + J_rS3.' * J_S3 * J_rS3 + J_rE.'*J_E*J_rE;

% simplification
for i=1:n+6
    for j=1:n+6
        if j>i
            M(j,i) = M(i,j);
        end
        M(i,j) = simplify(M(i,j),'Steps',100);
        fprintf('M(%d,%d) ',i,j);
    end
    fprintf('\n');
end
fprintf('M generation done... \n\n');

%% coriolis array generation, C
C = sym(zeros(n+6,1));
for i = 1:n+6
    for j = 1:n+6
        for k = 1:n+6
            C(i,1) = C(i,1) + (1/2)*(diff(M(i,j),q(k,1))+diff(M(i,k),q(j,1))-diff(M(k,j),q(i,1)))*qdot(k,1)*qdot(j,1);
        end
    end
end

for i = 1:n+6
    C(i,1) = simplify(C(i,1),'Steps',100);
    fprintf('C(%d,1)\n',i);
end
fprintf('C generation done... \n\n');

%% gravitational term generation, G

U = m_B*g*p_z + m_S1*g*p_IS1(3) + m_L1*g*p_IL1(3)...
    + m_S2*g*p_IS2(3) + m_L2*g*p_IL2(3)...
    + m_S3*g*p_IS3(3) + m_E*g*p_IE(3);

for i = 1:n+6
    G(i,1) = diff(U,q(i));
end
for i=1:n+6
    G(i,1) = simplify(G(i,1),'Steps',100);
    fprintf('G(%d,1)\n',i);
end
fprintf('G generation done... \n\n');

%% input mapping Jacobian, J_tau
J_tau = [[R_IB(:,3) zeros(3,3+n)];zeros(3+n,1) blkdiag(Q.',eye(n))].';
fprintf('J_tau generation done... \n\n');

%% external wrench mapping Jacobian, A
A = sym(zeros(3,n+6));
for i = 1:3
    for j = 1:n+6
        A(i,j) = diff(p_IE(i), q(j));
    end
end
fprintf('A generation done... \n\n');

%% Adot
Adot = sym(zeros(3,n+6));
for i = 1:3
    for j = 1:n+6
        for k = 1:length(q)
            Adot(i,j) = Adot(i,j) + diff(A(i,j),q(k,1))*qdot(k,1);
        end
    end
end

fprintf('Adot generation done... \n\n');

%% matlabFunction generation
matlabFunction(M,'File','./getM.m','Optimize',true);
fprintf('matlab function M generated.\n');
matlabFunction(C,'File','./getC.m','Optimize',true);
fprintf('matlab function C generated.\n');
matlabFunction(G,'File','./getG.m','Optimize',true);
fprintf('matlab function G generated.\n');
matlabFunction(J_tau,'File','./getJ_tau.m','Optimize',true);
fprintf('matlab function J_tau generated.\n');
matlabFunction(A,'File','./getA.m','Optimize',true);
fprintf('matlab function A generated.\n');
matlabFunction(Adot,'File','./getAdot.m','Optimize',true);
fprintf('matlab function Adot generated.\n');
