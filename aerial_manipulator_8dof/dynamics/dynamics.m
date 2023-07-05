function dQdt = dynamics(t,Q,tau,del) 
global MODE W_E_in p_IE_in take_off

q = Q(1:length(Q)/2,1);
qdot = Q(length(Q)/2+1:length(Q),1);

parameters;

%% operation
M_ = getM(param,q);
C_ = getC(param,q,qdot);
G_ = getG(param,q);
J_tau_ = getJ_tau(param,q);
A_ = getA(param,q);
Adot_ = getAdot(param,q,qdot);
J_ = [-A_(1:3,4:8);eye(5)];
Jdot_ = [-Adot_(1:3,4:8);zeros(5,5)];

if MODE == 1 % free-flight
    if take_off == false
        if tau(1) - (param.m_B + param.m_S1 + param.m_L1 + param.m_S2 + param.m_S3 + param.m_E) * param.g > 0
            take_off = true;
            fprintf("take-off! \n");
        end
        dQdt = zeros(16,1);
    else
        q2dot = M_\(-C_-G_+J_tau_.'*tau+del);
        dQdt = [qdot;q2dot];
    end
    % force on end-effector
    W_E_in = zeros(3,1);
    
elseif MODE == 2 % wire-pulling mode
    rdot = qdot(4:8);
    r2dot = (J_.'*M_*J_)\(-J_.'*M_*Jdot_*rdot-J_.'*(C_+G_)+J_.'*J_tau_.'*tau + J_.'*del);
    W_E_in = inv(A_*inv(M_)*A_.')...
        *(-Adot_*qdot+A_*inv(M_)*(C_+G_-J_tau_.'*tau-del));
    if W_E_in(1,1) > 15 % 220V
%         MODE = 1;
    end
    dQdt = [J_*rdot;J_*r2dot+Jdot_*rdot];
    
else
    fprintf('Flight mode selection error!');
    return
end
% check end-effector position
R_IB = Rz(q(6))*Ry(q(5))*Rx(q(4));
p_BE = [param.dx;param.dy;param.dz]...
    +[param.l_1*sin(q(7))+param.l_2*sin(q(7)+q(8));0;param.l_1*cos(q(7))+param.l_2*cos(q(7)+q(8))];
p_IE_in = q(1:3,1) + R_IB*p_BE;
end