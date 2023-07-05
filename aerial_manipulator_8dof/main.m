%% main file: plug-pulling using aerial manipulator
clear; close all; clc;

addpath('./controller/');
addpath('./dynamics/');
addpath('./utils/'); 

%% NOTES
% Name: Jeonghyun Byun
% aerial plug-pulling simulation for ICCAS 2021
% quswjdgus97@snu.ac.kr
% Last updated: 2023/07/05

animation_view = false; % Whether if you want to see the simulation video or not

%% Simulation environment settings
% time setting
DT = 0.01;
t_final = 40;
tspan = 0:DT:t_final;
t_length = length(tspan);

% non-observable variables (real variables)
    % mode
    global MODE W_E_in p_IE_in take_off grabbed
    MODE_REAL = zeros(1,t_length);
 
    % state variables
    q_raw = zeros(8,t_length);
    qdot_raw = zeros(8,t_length);
    W_E_raw = zeros(3,t_length);
    p_IE_raw = zeros(3,t_length);
    
    % actual parameter values
    parameters;
    
    % measurement noise settings
    sigma_q = diag([0.01^2 0.01^2 0.01^2 (pi/90)^2 (pi/90)^2 (pi/90)^2 0.0 0.0]); % position
    sigma_qdot = diag([0.01^2 0.01^2 0.01^2 (pi/90)^2 (pi/90)^2 (pi/90)^2 0.0 0.0]); % velocity
    
    % kinematic quadrotor parameters
    r = 0.2; % length from the quadrotor centroid to each rotor
    c_m = 0.1; % Thrust/Moment
    h_rotor = 0.01; 
    r_rotor = 0.09; % radius of the props
    
% observable variables
    % state variables
        % measured values
        q = zeros(8, t_length);
        qdot = zeros(8, t_length);
        
        % filtered values
        qhat = zeros(8,t_length);
        qhatdot = zeros(8,t_length);
        
        % desired values
        q_d = zeros(8,t_length);
        q_ddot = zeros(8,t_length);
        q_d2dot = zeros(8,t_length);   
        
        % target values
        q_t = zeros(8,1);
        
        % F_phi & G_phi
        F_phi = zeros(3,2);
        G_phi = zeros(3,3,2);
        
    % inner-loop variables
    qxydot = zeros(4,1);
    pxydot = zeros(4,1);
    qzdot = zeros(2,1);
    pzdot = zeros(2,1);
    qphidot = zeros(6,1);
    pphidot = zeros(6,1);
            
    % control input trajectory
    u = zeros(4,t_length);
    tau = zeros(6,t_length);
    T = zeros(1,2);
    tau_B0 = zeros(3,2);
    tau_B = zeros(3,2);
    F = zeros(4,t_length);
    
    % controller mode array
    MODE_CON = ones(1,t_length);
    
    % controller performance values
    spent_time = zeros(1,t_length);

%% Initialization & Parameter settings
% initialization
    % initialize mode
    MODE = 1; % 1: free-flight mode, 2: plug-pulling mode
    MODE_REAL(1) = MODE;
    take_off = false;
    grabbed = false;

    % initialize states
        % real values
        q_raw(:,1) = [0;0;0.25;0;0;0;pi/4;pi/4];
        qdot_raw(:,1) = [0;0;0;0;0;0;0;0];

    % initialize inner variables
    qxy = zeros(4,1); pxy = zeros(4,1);
    qz = zeros(2,1); pz = zeros(2,1);
    qphi(:,1) = zeros(6,1); pphi(:,1) = zeros(6,1);
    qphi(:,2) = zeros(6,1); pphi(:,2) = zeros(6,1);

% parameter settings
    % hovering test 
    hovering_test = false;
    
    % mixer on and off
    mixer_on = false;
    air_mode = false;

    % nominal inertial & geometric params
    m_B = 2.5;
    J_B = diag([0.02 0.02 0.02]);
    l_1 = 0.25; l_2 = 0.25;
    g = 9.8;
    p_BS1 = [0.05;0.05;0.05];
    phi_ymax = pi/12;
    e_3 = [0;0;1];
    
    % end-effector's target position
    p_IEd = [3.0;0.0;2.0];
    
    % settings for minimun-jerk trajectory generation planning
    c = zeros(8,8);
    T_arr = [15.0 10.0 15.0]; % desired time period for each mode
    s_num = 0; % number of mode-switching time
    t_s = zeros(1,3); % mode-switching time storage
    
    % XY_CON control gains
        % nominal
        Kxy_P = diag([0.5 0.5]); 
        Kxy_D = diag([0.6 0.5]); 
        % XY_DOB
        XY_DOB = true;
        axy_10 = 1.0; axy_11 = 2.0;
        axy_20 = 1.0; axy_21 = 2.0;
        eps_xy = 0.1;
    % Z_CON control gains
        % nominal
        Kz_P = 20.0; Kz_D = 15.0;
        % Z_DOB
        Z_DOB = true;
        az_10 = 1; az_11 = 2;
        eps_z = 0.1;
    % RPY_CON control gains
        % nominal
        K_R = diag([10 5 10]);
        Kphi_P(:,:,1) = diag([150 150 150]); Kphi_D(:,:,1) = diag([100 100 100]); % for mode 1
        Kphi_P(:,:,2) = diag([80 80 80]); Kphi_D(:,:,2) = diag([50 50 50]); % for mode 2
        % RPY_DOB
        RPY_DOB = true;
        aphi_10 = 1; aphi_11 = 2;
        aphi_20 = 1; aphi_21 = 2;
        aphi_30 = 1; aphi_31 = 2;
        eps_phi = 0.1;
        Ldaphi = diag([sqrt(J_B(1,1)) sqrt(J_B(2,2)) sqrt(J_B(3,3))]);
    
%% Simulation (closed-loop)
clc
for i = 1:t_length-1
    % measurement 
    q(:,i) = mvnrnd(q_raw(:,i),sigma_q).';
    qdot(:,i) = mvnrnd(qdot_raw(:,i),sigma_qdot).';
    
    % filtering
    qhat(:,i) = q_raw(:,i);
    qhatdot(:,i) = qdot_raw(:,i);
    
    % grabbing determination
        % actual p_IE, p_IEdot value
        p_BS1_raw = [param.dx;param.dy;param.dz];
        p_BE_raw =  p_BS1_raw...
            + [param.l_1*sin(q_raw(7,i))+param.l_2*sin(q_raw(7,i)+q_raw(8,i));0;param.l_1*cos(q_raw(7,i))+param.l_2*cos(q_raw(7,i)+q_raw(8,i))];
        R_IB_raw = Rz(q_raw(6,i))*Ry(q_raw(5,i))*Rx(q_raw(4,i));
        Q_raw = getQ(q_raw(4:6,i));
        J_tE_raw = [param.l_1*cos(q_raw(7,i))+param.l_2*cos(q_raw(7,i)+q_raw(8,i)) param.l_2*cos(q_raw(7,i)+q_raw(8,i));
            0 0;
            -param.l_1*sin(q_raw(7,i))-param.l_2*sin(q_raw(7,i)+q_raw(8,i)) -param.l_2*sin(q_raw(7,i)+q_raw(8,i))];
        p_IEdot_raw = qdot_raw(1:3,i) + hat(Q_raw*qdot_raw(4:6,i))*R_IB_raw*p_BE_raw + R_IB_raw*J_tE_raw*qdot_raw(7:8,i);
        % grabbing condition
        if i~=1 && MODE_REAL(1,i-1) == 1 && norm(p_IE_raw(:,i)-p_IEd) < 1e-1 && norm(p_IEdot_raw) < 5e-2 && grabbed == false
            MODE = 2;
            grabbed = true;
        end
    
    % supervisor: control mode selection
    MODE_CON(1,i) = MODE_REAL(1,i);
    if i == 1 || (i~=1 && MODE_REAL(:,i) ~= MODE_REAL(1,i-1))
        fprintf('Control mode changed! \n');
        s_num = s_num + 1; % mode-switching occurs
        t_s(s_num) = tspan(i); % stores mode-switching time
        t_f = t_s(s_num) + T_arr(s_num);
    end
     
    % target point generation
    p_BE = p_BS1 + [l_1*sin(qhat(7,i));0;l_1*cos(qhat(7,i))] + [l_1*sin(qhat(7,i)+qhat(8,i));0;l_1*cos(qhat(7,i)+qhat(8,i))];
    if hovering_test == true
        q_t = [0;0;0.75;0;0;0;pi/4;pi/4];
    else
        if MODE_CON(1,i) == 1 % target position for free-flight mode 
            q_t(4:6) = [0;0;0];
            q_t(1:3) = p_IEd - Rz(q_t(6))*Ry(q_t(5))*Rx(q_t(4))*p_BE;
            q_t(7:8) = [pi/4;pi/4];
        elseif MODE_CON(:,i) == 2 % target position perching mode
            q_t(4:6) = [0;-phi_ymax;0];
            q_t(7:8) = [pi/4;pi/4];
        end
    end
    
    % minimum-jerk trajectory planning
        % coefficient matrix (c) generation 
        if i == 1 || (i ~= 1 && MODE_CON(1,i) ~= MODE_CON(1,i-1))
            q_0 = qhat(:,i);
            q_0dot = qhatdot(:,i);
            for j = 1:8
                H_part = [36*(t_f-t_s(s_num)) 72*(t_f^2-t_s(s_num)^2) 120*(t_f^3-t_s(s_num)^3) 180*(t_f^4 - t_s(s_num)^4) 252*(t_f^5-t_s(s_num)^5);...
                    72*(t_f^2-t_s(s_num)^2) 192*(t_f^3-t_s(s_num)^3) 360*(t_f^4 - t_s(s_num)^4) 576*(t_f^5-t_s(s_num)^5) 840*(t_f^6-t_s(s_num)^6);...
                    120*(t_f^3-t_s(s_num)^3) 360*(t_f^4-t_s(s_num)^4) 720*(t_f^5-t_s(s_num)^5) 1200*(t_f^6-t_s(s_num)^6) 1800*(t_f^7-t_s(s_num)^7);...
                    180*(t_f^4-t_s(s_num)^4) 576*(t_f^5-t_s(s_num)^5) 1200*(t_f^6-t_s(s_num)^6) (14000/7)*(t_f^7-t_s(s_num)^7) 3150*(t_f^8-t_s(s_num)^8);...
                    252*(t_f^5-t_s(s_num)^5) 840*(t_f^6-t_s(s_num)^6) 1800*(t_f^7-t_s(s_num)^7) 3150*(t_f^8-t_s(s_num)^8) 4900*(t_f^9-t_s(s_num)^9)];
                H = [zeros(3,3) zeros(3,5);...
                    zeros(5,3) H_part];
                f = zeros(8,1);
                Aeq = [1 t_s(s_num) t_s(s_num)^2 t_s(s_num)^3 t_s(s_num)^4 t_s(s_num)^5 t_s(s_num)^6 t_s(s_num)^7;...
                    0 1 2*t_s(s_num) 3*t_s(s_num)^2 4*t_s(s_num)^3 5*t_s(s_num)^4 6*t_s(s_num)^5 7*t_s(s_num)^6;...
                    1 t_f t_f^2 t_f^3 t_f^4 t_f^5 t_f^6 t_f^7;...
                    0 1 2*t_f 3*t_f^2 4*t_f^3 5*t_f^4 6*t_f^5 7*t_f^6];
                beq = [q_0(j);q_0dot(j);q_t(j);0.0];
                [c(:,j),fval,exitflag,output,lambda] = quadprog(H,f,[],[],Aeq,beq);
            end
        end

        % trajectory generation
        for j = 1:6
            if tspan(i) < t_f
                q_d(j,i) = [1 tspan(i) tspan(i)^2 tspan(i)^3 tspan(i)^4 tspan(i)^5 tspan(i)^6 tspan(i)^7]*c(:,j);
                q_ddot(j,i) = [0 1 2*tspan(i) 3*tspan(i)^2 4*tspan(i)^3 5*tspan(i)^4 6*tspan(i)^5 7*tspan(i)^6]*c(:,j);
                q_d2dot(j,i) = [0 0 2 6*tspan(i) 12*tspan(i)^2 20*tspan(i)^3 30*tspan(i)^4 42*tspan(i)^5]*c(:,j);
            else
                q_d(j,i) = q_t(j);
                q_ddot(j,i) = 0.0;
                q_d2dot(j,i) = 0.0;
            end
        end
        q_d(7:8,i) = [q_t(7)-qhat(5,i);q_t(8)];
        q_ddot(7:8,i) = zeros(2,1);
        q_d2dot(7:8,i) = zeros(2,1);    
    
    % controller
        tic;
        % common values
        Q = getQ(qhat(4:6,i));
        Qdot = getQdot(qhat(4:6,i), qhatdot(4:6,i));
        R_IB = Rz(qhat(6,i))*Ry(qhat(5,i))*Rx(qhat(4,i));
        % xy-plane control
            % errors
            e = q_d(:,i) - qhat(:,i);
            edot = q_ddot(:,i) - qhatdot(:,i);
            % outer-loop control law
            PSI = [cos(qhat(6,i)) sin(qhat(6,i));...
                sin(qhat(6,i)) -cos(qhat(6,i))];
            G_xy = g*cos(qhat(4,i))*cos(qhat(5,i))*PSI;
            PHI_d = G_xy\(Kxy_P*e(1:2) + Kxy_D*edot(1:2));
            % inner-loop control law
            if XY_DOB == true
                PHI_0 = PHI_d;
                % control parameters
                Axy = blkdiag([0 1;...
                    -axy_10/eps_xy^2 -axy_11/eps_xy],[0 1;...
                    -axy_20/eps_xy^2 -axy_21/eps_xy]); 
                Bxy = [0;axy_10/eps_xy^2;0;axy_20/eps_xy^2];
                % control input generation
                qxydot(1:2) = Axy(1:2,1:2)*qxy(1:2) + Bxy(1:2)*qhat(1,i);
                qxydot(3:4) = Axy(3:4,3:4)*qxy(3:4) + Bxy(3:4)*qhat(2,i);  

                uxy = [pxy(1)-qxydot(2);pxy(3)-qxydot(4)];
                PHI_1 = G_xy*PHI_0;
                uxy(1) = constraint(uxy(1), -5, 5);
                uxy(2) = constraint(uxy(2), -5, 5);
                PHI_2 = PHI_1 + uxy;
                PHI_d = PHI_0 + inv(G_xy)*uxy;

                pxydot(1:2) = Axy(1:2,1:2)*pxy(1:2,1)+Bxy(1:2,1)*PHI_2(1,1);
                pxydot(3:4) = Axy(3:4,3:4)*pxy(3:4,1)+Bxy(3:4,1)*PHI_2(2,1);

                qxy = qxy + qxydot * DT; % qxy update
                pxy = pxy + pxydot * DT; % pxy update
            end        
            if MODE_CON(i) == 1
                q_d(4,i) = asin(constraint(PHI_d(2), -1/sqrt(2), 1/sqrt(2)));
                q_d(5,i) = asin(constraint(PHI_d(1)/cos(q_d(4,i)), -1/sqrt(2), 1/sqrt(2)));
            end  
        % altitude control
            % errors
            e(3) = q_d(3,i) - qhat(3,i);
            edot(3) = q_ddot(3,i) - qhatdot(3,i);
            % outer-loop control law
            F_z = -g;
            G_z = e_3.'*R_IB*e_3/m_B;
            T(1) = (-F_z + Kz_P*e(3) + Kz_D*edot(3))/G_z;
            % inner-loop control law
            if Z_DOB == true 
                T_0 = T(1);
                % control parameters
                Az = [0 1;...
                    -az_10/eps_z^2 -az_11/eps_z]; 
                Bz = [0;az_10/eps_z^2];
                % control input generation
                qzdot = Az*qz + Bz*qhat(3,i);
                uz = pz(1) - sqrt(m_B)*(qzdot(2)-F_z);
                T_1 = sqrt(m_B)*G_z*T_0;
                uz = constraint(uz, -100, 100);
                T_2 = T_1 + uz;
                T(1) = T_0 + uz/(sqrt(m_B)*G_z);
                pzdot = Az*pz + Bz*T_2;
                qz = qz + qzdot*DT;
                pz = pz + pzdot*DT;
                if MODE_REAL(:,i) == 1
                    T_h = T(1)*cos(qhat(4,i))*cos(qhat(5,i));
                end
            end
        % attitude control
            % errors
            e(4:6) = q_d(4:6,i) - qhat(4:6,i);
            edot(4:6) = q_ddot(4:6,i) - qhatdot(4:6,i);
            % outer-loop control law
                % desired angular velocity
                R_IBd = Rz(q_d(6,i))*Ry(q_d(5,i))*Rx(q_d(4,i));
                A_IB = R_IB.'*R_IBd;
                dR_IB = (1/2)*(A_IB.'-A_IB);
                e_R = invhat(dR_IB);
                omB_IB_d = -inv(trace(A_IB)*eye(3) - A_IB)*K_R*e_R;
                phi_ddot = inv(Q)*omB_IB_d;
                q_ddot(4,i) = phi_ddot(1);
                q_ddot(5,i) = phi_ddot(2);
                % F_phi & G_phi
                T(2) = T_h/cos(qhat(4,i))*cos(qhat(5,i));
                J_t = J_B + m_B*hat(R_IB*p_BE).'*hat(R_IB*p_BE);
                F_phi(:,1) = -inv(Q)*Qdot*qhatdot(4:6,i)-inv(Q)*inv(J_B)*hat(Q*qhatdot(4:6,i))*J_B*Q*qhatdot(4:6,i);
                F_phi(:,2) = -(J_t*Q)\((m_B*hat(R_IB*p_BE).'*(hat(hat(Q*qhatdot(4:6,i))*R_IB*p_BE)*Q+hat(R_IB*p_BE)*Qdot)...
                    +J_B*Qdot*hat(Q*qhatdot(4:6,i))*J_B*Q)*qhatdot(4:6,i)+m_B*g*hat(R_IB*p_BE).'*e_3-T(2)*Q.'*hat(R_IB*p_BE)*R_IB*e_3);
                G_phi(:,:,1) = inv(J_B*Q);
                G_phi(:,:,2) = inv(J_t*Q);
            % inner-loop control law
            for j = 1:2
                % tau_B
                tau_B(:,j) = G_phi(:,:,j)\(-F_phi(:,j) + Kphi_P(:,:,j)*e(4:6) + Kphi_D(:,:,j)*edot(4:6));
                if RPY_DOB == true
                    tau_B0(:,j) = tau_B(:,j);
                    Aphi = blkdiag([0 1;...
                        -aphi_10/eps_phi^2 -aphi_11/eps_phi],[0 1;...
                        -aphi_20/eps_phi^2 -aphi_21/eps_phi],[0 1;...
                        -aphi_30/eps_phi^2 -aphi_31/eps_phi]);
                    Bphi = [0;aphi_10/eps_phi^2;0;aphi_20/eps_phi^2;0;aphi_30/eps_phi^2];
                    % control input generation
                    qphidot(1:2,1) = Aphi(1:2,1:2)*qphi(1:2,j)+Bphi(1:2,1)*qhat(4,i);
                    qphidot(3:4,1) = Aphi(3:4,3:4)*qphi(3:4,j)+Bphi(3:4,1)*qhat(5,i);
                    qphidot(5:6,1) = Aphi(5:6,5:6)*qphi(5:6,j)+Bphi(5:6,1)*qhat(6,i);

                    uphi = [pphi(1,j)-Ldaphi(1,1)*(qphidot(2,1)-F_phi(1,j));...
                        pphi(3,j)-Ldaphi(2,2)*(qphidot(4,1)-F_phi(2,j));...
                        pphi(5,j)-Ldaphi(3,3)*(qphidot(6,1)-F_phi(3,j))];
                    tau_B1 = Ldaphi*G_phi(:,:,j)*tau_B0(:,j);
                    uphi(1,1) = constraint(uphi(1,1),-20,20);
                    uphi(2,1) = constraint(uphi(2,1),-20,20);
                    uphi(3,1) = constraint(uphi(3,1),-20,20);
                    tau_B2 = tau_B1+uphi;
                    tau_B(:,j) = tau_B0(:,j)+inv(Ldaphi*G_phi(:,:,j))*uphi;

                    pphidot(1:2,1) = Aphi(1:2,1:2)*pphi(1:2,j)+Bphi(1:2,1)*tau_B2(1,1);
                    pphidot(3:4,1) = Aphi(3:4,3:4)*pphi(3:4,j)+Bphi(3:4,1)*tau_B2(2,1);
                    pphidot(5:6,1) = Aphi(5:6,5:6)*pphi(5:6,j)+Bphi(5:6,1)*tau_B2(3,1);

                    pphi(:,j) = pphi(:,j) + pphidot * DT; % pphi update
                    qphi(:,j) = qphi(:,j) + qphidot * DT; % qphi update
                end
            end
            
        % mixer
        T(MODE_CON(i)) = constraint(T(MODE_CON(i)), 0, 80);
        tau_B(1,MODE_CON(i)) = constraint(tau_B(1,MODE_CON(i)), -4.0, 4.0);
        tau_B(2,MODE_CON(i)) = constraint(tau_B(2,MODE_CON(i)), -4.0, 4.0);
        tau_B(3,MODE_CON(i)) = constraint(tau_B(3,MODE_CON(i)), -4.0, 4.0);
        MIXER = [1 1 1 1;...
            r/sqrt(2) r/sqrt(2) -r/sqrt(2) -r/sqrt(2);...
            -r/sqrt(2) r/sqrt(2) r/sqrt(2) -r/sqrt(2);...
            -c_m c_m -c_m c_m];
        u(:,i) = [T(MODE_CON(i));tau_B(:,MODE_CON(i))];
        F(:,i) = MIXER\u(:,i); 
        
    spent_time(1,i) = toc; % measure control performance
        
    % robotic manipulator control (accomplished by dynamixel position control)
        % random disturbance
        mu = zeros(8,1);
        Sigma = diag([0.1^2 0.1^2 0.1^2 0.1^2 0.1^2 0.1^2 0.0 0.0]);
        del = mvnrnd(mu, Sigma).';
        % servo torque generation
        u_r = robotic_manipulator_control(q_d(:,i), q_ddot(:,i), q_d2dot(:,i), q(:,i), qdot(:,i), u(:,i), del);

    % dynamics
    opts = odeset('RelTol',1e-12,'AbsTol',1e-12);
    Q0 = [q_raw(:,i);qdot_raw(:,i)];
    tau = [u(:,i);u_r];
    [t,Q] = ode45(@(t,Q) dynamics(t,Q,tau,del),[0 DT],Q0,opts);
    Q = Q.';
    W_E_raw(:,i+1) = W_E_in;
    p_IE_raw(:,i+1) = p_IE_in;
    q_raw(:,i+1) = Q(1:8,length(t));
    qdot_raw(:,i+1) = Q(9:16,length(t));
    MODE_REAL(:,i+1) = MODE;
    
    % loop alert
    if rem(i,100) == 0
        if MODE_REAL(i) == 1
            fprintf('%d-th loop (FF) \n', i);
        elseif MODE_REAL(i) == 2
            fprintf('%d-th loop (PM) \n', i);
        else    
            fprintf('Mode selection error \n');
        end
    end
end

%% Result plot settings
minus_span = 1:t_length-1; % time span for result outputs
default_font_size = 12.5; % default font size
default_line_width = 2; % default line width
fig_loc_size = [420, 420, 500, 500]; % location & size of the figure 
time_sec = [0 t_final]; % necessary time section

% patch settings
colour_1 = 'blue'; colour_2 = 'green'; 
transparency_1 = 0.05; transparency_2 = 0.05;
v_1 = [t_s(1) -1000; t_s(1) 1000; t_s(2) 1000; t_s(2) -1000]; f_1 = [1 2 3 4];
v_2 = [t_s(2) -1000; t_s(2) 1000; t_final 1000; t_final -1000]; f_2 = [1 2 3 4];

%% Position plot
fig = figure('DefaultAxesFontSize',default_font_size);
set(fig, 'OuterPosition', fig_loc_size)
subplot(3,1,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), q_d(1,minus_span),'--' ,'LineWidth',default_line_width);
hold on
plot(tspan(minus_span), q_raw(1,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('p_x [m]');
ylim([0 3.0]); 
xlim(time_sec);
lgd = legend({'', '', 'desired', 'actual'}, 'Location', 'best');
lgd.NumColumns = 1;
subplot(3,1,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), q_d(2,minus_span),'--' , 'LineWidth',default_line_width);
hold on
plot(tspan(minus_span), q_raw(2,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('p_y [m]');
ylim([-1.5 1.5]);
xlim(time_sec);
subplot(3,1,3)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), q_d(3,minus_span),'--' , 'LineWidth',default_line_width);
hold on
plot(tspan(minus_span), q_raw(3,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('p_z [m]');
ylim([0 3.0]);
xlim(time_sec);

%% Euler angle plot
fig = figure('DefaultAxesFontSize',default_font_size);
set(fig, 'OuterPosition', fig_loc_size)
subplot(3,1,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), (180/pi)*q_d(4,minus_span),'--','LineWidth',default_line_width);
hold on
plot(tspan(minus_span), (180/pi)*q_raw(4,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('$\phi_x$ [deg]','Interpreter','Latex');
ylim([-30.0 30.0]);
xlim(time_sec);
legend({'','','desired', 'actual'},'Location','best');
subplot(3,1,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), (180/pi)*q_d(5,minus_span),'--', 'LineWidth',default_line_width);
hold on
plot(tspan(minus_span), (180/pi)*q_raw(5,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('$\phi_y$ [deg]','Interpreter','Latex');
ylim([-30.0 30.0]);
xlim(time_sec);
subplot(3,1,3)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), (180/pi)*q_d(6,minus_span),'--', 'LineWidth',default_line_width);
hold on
plot(tspan(minus_span), (180/pi)*q_raw(6,minus_span), 'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('$\phi_{z}$ [deg]','Interpreter','Latex');
ylim([-30.0 30.0]);
xlim(time_sec);

%% Servo motor angle plot
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
subplot(2,1,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),(180/pi)*q_d(7,minus_span),'LineWidth',default_line_width);
hold on
plot(tspan(minus_span),(180/pi)*q_raw(7,minus_span),'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('$\gamma_{1}$ [deg]','Interpreter','Latex');
ylim([35 65]);
legend({'','','desired', 'actual'},'Location','best');
subplot(2,1,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),(180/pi)*q_d(8,minus_span),'LineWidth',default_line_width);
hold on
plot(tspan(minus_span),(180/pi)*q_raw(8,minus_span),'LineWidth',default_line_width);
grid on
xlabel('time [sec]');
ylabel('$\gamma_{2}$ [deg]','Interpreter','Latex');
ylim([35 65]);

%% Euler rate plot
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
subplot(1,2,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), (180/pi)*q_ddot(4,minus_span), 'LineWidth', 2);
hold on
plot(tspan(minus_span), (180/pi)*qdot_raw(4,minus_span), 'LineWidth', 2);
grid on
xlabel('time [sec]');
ylabel('$\dot{\phi_x}$ [deg/sec]','Interpreter','Latex');
legend({'','','desired', 'actual'},'Location','best');
ylim([-40 40]);
subplot(1,2,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span), (180/pi)*q_ddot(5,minus_span), 'LineWidth', 2);
hold on
plot(tspan(minus_span), (180/pi)*qdot_raw(5,minus_span), 'LineWidth', 2);
grid on
xlabel('time [sec]');
ylabel('$\dot{\phi_y}$ [deg/sec]','Interpreter','Latex');
ylim([-40 40]);
sgtitle('Roll & pitch rate plot');

%% plot of the force exerted on the end-effector 
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
subplot(3,1,1);
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),W_E_raw(1,minus_span),'LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$F_{Ex}$ [N]','Interpreter','Latex');
ylim([-5 15]);
subplot(3,1,2);
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),W_E_raw(2,minus_span),'LineWidth',2)
grid on
xlabel('time [sec]');
ylabel('$F_{Ey}$ [N]','Interpreter','Latex');
ylim([-5 15]);
subplot(3,1,3);
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),W_E_raw(3,minus_span),'LineWidth',2)
grid on
xlabel('time [sec]');
ylabel('$F_{Ez}$ [N]','Interpreter','Latex');
ylim([-10 10]);
sgtitle('Force on the end-effector');
    
%% control performance performance plot
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),spent_time(1,minus_span),'-','LineWidth',2)
grid on
xlabel('time [sec]');
ylabel('loop time [sec]');
ylim([0 0.05]);
title('Control loop computation time for each loop');
    
%% plot of the control input u = [T;tau_B]
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
subplot(2,2,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),u(1,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 80*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), 0.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$T$ [N]','Interpreter','Latex');
ylim([-10 90]);
subplot(2,2,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),u(2,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 4.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), -4.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$\tau_{Bx}$ [Nm]','Interpreter','Latex');
ylim([-5.0 5.0]);
subplot(2,2,3)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),u(3,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 4.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), -4.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$\tau_{By}$ [Nm]','Interpreter','Latex');
ylim([-5.0 5.0]);
subplot(2,2,4)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),u(4,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 4.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), -4.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$\tau_{Bz}$ [Nm]','Interpreter','Latex');
ylim([-5.0 5.0]);
sgtitle('Control input history');

%% plot of the motor thrust F
minus_span = 1:t_length-1;
figure('DefaultAxesFontSize',default_font_size)
subplot(2,2,1)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),F(1,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 20*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), 0.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$F_1$ [N]','Interpreter','Latex');
ylim([-5 25]);
subplot(2,2,2)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),F(2,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 20.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), 0.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$F_2$ [N]','Interpreter','Latex');
ylim([-5 25]);
subplot(2,2,3)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),F(3,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 20.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), 0.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$F_3$ [N]','Interpreter','Latex');
ylim([-5 25]);
subplot(2,2,4)
patch('Faces', f_1, 'Vertices', v_1, 'FaceColor', colour_1, 'FaceAlpha', transparency_1, 'EdgeColor', 'none')
hold on
patch('Faces', f_2, 'Vertices', v_2, 'FaceColor', colour_2, 'FaceAlpha', transparency_2, 'EdgeColor', 'none')
hold on
plot(tspan(minus_span),F(4,minus_span),'LineWidth',2)
hold on
plot(tspan(minus_span), 20.0*ones(1,t_length-1),'--','LineWidth',2);
hold on
plot(tspan(minus_span), 0.0*ones(1,t_length-1),'--','LineWidth',2);
grid on
xlabel('time [sec]');
ylabel('$F_4$ [N]','Interpreter','Latex');
ylim([-5 25]);
sgtitle('Motor thrust history');

%% Animation visualization
if animation_view
    % task space scope
    x_min = -0.5; x_max = 3.5;
    y_min = -2.0; y_max = 2.0;
    z_min = 0.0; z_max = 4.0;
    
    fig = figure('DefaultAxesFontSize',default_font_size);
    set(fig, 'OuterPosition', fig_loc_size)
    
    % animation visualization loop
    for i = 1:length(tspan)-1
        if rem(i, 10) == 1
            % wipe the slate clean so we are plotting with a black figure
            clf        
            % plane
            vert_p = [[x_min y_min z_min];[x_min y_max z_min];[x_max y_max z_min];[x_max y_min z_min];...
                [x_min y_min z_min-0.1];[x_min y_max z_min-0.1];[x_max y_max z_min-0.1];[x_max y_min z_min-0.1]];
            fac_p = [1 2 3 4;1 2 6 5;2 3 7 6;3 7 8 4;1 2 8 4;5 6 7 8];
            C_p = rand(8,3);
            patch('Vertices',vert_p,'Faces',fac_p,'FaceVertexCData',C_p,'FaceColor','white')
            view(3)
            % draw a socket frame
                % frame
                hold on
                plot3([p_IE_raw(1,round(round(t_s(2)*100))+1) p_IE_raw(1,round(round(t_s(s_num)*100))+1)],...
                    [p_IE_raw(2,round(round(t_s(s_num)*100))+1) p_IE_raw(2,round(round(t_s(s_num)*100))+1)],...
                    [p_IE_raw(3,round(round(t_s(s_num)*100))+1)-0.1 0],'k-','LineWidth',3)
                % socket
                hold on
                vert_s = [p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0 -0.05 -0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0 -0.05 0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0 0.05 0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0 0.05 -0.1];...
                    p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0.01 -0.05 -0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0.01 -0.05 0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0.01 0.05 0.1];p_IE_raw(:,round(t_s(s_num)*100)+1).'+[0.01 0.05 -0.1]];
                fac_s = [1 2 3 4;1 2 6 5;2 6 7 3;3 7 8 4;1 4 5 8;5 6 7 8];
                C_s = rand(8,3);
                patch('Vertices',vert_s,'Faces',fac_s,'FaceVertexCData',C_s,'FaceColor',[0.3010 0.7450 0.9330])
                view(3) 
            % aerial manipulator
                % quadrotor
                R_IB_raw = Rz(q_raw(6,i))*Ry(q_raw(5,i))*Rz(q_raw(4,i));
                vert_q = [(q_raw(1:3,i)+R_IB_raw*[-0.05;-0.05;-0.02]).';(q_raw(1:3,i)+R_IB_raw*[-0.05;0.05;-0.02]).';...
                    (q_raw(1:3,i)+R_IB_raw*[0.05;0.05;-0.02]).';(q_raw(1:3,i)+R_IB_raw*[0.05;-0.05;-0.02]).';...
                    (q_raw(1:3,i)+R_IB_raw*[-0.05;-0.05;0.05]).';(q_raw(1:3,i)+R_IB_raw*[-0.05;0.05;0.05]).';...
                    (q_raw(1:3,i)+R_IB_raw*[0.05;0.05;0.05]).';(q_raw(1:3,i)+R_IB_raw*[0.05;-0.05;0.05]).'];
                fac_q = [1 2 3 4;1 2 6 5;2 6 7 3;3 7 8 4;1 4 5 8;5 6 7 8];
                C_q = rand(8,3);
                patch('Vertices',vert_q,'Faces',fac_q,'FaceVertexCData',C_q,'FaceColor','black')
                view(3)
                p_IR1_raw = q_raw(1:3,i) + R_IB_raw*[r/sqrt(2);r/sqrt(2);0];
                p_IR2_raw = q_raw(1:3,i) + R_IB_raw*[-r/sqrt(2);r/sqrt(2);0];
                p_IR3_raw = q_raw(1:3,i) + R_IB_raw*[-r/sqrt(2);-r/sqrt(2);0];
                p_IR4_raw = q_raw(1:3,i) + R_IB_raw*[r/sqrt(2);-r/sqrt(2);0];
                p_IR1h_raw = q_raw(1:3,i) + R_IB_raw*([r/sqrt(2);r/sqrt(2);0]+[0;0;h_rotor]);
                p_IR2h_raw = q_raw(1:3,i) + R_IB_raw*([-r/sqrt(2);r/sqrt(2);0]+[0;0;h_rotor]);
                p_IR3h_raw = q_raw(1:3,i) + R_IB_raw*([-r/sqrt(2);-r/sqrt(2);0]+[0;0;h_rotor]);
                p_IR4h_raw = q_raw(1:3,i) + R_IB_raw*([r/sqrt(2);-r/sqrt(2);0]+[0;0;h_rotor]);
                N_circle = 100;
                p_R1_raw = zeros(3,N_circle+1);
                p_R2_raw = zeros(3,N_circle+1);
                p_R3_raw = zeros(3,N_circle+1);
                p_R4_raw = zeros(3,N_circle+1);
                for j = 1:N_circle+1
                    p_R1_raw(:,j) = q_raw(1:3,i) + R_IB_raw*([r/sqrt(2);r/sqrt(2);0]+[0;0;h_rotor]+r_rotor*[cos(2*pi*j/N_circle);sin(2*pi*j/N_circle);0]);
                    p_R2_raw(:,j) = q_raw(1:3,i) + R_IB_raw*([-r/sqrt(2);r/sqrt(2);0]+[0;0;h_rotor]+r_rotor*[cos(2*pi*j/N_circle);sin(2*pi*j/N_circle);0]);
                    p_R3_raw(:,j) = q_raw(1:3,i) + R_IB_raw*([-r/sqrt(2);-r/sqrt(2);0]++[0;0;h_rotor]+r_rotor*[cos(2*pi*j/N_circle);sin(2*pi*j/N_circle);0]);
                    p_R4_raw(:,j) = q_raw(1:3,i) + R_IB_raw*([r/sqrt(2);-r/sqrt(2);0]+[0;0;h_rotor]+r_rotor*[cos(2*pi*j/N_circle);sin(2*pi*j/N_circle);0]);
                end            
                hold on
                plot3([p_IR1h_raw(1) p_IR1_raw(1) p_IR3_raw(1) p_IR3h_raw(1)],...
                    [p_IR1h_raw(2) p_IR1_raw(2) p_IR3_raw(2) p_IR3h_raw(2)],...
                    [p_IR1h_raw(3) p_IR1_raw(3) p_IR3_raw(3) p_IR3h_raw(3)],'k-')
                hold on
                plot3([p_IR2h_raw(1) p_IR2_raw(1) p_IR4_raw(1) p_IR4h_raw(1)],...
                    [p_IR2h_raw(2) p_IR2_raw(2) p_IR4_raw(2) p_IR4h_raw(2)],...
                    [p_IR2h_raw(3) p_IR2_raw(3) p_IR4_raw(3) p_IR4h_raw(3)],'k-')
                hold on
                plot3(p_R1_raw(1,:),p_R1_raw(2,:),p_R1_raw(3,:),'k-');
                hold on
                plot3(p_R2_raw(1,:),p_R2_raw(2,:),p_R2_raw(3,:),'k-');
                hold on
                plot3(p_R3_raw(1,:),p_R3_raw(2,:),p_R3_raw(3,:),'k-');
                hold on
                plot3(p_R4_raw(1,:),p_R4_raw(2,:),p_R4_raw(3,:),'k-');
                % added equipment
                hold on
                p_IS1_raw = q_raw(1:3,i) + R_IB_raw*p_BS1_raw;
                p_IS2_raw = p_IS1_raw + R_IB_raw*[param.l_1*sin(q_raw(7,i));0;param.l_2*cos(q_raw(7,i))];
                plot3([q_raw(1,i) p_IS1_raw(1) p_IS2_raw(1) p_IE_raw(1,i)],...
                    [q_raw(2,i) p_IS1_raw(2) p_IS2_raw(2) p_IE_raw(2,i)],...
                    [q_raw(3,i) p_IS1_raw(3) p_IS2_raw(3) p_IE_raw(3,i)],'k-');   
            % decorate the plot
            grid on
            xlabel('p_x [m]')
            ylabel('p_y [m]')
            zlabel('p_z [m]')
            title(['Aerial Manipulator at t = ', num2str(tspan(i)), ' sec'])
            % limit plot space
            xlim([x_min x_max]);
            ylim([y_min y_max]);
            zlim([z_min z_max]);
            % force Matlab to draw the image at this point
            drawnow
        end
    end
end

