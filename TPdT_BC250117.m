% copyright @ Ezekiel Mok
% mozhb@mail2.sysu.edu.cn
% 2025/01/17
clc,clear
close all
sim.t_total = 24;%s
sim.point_num=2000;
sim.ts = sim.t_total / sim.point_num;
sim.t=linspace(0, sim.t_total, sim.point_num);
cx = zeros(sim.point_num,1);
cy= zeros(sim.point_num,1);
cz= zeros(sim.point_num,1);
start_ang=0.55*pi;
end_ang=2.25*pi;
x0 = @(t_step) 16*sin(2 * t_step + 1);
y0 = @(t_step) -40*cos(t_step+ 0.5);
z0 = @(t_step) 0;
start_psi=0;
end_psi=sim.t_total;
k=0;
for theta=start_psi:(end_psi-start_psi)/(sim.point_num-1):end_psi
    k=k+1;
    cx(k) = x0( start_ang+theta/(end_psi-start_psi)*(end_ang-start_ang));
    cy(k) = y0( start_ang+theta/(end_psi-start_psi)*(end_ang-start_ang));
    cz(k)=0;
end
% figure(1)
% plot(cx, cy, 'm.',LineWidth=2)
% axis equal
% grid on
refer_path(:, 1:3) =[cx, cy, cz];
for i=1:length(refer_path)
    if i==1
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        dz = refer_path(i + 1, 3) - refer_path(i, 3);
        ddx = refer_path(3, 1) + refer_path(1, 1) - 2 * refer_path(2, 1);
        ddy = refer_path(3, 2) + refer_path(1, 2) - 2 * refer_path(2, 2);
        ddz = refer_path(3, 3) + refer_path(1, 3) - 2 * refer_path(2, 3);
    elseif  i==length(refer_path)
        dx = refer_path(i, 1) - refer_path(i - 1, 1);
        dy = refer_path(i, 2) - refer_path(i - 1, 2);
        dz = refer_path(i, 3) - refer_path(i - 1, 3);
        ddx = refer_path(i, 1) + refer_path(i - 2, 1) - 2 * refer_path(i - 1, 1);
        ddy = refer_path(i, 2) + refer_path(i - 2, 2) - 2 * refer_path(i - 1, 2);
        ddz = refer_path(i, 3) + refer_path(i - 2, 3) - 2 * refer_path(i - 1, 3);
    else
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        dz = refer_path(i + 1, 3) - refer_path(i, 3);
        ddx = refer_path(i + 1, 1) + refer_path(i - 1, 1) - 2 * refer_path(i, 1);
        ddy = refer_path(i + 1, 2) + refer_path(i - 1, 2) - 2 * refer_path(i, 2);
        ddz = refer_path(i + 1, 3) + refer_path(i - 1, 3) - 2 * refer_path(i, 3);
    end
    refer_path(i,4)=atan2(dy, dx);%
    refer_path(i,5)=(ddy * dx - ddx * dy) / ((dx ^ 2 + dy ^ 2) ^ (3 / 2));
    refer_path(i,6) = dx;
    refer_path(i,7) = dy;
    refer_path(i,8) = dz;
end
for i=1:length(refer_path)
    if i==1
        dphai = refer_path(i + 1, 4) - refer_path(i, 4);
    elseif  i==length(refer_path)
        dphai = refer_path(i,4) - refer_path(i - 1, 4);
    else
        dphai = refer_path(i + 1, 4) - refer_path(i, 4);
    end
    refer_path(i, 9) = dphai;%yaw
end
x_d=refer_path(:, 1);
y_d=refer_path(:, 2);
z_d=refer_path(:, 3);
phai_d = refer_path(:, 4);
dx_d = refer_path(:, 6);
dy_d = refer_path(:, 7);
dz_d = refer_path(:, 8);
dphai_d = refer_path(:, 9);
sim.list_x_rd = refer_path(:, 1:3)';
sim.list_v_rd =refer_path(:, 6:8)';
sim.x_0 =sim.list_x_rd(:,1);% 可以看做领航者的初始位置
sim.v_0 = [0; 0;0];% 可以看做领航者的初始速度
sim.x_10 =[-17.5,15,0]';%
sim.v_10 = [0; 0;0];%
sim.x_20 =[-10,5,0]';%
sim.v_20 = [0; 0;0];%
sim.x_30 =[-30,10,0]';%
sim.v_30 = [0; 0;0];%
ds_agent0=4;
ds_agent=2.8;
r_robot=0.4;
agent0 = agentclass2dim(sim.x_0, sim.v_0, sim.list_v_rd, sim.list_x_rd, ds_agent0, r_robot);% 虚拟领航者
agent1 = agentclass2dim(sim.x_10, sim.v_10, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体1
agent2 = agentclass2dim(sim.x_20, sim.v_20, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体2
agent3 = agentclass2dim(sim.x_30, sim.v_30, sim.list_v_rd, sim.list_x_rd, ds_agent, r_robot);% 智能体3
k=0;
param.k1 =0.75;
param.k2 = 1-param.k1;
param.alpha=1;
param.beta =1;
param.zeta =1;
param.q =2;
param.km = 3.5;
param.k_f = 12;
param.ka_gain = 24;
param.A_matrix = [0 0.5, 0.5;
    1, 0, 0;
    1, 0, 0];%有向生成树图
param.B_matrix = [1, 0, 0]';%表征与领航者的直接通讯关系
param.K_matrix = diag([sum(param.A_matrix(1,:)),sum(param.A_matrix(2,:)), ...
    sum(param.A_matrix(3,:))]);
param.L_matrix = param.K_matrix - param.A_matrix;
param.H_matrix = param.L_matrix + param.B_matrix;
param.P_matrix =diag([1,0.5,0.5]);
param.lamda_PH=min(eig(param.P_matrix * param.H_matrix));
param.rho_epv = 0.75;
param.mu_epv= 1;
param.T_epv = 8;
param.lo1 = -0.1;
param.lo2 = -0.1;
param.kf_gain = 12;
param.rho_f = 0.5;
param.rho_f1=0.5;
param.mu_f = 0.6;
param.mu_f1 = 0.6;
param.T_f = 6;
param.T_f1 = 6;
param.fontsize=15;
a_interg=0;
aa_interg=0;
b_interg=0;
bb_interg=0;
x1_esm=agent0.x-[0.5,0.5,0]'*5;
x2_esm=agent0.x+[0.4,0.4,0]'*5;
x3_esm=agent0.x+[0.3,0.3,0]'*5;
v1_esm = agent0.v;
v2_esm = agent0.v;
v3_esm = agent0.v;
ldestimator.lo1 = 0.1;
ldestimator.lo2 = 0.1;
ldestimator.lo3 = 25;
ldestimator.lo4 = 6;
ldestimator.yita1 = 3/2;
ldestimator.yita2 = 3/2;
param.km_gain = 3.5;
param.rho_c = 0.5;
param.rho_c1=0.5;
param.mu_c = 0.6;
param.mu_c1 = 0.6;
param.T_c = 6;
param.T_c1 = 6;
sm_p1=[0,0,0]';
sm_p2=[4,-4,8]';
sm_p3=[4,4,8]';
sm_p1_dot=[0,0,0]';
sm_p2_dot=[0,0,0]';
sm_p3_dot=[0,0,0]';
sigma_p = [sm_p1, sm_p2, sm_p3];
sigma_p1 = [sm_p1, sm_p2, sm_p3];
sigma_pv = [sm_p1_dot, sm_p2_dot, sm_p3_dot];
trace_err1=nan+[sim.t];
trace_err2=nan+[sim.t];
trace_err3=nan+[sim.t];
trace_terr1=nan+[sim.t];
trace_terr2=nan+[sim.t];
trace_terr3=nan+[sim.t];
trace_esterr1=nan+[sim.t];
trace_esterr2=nan+[sim.t];
trace_esterr3=nan+[sim.t];
trace_obs1=nan+[sim.t];
trace_obs2=nan+[sim.t];
trace_obs3=nan+[sim.t];
trace_u1=nan+[sim.t];
trace_u2=nan+[sim.t];
trace_u3=nan+[sim.t];
trace_x0=nan+[sim.t;sim.t;sim.t];
trace_agentdis1=nan+[sim.t];
trace_agentdis2=nan+[sim.t];
trace_agentdis3=nan+[sim.t];
 trace_dis_12=nan+[sim.t];
 trace_dis_23=nan+[sim.t];
 trace_dis_13=nan+[sim.t];
sim.obstacle_x =[0,9.5,0;9.5,2,0;0,-35,8; 15,-18,0; -14,-25,0; 0,35,8;18,29,0]';%
sim.obstacle_v =zeros(3,5);
int_c=[0,0,0]';
int_f1 = [0,0,0]';
int_f2 = [0,0,0]';
int_f3 = [0,0,0]';
x1 = agent1.x;
x2 = agent1.v;
x21 = agent2.x;
x22 = agent2.v;
x31 = agent3.x;
x32 = agent3.v;
z_10 = [0.01, 0.01,0.01]';
z_20 = [0.01, 0.01,0.01]';
z1_esm = z_10;
z2_esm = z_20;
z21_esm = z_10;
z22_esm = z_20;
z31_esm = z_10;
z32_esm = z_20;
z1 = z_10;
z2 = z_20;
z21 = z_10;
z22 = z_20;
z31 = z_10;
z32 = z_20;
u1 = [0, 0, 0]';
u2 = [0, 0, 0]';
u3 = [0, 0, 0]';
mu1 = 0.6;
mu2 = 0.2;
r1= 0.7;
r2= 0.7;
ctr_param.rho=0.6;
ctr_param.T = 2;
ctr_param.gama = -1.5;
ctr_param.cmu_p = 1.2;
for j = sim.t
    k=k+1;
    agent0.record_x(k)=agent0.x(1);
    agent0.record_y(k)=agent0.x(2);
    agent0.record_z(k)=agent0.x(3);
    agent0.record_psi(k)=refer_path(k, 4);
    agent1.record_x(k)=agent1.x(1);
    agent1.record_psi(k)=refer_path(k, 4);
    agent1.record_y(k)=agent1.x(2);
    agent1.record_z(k)=agent1.x(3);
    agent2.record_x(k)=agent2.x(1);
    agent2.record_y(k)=agent2.x(2);
    agent2.record_psi(k)=refer_path(k, 4);
    agent2.record_z(k)=agent2.x(3);
    agent3.record_x(k)=agent3.x(1);
    agent3.record_y(k)=agent3.x(2);
    agent3.record_z(k)=agent3.x(3);
    trace_dis_12(k)=norm(agent1.x-agent0.x-sigma_p1(:,1));
    trace_dis_23(k)=norm(agent2.x-agent0.x-sigma_p1(:,2));
    trace_dis_13(k)=norm(agent3.x-agent0.x-sigma_p1(:,3));
    agent3.record_psi(k)=refer_path(k, 4);
    bar_sigma_c = sim.list_x_rd(:, k) - agent0.x;
    int_c_dot = -(2 ./ ( param.km_gain .* param.rho_c .* param.mu_c .* param.T_c))*(2 .* param.mu_c .* bar_sigma_c + return_sign2(bar_sigma_c, ...
        1-param.rho_c) + param.mu_c^2 .* return_sign2(bar_sigma_c, 1+param.rho_c));
    int_c = int_c + int_c_dot * sim.ts;
    s_c = bar_sigma_c + int_c;
    [v_m0, jacobi_m0] = PIBC(param, bar_sigma_c,  sim.list_v_rd(:, k)./sim.ts, s_c);
    agent0.v = v_m0;
    agent0.x = agent0.x + sim.ts .* agent0.v;
    p_now = [agent1.x, agent2.x, agent3.x];
    v_now = [agent1.v, agent2.v, agent3.v];
    [min_id1,min_pos1, min_dis_detected1] = returnnearobstacle(p_now,  sim.obstacle_x,  1) ;
    [min_id2,min_pos2, min_dis_detected2] = returnnearobstacle(p_now,  sim.obstacle_x,  2) ;
    [min_id3,min_pos3, min_dis_detected3] = returnnearobstacle(p_now,  sim.obstacle_x,  3) ;
    [pmin_id1, min_dis1] = returnnearagent(p_now, 1) ;
    [pmin_id2, min_dis2] = returnnearagent(p_now, 2) ;
    [pmin_id3, min_dis3] = returnnearagent(p_now, 3) ;
    trace_obs1(k)=min_dis_detected1;
    trace_obs2(k)=min_dis_detected2;
    trace_obs3(k)=min_dis_detected3;
    trace_agentdis1(k)=min_dis1;
    trace_agentdis2(k)=min_dis2;
    trace_agentdis3(k)=min_dis3;
    trace_obs1(k)=min_dis_detected1;
    trace_obs2(k)=min_dis_detected2;
    trace_obs3(k)=min_dis_detected3;
    trace_agentdis1(k)=min_dis1;
    trace_agentdis2(k)=min_dis2;
    trace_agentdis3(k)=min_dis3;
    time_start_scaleup=720*sim.ts;
    time_span_scaledown=1.5;
    if j>=time_start_scaleup && j<=time_start_scaleup+time_span_scaledown
        % agent 3
        a=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.5);
        % agent 3
        b=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a * sim.ts;
        aa_interg = aa_interg + a_interg * sim.ts;
        b_interg = b_interg + b * sim.ts;
        bb_interg = bb_interg + b_interg * sim.ts;
    end
    time_start_scaledown=850*sim.ts;
    time_span_scaledown=1.5;
    if j>=time_start_scaledown && j<=time_start_scaledown+time_span_scaledown
        % agent 3
        a=-Accel_slow_fast(time_start_scaledown, time_span_scaledown, j, 0.5);
        % agent 3
        b=-Accel_slow_fast(time_start_scaledown, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a * sim.ts;
        aa_interg = aa_interg + a_interg * sim.ts;
        b_interg = b_interg + b * sim.ts;
        bb_interg = bb_interg + b_interg * sim.ts;
    end
    time_start_scaleup=1800*sim.ts;
    time_span_scaledown=1.5;
    if j>=time_start_scaleup && j<=time_start_scaleup+time_span_scaledown
        % agent 3
        a=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j,0.82);
        % agent 3
        b=Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.82);
        a_interg = a_interg + a * sim.ts;
        aa_interg = aa_interg + a_interg * sim.ts;
        b_interg = b_interg + b * sim.ts;
        bb_interg = bb_interg + b_interg * sim.ts;
    end
    offset=[0,0,0];
    sigma_p1_last=sigma_p1;
    for i=1:1:3
        sigma_p1(:,i)=Aff('A', refer_path(k, 4), 1+aa_interg, 1+aa_interg,1+aa_interg,  0, 0, 0, sigma_p(:,i));
    end
    sigma_pv= (1/sim.ts).*(sigma_p1_last-sigma_p1);
    p_esm   = [x1_esm, x2_esm, x3_esm];
    pv_esm = [v1_esm, v2_esm, v3_esm];
    [x1_esm_dot, v1_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v,  1);
    [x2_esm_dot, v2_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v,  2);
    [x3_esm_dot, v3_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v,  3);
    x1_esm = x1_esm + sim.ts .* x1_esm_dot;
    x2_esm = x2_esm + sim.ts .* x2_esm_dot;
    x3_esm = x3_esm + sim.ts .* x3_esm_dot;
    v1_esm = v1_esm + sim.ts .* v1_esm_dot;
    v2_esm = v2_esm + sim.ts .* v2_esm_dot;
    v3_esm = v3_esm + sim.ts .* v3_esm_dot;
    [bar_sigma_f1, sigmaf_i1, pf_i1, vf_i1] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p1, sigma_pv, 1);
    int_f1_dot = -(2 ./ ( param.kf_gain*param.rho_f .* param.mu_f .* param.T_f))* (2 .* param.mu_f ...
        .* bar_sigma_f1 + return_sign2(bar_sigma_f1, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f1, 1+param.rho_f));
    int_f1 = int_f1 + int_f1_dot * sim.ts;
    s_f1 = bar_sigma_f1 + int_f1;
    [v_fm1, jacobi_fm1] = PDFBC(param,  bar_sigma_f1, s_f1, vf_i1);
    [bar_sigma_f2, sigmaf_i2, pf_i2, vf_i2] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p1, sigma_pv, 2);
    int_f2_dot = -(2 ./ ( param.kf_gain*param.rho_f .* param.mu_f .* param.T_f))* (2 .* param.mu_f ...
        .* bar_sigma_f2 + return_sign2(bar_sigma_f2, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f2, 1+param.rho_f));
    int_f2 = int_f2 + int_f2_dot * sim.ts;
    s_f2 = bar_sigma_f2 + int_f2;
    [v_fm2, jacobi_fm2] = PDFBC(param,  bar_sigma_f2, s_f2, vf_i2);
    [bar_sigma_f3, sigmaf_i3, pf_i3, vf_i3] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p1, sigma_pv, 3);
    int_f3_dot = -(2 ./ ( param.kf_gain*param.rho_f .* param.mu_f .* param.T_f))* (2 .* param.mu_f ...
        .* bar_sigma_f3 + return_sign2(bar_sigma_f3, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f3, 1+param.rho_f));
    int_f3 = int_f3 + int_f3_dot * sim.ts;
    s_f3 = bar_sigma_f3 + int_f3;
    [v_fm3, jacobi_fm3] = PDFBC(param,  bar_sigma_f3, s_f3, vf_i3);
    if min_dis_detected1<=agent1.R_safe
        [smrho_oa1, jacobi_o1] = returnobstaclejacobi(p_now, agent1.R_safe, ...
            agent1.r_robot, sim.obstacle_x(:,min_id1), min_dis_detected1, 1);
        v_o1 = pinv(jacobi_o1).*param.ka_gain*(agent1.R_safe - smrho_oa1);
    else
        v_o1 = zeros(3, 1);
        jacobi_o1 = zeros(1, 3);
    end
    if min_dis_detected2<=agent2.R_safe
        [smrho_oa2, jacobi_o2] = returnobstaclejacobi(p_now, agent2.R_safe, ...
            agent2.r_robot, sim.obstacle_x(:,min_id2), min_dis_detected2, 2);
        v_o2 = pinv(jacobi_o2).*param.ka_gain*(agent2.R_safe - smrho_oa2);
    else
        v_o2 = zeros(3, 1);
        jacobi_o2 = zeros(1, 3);
    end
    if min_dis_detected3<=agent3.R_safe
        [smrho_oa3, jacobi_o3] = returnobstaclejacobi(p_now, agent3.R_safe, ...
            agent3.r_robot, sim.obstacle_x(:,min_id3), min_dis_detected3, 3);
        v_o3 = pinv(jacobi_o3).*param.ka_gain*(agent3.R_safe - smrho_oa3);
    else
        v_o3 = zeros(3, 1);
        jacobi_o3 = zeros(1, 3);
    end
    if min_dis_detected1<=agent1.R_safe
        agent1.v =  v_o1 + (eye(3, 3) - pinv(jacobi_o1) * jacobi_o1) *v_fm1;
    else
        agent1.v =  v_fm1 + (eye(3, 3) - pinv(jacobi_fm1) * jacobi_fm1) * v_o1;
    end
    agent1.x =  agent1.x + sim.ts .* agent1.v;
    if min_dis_detected2<=agent2.R_safe
        agent2.v =  v_o2 + (eye(3, 3) - pinv(jacobi_o2) * jacobi_o2) *v_fm2;
    else
        agent2.v =  v_fm2 + (eye(3, 3) - pinv(jacobi_fm2) * jacobi_fm2) * v_o2;
    end
    agent2.x =  agent2.x + sim.ts .* agent2.v;
    if min_dis_detected3<=agent3.R_safe
        agent3.v =  v_o3 + (eye(3, 3) - pinv(jacobi_o3) * jacobi_o3) *v_fm3;
    else
        agent3.v =  v_fm3 + (eye(3, 3) - pinv(jacobi_fm3) * jacobi_fm3) * v_o3;
    end
    agent3.x =  agent3.x + sim.ts .* agent3.v;
    trace_u1(k)=norm(u1);
    trace_u2(k)=norm(u2);
    trace_u3(k)=norm(u3);
    %%  机器人1
    [x1_dot, x2_dot] = dynamic_model(u1, x1, x2, j);
    x1 = x1 + sim.ts .* x1_dot;
    x2 = x2 + sim.ts .* x2_dot;
    err = x1 - agent1.x;
    err_dot = x2  - agent1.v;
    v_term = ctr_term(ctr_param, err, err_dot , z2_esm);
    z1 =err_dot;
    e1 = z1 - z1_esm;
    z1_esm_dot = v_term + z2_esm + mu1 .* return_sign2(e1, r1);
    z2_esm_dot = mu2 .*return_sign2(e1, r2);
    z1_esm = z1_esm +   sim.ts .* z1_esm_dot;
    z2_esm = z2_esm +   sim.ts .* z2_esm_dot;
    B1 = eye(3,3);
    f_11=0.5*[-0.5*x1(1)+x1(2)*(1+0.5*x1(2)^2)+cos(x2(1));...
        -0.8*(x1(1)+x1(2))+0.5*x1(2)*(1-0.3*x1(2)^2)+sin(x2(2));...
        -0.8*(x1(1)+x1(2))+0.5*x1(3)*(1-0.3*x1(2)^2)+sin(x2(3))];
    u1 =  - pinv(B1) * (f_11)  + pinv(B1) * v_term;
    %%  机器人2
    [x21_dot, x22_dot] = dynamic_model(u2, x21, x22, j);
    x21 = x21 + sim.ts .* x21_dot;
    x22 = x22 + sim.ts .* x22_dot;
    err2 = x21 - agent2.x;
    err2_dot = x22  - agent2.v;
    v_term2 = ctr_term(ctr_param, err2, err2_dot , z22_esm);
    z21 =err2_dot;
    e21 = z21 - z21_esm;
    z21_esm_dot = v_term2 + z22_esm + mu1 .* return_sign2(e21, r1);
    z22_esm_dot = mu2 .*return_sign2(e21, r2);
    z21_esm = z21_esm +   sim.ts .* z21_esm_dot;
    z22_esm = z22_esm +   sim.ts .* z22_esm_dot;
    B2 = eye(3,3);
    f_22=0.5*[-0.5*x21(1)+x21(2)*(1+0.5*x21(2)^2)+cos(x22(1));...
        -0.8*(x21(1)+x21(2))+0.5*x21(2)*(1-0.3*x21(2)^2)+sin(x22(2));...
        -0.8*(x21(1)+x21(2))+0.5*x21(3)*(1-0.3*x21(2)^2)+sin(x22(3))];
    u2 =  - pinv(B2) * (f_22)  + pinv(B2) * v_term2;
    %%  机器人3
    [x31_dot, x32_dot] = dynamic_model(u3, x31, x32, j);
    x31 = x31 + sim.ts .* x31_dot;
    x32 = x32 + sim.ts .* x32_dot;
    err3 = x31 - agent3.x;
    err3_dot = x32  - agent3.v;
    v_term3 = ctr_term(ctr_param, err3, err3_dot , z32_esm);
    z31 =err3_dot;
    e31 = z31 - z31_esm;
    z31_esm_dot = v_term3 + z32_esm + mu1 .* return_sign2(e31, r1);
    z32_esm_dot = mu2 .*return_sign2(e31, r2);
    z31_esm = z31_esm +   sim.ts .* z31_esm_dot;
    z32_esm = z32_esm +   sim.ts .* z32_esm_dot;
    B3 = eye(3,3);
    f_33=0.5*[-0.5*x31(1)+x31(2)*(1+0.5*x31(2)^2)+cos(x32(1));...
        -0.8*(x31(1)+x31(2))+0.5*x31(2)*(1-0.3*x31(2)^2)+sin(x32(2));...
        -0.8*(x31(1)+x31(2))+0.5*x31(3)*(1-0.3*x31(2)^2)+sin(x32(3))];
    u3 =  - pinv(B3) * (f_33)  + pinv(B3) * v_term3;
    trace_err1(k)=norm(bar_sigma_f1);
    trace_err2(k)=norm(bar_sigma_f2);
    trace_err3(k)=norm(bar_sigma_f3);
    trace_esterr1(k)=norm(agent0.x-p_esm(:,1));
    trace_esterr2(k)=norm(agent0.x-p_esm(:,2));
    trace_esterr3(k)=norm(agent0.x-p_esm(:,3));
end
agcolor1=[0.99, 0.49, 0.00];
agcolor2=[0.17, 0.62, 0.47];
agcolor3=[0.88, 0.17, 0.56];
figure(6)
subplot(2,1,1)
hold on; box on;
grid on
set(gcf, 'Position', [-1800, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'ylabel'), 'String', 'Distance (m)', 'fontSize', param.fontsize);
set(get(gca, 'xlabel'), 'String', 't (s)', 'fontSize', param.fontsize);
set(gca, 'fontsize', param.fontsize);
plot(sim.t,  trace_agentdis1, Color=agcolor1,LineStyle=':',LineWidth=2)
plot(sim.t, trace_agentdis2, Color=agcolor2,LineStyle='--',LineWidth=2)
plot(sim.t, trace_agentdis3, Color=agcolor3,LineStyle='-.',LineWidth=2)
line_obs=2*ones(1,2000);
plot(sim.t, line_obs, Color='r',LineStyle='-',LineWidth=2)
legend('Robot 1','Robot 2','Robot 3','Safty distance to other robots (D_s=2m)','Location','northEast',Orientation='vertical');
axis([0,24,0,20])
subplot(2,1,2)
hold on; box on;
grid on
set(gcf, 'Position', [-1800, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'ylabel'), 'String', 'Distance (m)', 'fontSize', param.fontsize);
set(get(gca, 'xlabel'), 'String', 't (s)', 'fontSize', param.fontsize);
set(gca, 'fontsize', param.fontsize);
plot(sim.t, trace_obs1, Color=agcolor1,LineStyle=':',LineWidth=2)
plot(sim.t, trace_obs2, Color=agcolor2,LineStyle='--',LineWidth=2)
plot(sim.t, trace_obs3, Color=agcolor3,LineStyle='-.',LineWidth=2)
line_obs=2*ones(1,2000);
plot(sim.t, line_obs, Color='r',LineStyle='-',LineWidth=2)
legend('Robot 1','Robot 2','Robot 3','Safty distance to obstacles (D_s=2m)','Location','northEast',Orientation='vertical');
axis([0,24,0,30])

figure(2);
subplot(3,1,1)
hold on; box on;
grid on
set(gcf, 'Position', [-1800, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'ylabel'), 'String', 'Task error (m)', 'fontSize', param.fontsize);
set(get(gca, 'xlabel'), 'String', 't (s)', 'fontSize', param.fontsize);
set(gca, 'fontsize', param.fontsize);
plot(sim.t, trace_err1, Color=agcolor1,LineStyle=':',LineWidth=2)
plot(sim.t, trace_err2, Color=agcolor2,LineStyle='--',LineWidth=2)
plot(sim.t, trace_err3, Color=agcolor3,LineStyle='-.',LineWidth=2)
axis([0,24,0,24])
x = 2.5;
y = [0,24];
n = length(x);
line([x,x], [y(1),y(2)], 'color', 'k',LineWidth=2)
text(x+0.2,y(1)+(y(2)-y(1))/2, 't=2.5s', 'Color','k')
legend('Robot1: ||q_1-q_0-ρ_1||','Robot2: ||q_2-q_0-ρ_2||','Robot3: ||q_3-q_0-ρ_3||','Location','northEast');
subplot(3,1,2)
hold on; box on;
grid on
set(gcf, 'Position', [-1800, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'ylabel'), 'String', 'Estimating error (m)', 'fontSize', param.fontsize);
set(get(gca, 'xlabel'), 'String', 't (s)', 'fontSize', param.fontsize);
set(gca, 'fontsize', param.fontsize);
plot(sim.t, trace_esterr1, Color=agcolor1,LineStyle=':',LineWidth=2)
plot(sim.t, trace_esterr2, Color=agcolor2,LineStyle='--',LineWidth=2)
plot(sim.t, trace_esterr3, Color=agcolor3,LineStyle='-.',LineWidth=2)

x = 2.5;
y = [0,2];
n = length(x);
line([x,x], [y(1),y(2)], 'color', 'k',LineWidth=2)
text(x+0.2,y(1)+(y(2)-y(1))/2, 't=2.5s', 'Color','k')
axis([0,24,0,2])
legend('Robot 1','Robot 2','Robot 3','Location','northEast');
subplot(3,1,3)
hold on; box on;
grid on
set(gcf, 'Position', [-1800, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'ylabel'), 'String', 'Control input (N·m)', 'fontSize', param.fontsize);
set(get(gca, 'xlabel'), 'String', 't (s)', 'fontSize', param.fontsize);
set(gca, 'fontsize', param.fontsize);
plot(sim.t, trace_u1, Color=agcolor1,LineStyle=':',LineWidth=2)
plot(sim.t, trace_u2, Color=agcolor2,LineStyle='--',LineWidth=2)
plot(sim.t, trace_u3, Color=agcolor3,LineStyle='-.',LineWidth=2)
legend('Robot 1','Robot 2','Robot 3','Location','northEast',Orientation='vertical');
axis([0,24,0,4*10^4])
figure(1);
hold on; box on;
grid on
set(gcf, 'Position', [-1200, 100, 1.2*800, 1.2*600]);
set(gca, 'fontSize', param.fontsize)
set(get(gca, 'xlabel'), 'String', 'x (m)', 'fontSize', param.fontsize);
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', param.fontsize);
set(get(gca, 'zlabel'), 'String', 'z (m)', 'fontSize', param.fontsize);
axis([-38,38,-48,48,-10,40])
hg   = gca;
view(3)
set(gca, 'fontsize', param.fontsize);
idx=2000;
h0=plot3(agent0.record_x(1:idx),agent0.record_y(1:idx),agent0.record_z(1:idx),'--','Color','b', 'linewidth', 3);
h1=plot3(agent1.record_x(1:idx),agent1.record_y(1:idx),agent1.record_z(1:idx),'-','Color',agcolor1, 'linewidth', 2);
h2=plot3(agent2.record_x(1:idx),agent2.record_y(1:idx),agent2.record_z(1:idx),'-','Color',agcolor2, 'linewidth', 2);
h3=plot3(agent3.record_x(1:idx),agent3.record_y(1:idx),agent3.record_z(1:idx),'-','Color',agcolor3, 'linewidth', 2);
[~,obs_num]=size(sim.obstacle_x);
obs1=pbsplot('C',sim.obstacle_x(1,1),sim.obstacle_x(2,1),sim.obstacle_x(3,1),3,15,0.3*ones(1,3));
obs2=pbsplot('C',sim.obstacle_x(1,2),sim.obstacle_x(2,2),sim.obstacle_x(3,2),3,15,0.3*ones(1,3));
obs3=pbsplot('S',sim.obstacle_x(1,3),sim.obstacle_x(2,3),sim.obstacle_x(3,3),2,0,0.3*ones(1,3));
obs4=pbsplot('S',sim.obstacle_x(1,4),sim.obstacle_x(2,4),sim.obstacle_x(3,4),2,0,0.3*ones(1,3));
obs5=pbsplot('S',sim.obstacle_x(1,5),sim.obstacle_x(2,5),sim.obstacle_x(3,5),2,0,0.3*ones(1,3));
obs6=pbsplot('S',sim.obstacle_x(1,6),sim.obstacle_x(2,6),sim.obstacle_x(3,6),2,0,0.3*ones(1,3));
obs7=pbsplot('S',sim.obstacle_x(1,7),sim.obstacle_x(2,7),sim.obstacle_x(3,7),2,0,0.3*ones(1,3));
index_star=1;
line_color='k';
line_styte=':';
line_w=2;
[drone1,combinedobject1] = drone_plot(hg,agcolor2);
translation1 = makehgtform('translate', [agent2.record_x(index_star),agent2.record_y(index_star),agent2.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent2.record_psi(index_star));
set(combinedobject1,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[drone2,combinedobject2] = drone_plot(hg,agcolor3);
translation1 = makehgtform('translate', [agent3.record_x(index_star),agent3.record_y(index_star),agent3.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent3.record_psi(index_star));
set(combinedobject2,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[satellite_1,combinedobject1] = drone_plot(hg,agcolor1);
set(combinedobject1,'matrix',...
    makehgtform('translate', [agent1.record_x(index_star),agent1.record_y(index_star),agent1.record_z(index_star)])*makehgtform('zrotate',agent1.record_psi(index_star)));
line([agent1.record_x(index_star),agent2.record_x(index_star)],[agent1.record_y(index_star), ...
    agent2.record_y(index_star)],[agent1.record_z(index_star), ...
    agent2.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent1.record_x(index_star),agent3.record_x(index_star)],[agent1.record_y(index_star), ...
    agent3.record_y(index_star)],[agent1.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent2.record_x(index_star),agent3.record_x(index_star)],[agent2.record_y(index_star), ...
    agent3.record_y(index_star)],[agent2.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
index_star=100;
line_color='k';
line_styte=':';
line_w=2;
[drone3,combinedobject3] = drone_plot(hg,agcolor2);
translation1 = makehgtform('translate', [agent2.record_x(index_star),agent2.record_y(index_star),agent2.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent2.record_psi(index_star));
set(combinedobject3,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[drone4,combinedobject4] = drone_plot(hg,agcolor3);
translation1 = makehgtform('translate', [agent3.record_x(index_star),agent3.record_y(index_star),agent3.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent3.record_psi(index_star));
set(combinedobject4,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[satellite_5,combinedobject5] = drone_plot(hg,agcolor1);
set(combinedobject5,'matrix',...
    makehgtform('translate', [agent1.record_x(index_star),agent1.record_y(index_star),agent1.record_z(index_star)])*makehgtform('zrotate',agent1.record_psi(index_star)));
line([agent1.record_x(index_star),agent2.record_x(index_star)],[agent1.record_y(index_star), ...
    agent2.record_y(index_star)],[agent1.record_z(index_star), ...
    agent2.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent1.record_x(index_star),agent3.record_x(index_star)],[agent1.record_y(index_star), ...
    agent3.record_y(index_star)],[agent1.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent2.record_x(index_star),agent3.record_x(index_star)],[agent2.record_y(index_star), ...
    agent3.record_y(index_star)],[agent2.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
index_star=2000;
line_color='k';
line_styte=':';
line_w=2;
[drone6,combinedobject6] = drone_plot(hg,agcolor2);
translation1 = makehgtform('translate', [agent2.record_x(index_star),agent2.record_y(index_star),agent2.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent2.record_psi(index_star));
set(combinedobject6,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[drone7,combinedobject7] = drone_plot(hg,agcolor3);
translation1 = makehgtform('translate', [agent3.record_x(index_star),agent3.record_y(index_star),agent3.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent3.record_psi(index_star));
set(combinedobject7,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[satellite_8,combinedobject8] = drone_plot(hg,agcolor1);
set(combinedobject8,'matrix',...
    makehgtform('translate', [agent1.record_x(index_star),agent1.record_y(index_star),agent1.record_z(index_star)])*makehgtform('zrotate',agent1.record_psi(index_star)));
line([agent1.record_x(index_star),agent2.record_x(index_star)],[agent1.record_y(index_star), ...
    agent2.record_y(index_star)],[agent1.record_z(index_star), ...
    agent2.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent1.record_x(index_star),agent3.record_x(index_star)],[agent1.record_y(index_star), ...
    agent3.record_y(index_star)],[agent1.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent2.record_x(index_star),agent3.record_x(index_star)],[agent2.record_y(index_star), ...
    agent3.record_y(index_star)],[agent2.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
index_star=1800;
line_color='k';
line_styte=':';
line_w=2;
[drone11,combinedobject11] = drone_plot(hg,agcolor2);
translation1 = makehgtform('translate', [agent2.record_x(index_star),agent2.record_y(index_star),agent2.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent2.record_psi(index_star));
set(combinedobject11,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[drone9,combinedobject9] = drone_plot(hg,agcolor3);
translation1 = makehgtform('translate', [agent3.record_x(index_star),agent3.record_y(index_star),agent3.record_z(index_star)]);
rotation11 = makehgtform('xrotate',(pi/180)*(0));
rotation12 = makehgtform('yrotate',(pi/180)*(0));
rotation13 = makehgtform('zrotate',agent3.record_psi(index_star));
set(combinedobject9,'matrix',...
    translation1*rotation13*rotation12*rotation11);
[satellite_10,combinedobject10] = drone_plot(hg,agcolor1);
set(combinedobject10,'matrix',...
    makehgtform('translate', [agent1.record_x(index_star),agent1.record_y(index_star),agent1.record_z(index_star)])*makehgtform('zrotate',agent1.record_psi(index_star)));
line([agent1.record_x(index_star),agent2.record_x(index_star)],[agent1.record_y(index_star), ...
    agent2.record_y(index_star)],[agent1.record_z(index_star), ...
    agent2.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent1.record_x(index_star),agent3.record_x(index_star)],[agent1.record_y(index_star), ...
    agent3.record_y(index_star)],[agent1.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
line([agent2.record_x(index_star),agent3.record_x(index_star)],[agent2.record_y(index_star), ...
    agent3.record_y(index_star)],[agent2.record_z(index_star), ...
    agent3.record_z(index_star)],'Color',line_color,'LineStyle',line_styte, LineWidth=line_w)
text(-20, 0, 0,['#t=', num2str(0/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
text(-25, 30, 0,['#t=', num2str(100/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
text(25, -35, 0,['#t=', num2str(1800/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
text(15, -5, 4,['#t=', num2str(2000/2000*24),'s'],'color', 'k', 'FontSize', 15, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');
temp11=(trace_err1).^2;
 squared_errors1=temp11(317:583);
mse1=sqrt(mean(squared_errors1));
mse_t1=sum(mse1)/length(mse1);
temp22=(trace_err2).^2;
 squared_errors2=temp22(317:583);
mse2=sqrt(mean(squared_errors2));
mse_t2=sum(mse2)/length(mse2);
temp33=(trace_err3).^2;
 squared_errors3=temp33(317:583);
mse3=sqrt(mean(squared_errors3));
mse_t3=sum(mse3)/length(mse3);
 (mse_t1+mse_t2+mse_t3)/3
temp11=(trace_esterr1).^2;
 squared_errors1=temp11(317:583);
mse1=sqrt(mean(squared_errors1));
mse_t1=sum(mse1)/length(mse1);
temp22=(trace_esterr2).^2;
 squared_errors2=temp22(317:583);
mse2=sqrt(mean(squared_errors2));
mse_t2=sum(mse2)/length(mse2);
temp33=(trace_esterr3).^2;
 squared_errors3=temp33(317:583);
mse3=sqrt(mean(squared_errors3));
mse_t3=sum(mse3)/length(mse3);
 (mse_t1+mse_t2+mse_t3)/3
legend('Virtual leader','Robot 1','Robot 2','Robot 3','Obstacles','Location','northEast','Orientation','Vertical');
function[drone,combinedobject] = drone_plot(hg, col)
D2R = pi/180;
scale = 7;
b   = scale * 0.6 ;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3 ;   % the legth of small square base of quadcopter(b/4)
H   = scale * 0.06 ;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4 ;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
    sin(ro) cos(ro)  0;
    0       0       1];     % rotation matrix to rotate the coordinates of base
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base
    -a/2 -a/2 a/2 a/2;
    0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree
to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Design Different parts
% design the base square
drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
drone(2) = patch([base(1,:)],[base(2,:)],base(3,:)+H,'r');

% design 2 parpendiculer legs of quadcopter
[xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ;

% design 4 cylindrical motors
drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');

% design 4 propellers
drone(9)  = patch(xp+b/2,yp,  zp+(H_m+H/2),col,'LineWidth',0.5);
drone(10) = patch(xp-b/2,yp,  zp+(H_m+H/2),col,'LineWidth',0.5);
drone(11) = patch(xp,yp+b/2, zp+(H_m+H/2),col,'LineWidth',0.5);
drone(12) = patch(xp,yp-b/2,  zp+(H_m+H/2),col,'LineWidth',0.5);
combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)
end

function updown = pbsplot(mode,x_p,y_p,z_p,r_p,hgt,col)
if mode=='S'
    [X,Y,Z]=sphere(20);
    xp = r_p*X+x_p;
    yp = r_p*Y+y_p;
    zp = r_p*Z+z_p;
    updown(1) = surf(xp,yp,zp+hgt,'EdgeColor','none',FaceColor=col);
else
    [x_x1,y_y1,z_z1] = cylinder(1);
    updown(1)=surf(r_p*x_x1+x_p,r_p*y_y1+y_p,hgt*z_z1+z_p,'FaceColor',col,'EdgeColor','none');
    to = linspace(0, 2*pi);
    xp = x_p+r_p*cos(to);
    yp = y_p+r_p*sin(to);
    zp = z_p+zeros(1,length(to));
    updown(2)  = patch(xp,yp,  zp,col,'EdgeColor','w');
    updown(3)  = patch(xp,yp,  zp+hgt,col,'EdgeColor','w');
end
end
function p2=Aff(mode, theta, a, b,c, xm, ym,zm, p)
if mode=='S'
    A=[a,0,0;0,b,0;0,0,c];
    B=[0;0;0];
elseif mode=='T'
    A=[1,0,0;0,1,0;0,0,1];
    B=[xm;ym;zm];
elseif mode=='R'
    A=[a,0,0;0,b,0;0,0,c] * [cos(-theta),sin(-theta),0;-sin(-theta),cos(-theta),0;0,0,1];
    B=[0;0;0];
elseif mode=='A'
    A=[a,0,0;0,b,0;0,0,c] * [cos(-theta),sin(-theta),0;-sin(-theta),cos(-theta),0;0,0,1];
    B=[xm;ym;ym];
end
p2=A*p+B;
end
function [v_m, jacobi_m] = PIBC(param, bar_sigma_c , v_expected, s_c1)
[scale_x, ~]  = size(bar_sigma_c );
jacobi_m = eye(scale_x, scale_x);
Phai_c=(2 ./ (param.rho_c .* param.mu_c .* param.T_c))*(2 .* param.mu_c .* bar_sigma_c + return_sign2(bar_sigma_c, ...
    1-param.rho_c) + param.mu_c^2 .* return_sign2(bar_sigma_c, 1+param.rho_c))-(2. / ( param.km_gain .* param.rho_c1 .* ...
    param.mu_c1 .* param.T_c1)) * (2 .* param.mu_c1 .* s_c1 + return_sign2(s_c1, ...
    1-param.rho_c) + param.mu_c1^2 .* return_sign2(s_c1,1+param.rho_c));
v_m = pinv(jacobi_m) * (v_expected + param.km_gain .* (Phai_c));
end
function [v_f, jacobi_f] = PDFBC(param,bar_sigma_f, s_f1,v_expected)
[scale_x, ~]  = size(bar_sigma_f );
jacobi_f = eye(scale_x, scale_x);
Phai_f = (2 ./ (param.rho_f .* param.mu_f .* param.T_f))*(2 .* param.mu_f .* bar_sigma_f + return_sign2(bar_sigma_f, ...
    1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f, 1+param.rho_f))-(2. / ( param.kf_gain .* param.rho_f1 .* ...
    param.mu_f1 .* param.T_f1)) * (2 .* param.mu_f1 .* s_f1 + return_sign2(s_f1, ...
    1-param.rho_f) + param.mu_f1^2 .* return_sign2(s_f1,1+param.rho_f));
v_f =  pinv(jacobi_f) * (v_expected + [param.kf_gain,0,0;0,param.kf_gain,0;0,0,param.kf_gain]* (Phai_f));
end
function [esm_x_dot, esm_v_dot] = retrunaikx(param, esm_p, esm_pv, x0, v0,  agent_id)
[~, cow] =  size(param.A_matrix);
esm_x_dot = zeros(3, 1); %#ok<PREALL>
sumaijx = zeros(3, 1);
sumaijv = zeros(3, 1);
for j = 1:1:cow
    sumaijx = sumaijx  + param.A_matrix(agent_id, j) .* (esm_p(:, agent_id) - esm_p(:, j)) ;
    sumaijv = sumaijv  + param.A_matrix(agent_id, j) .* (esm_pv(:, agent_id) - esm_pv(:, j)) ;
end
ep1 = sumaijx  +param.B_matrix(agent_id).*(esm_p(:, agent_id) - x0);
ev1 = sumaijv  +param.B_matrix(agent_id).*(esm_pv(:, agent_id) - v0);
esm_x_dot =-(5 ./ (param.lamda_PH .* param.rho_epv .* param.mu_epv .* param.T_epv))*(2 .* param.mu_epv ...
    .* ep1 + return_sign2(ep1, 1-param.rho_epv) + param.mu_epv^2 .* return_sign2(ep1, 1+ ...
    param.rho_epv)) - param.lo1 .*sign(ep1) +esm_pv(:, agent_id);
esm_v_dot =-(10 ./ (param.lamda_PH .* param.rho_epv .* param.mu_epv .* param.T_epv))*(2 .* param.mu_epv ...
    .* ev1 + return_sign2(ev1, 1-param.rho_epv) + param.mu_epv^2 .* return_sign2(ev1, 1+ ...
    param.rho_epv)) - param.lo2 .*sign(ev1) ;
end
function [sign_r1] = return_sign2(e1, r1)
[scale_e, ~] = size(e1);
if scale_e  == 2
    sign_r1= [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2))]';
elseif scale_e  == 3
    sign_r1 = [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2)), ((abs(e1(3)))^ r1) * sign(e1(3))]';
else
    sign_r1 = ((abs(e1))^ r1) * sign(e1);
end
end
function [bar_sigma_f, sigmaf_i, pf_i, vf_i] = return_bar_sigmaf(param, p_now, v_now, p_esm, pv_esm, sigma_x, sigma_v, agent_id)
[~, cow] =  size(param.A_matrix);
sum_aijpj = zeros(3, 1);
sumv_aijpj = zeros(3, 1);
sumaij = 0;
for i = 1:1:cow
    sum_aijpj = sum_aijpj   + param.A_matrix(agent_id, i) .* (p_now(:, i) -  sigma_x(:, i) + sigma_x(:, agent_id)) ;% 可以改为测量值
    sumv_aijpj = sumv_aijpj   + param.A_matrix(agent_id, i) .* (v_now(:, i) -  sigma_v(:, i) + sigma_v(:, agent_id)) ;% 可以改为测量值
    sumaij = sumaij  + param.A_matrix(agent_id, i);
end
pf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (p_esm(:, agent_id)) + param.k2 .* sum_aijpj);
vf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (pv_esm(:, agent_id)) + param.k2 .* sumv_aijpj);
sigmaf_i = pf_i + sigma_x(:,agent_id);
bar_sigma_f = sigmaf_i-p_now(:,agent_id);
end
function [id,pmin_id, min_dis] = returnnearobstacle(p, obs_detect, agent_id)
[~,a_num] = size(obs_detect);
p_id = p(:, agent_id);
min_id  = 1;
min_dis = norm(p_id - obs_detect(:, min_id));
for i = 1:1:a_num
    if i~= min_id
        if norm(p_id - obs_detect(:, i)) <= min_dis
            min_dis = norm(p_id - obs_detect(:, i));
            min_id = i;
        end
    end
end
pmin_id = obs_detect(:, min_id);
id=min_id;
end
function [smrho_oa, jacobi_oa] = returnobstaclejacobi(p, R, r, pmin_obid, min_obdis, agent_id)
if  min_obdis ~= inf
    smrho_oa =  R - R*sin((min_obdis + R - 2*r)*pi/(2*(R-1*r)));
    jacobi_oa = (-R*pi*(p(:, agent_id) - pmin_obid)'/(2*(R-1*r)*min_obdis))...
        *cos(pi*(min_obdis + R -2*r)/(2 * ( R - 1*r )));
else
    smrho_oa  = inf;
    jacobi_oa= zeros(1, 2);
end
if isnan(smrho_oa)
    smrho_oa  = inf;
    jacobi_oa= zeros(1, 2);
end
end
function [pmin_id, min_dis] = returnnearagent(p, agent_id)
[~,a_num] = size(p);
p_id = p(:, agent_id);
min_id  = agent_id;
min_dis  = inf;
for i = 1:1:a_num
    if i~= agent_id
        if norm(p(:, i) - p_id) <= min_dis
            min_dis = norm(p(:, i) - p_id);
            min_id = i;
        end
    end
end
pmin_id = p(:, min_id);
end
function [sign_r1] = return_sign(e1, r1)
sign_r1 = [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2))]';
end
function [x1_dot, x2_dot] = dynamic_model(u, x1, x2, t1)
f_2=0.1.*[1+cos(t1) * x1(1)+2 * sin(2*t1) * x2(1)+x1(1)^2 ;
    1+sin(t1) * x1(2)+2 * sin(2*t1) * x2(2)+x1(2)^2;
    1+sin(t1) * x1(3)+2 * sin(2*t1) * x2(3)+x1(3)^2];
x1_dot = x2;
f_1=0.5*[-0.5*x1(1)+x1(2)*(1+0.5*x1(2)^2)+cos(x2(1));-0.8*(x1(1)+x1(2))+0.5*x1(2)*(1-0.3*x1(2)^2)+sin(x2(2));-0.8*(x1(1)+x1(2))+0.5*x1(3)*(1-0.3*x1(2)^2)+sin(x2(3))];
B = eye(3,3);
x2_dot =B * u+f_1+f_2;
end
function v_term = ctr_term(ctr_param, e_rr, e_dot, z2_esm)
rho = ctr_param.rho;
cmu_p = ctr_param.cmu_p;
T = ctr_param.T;
gama = ctr_param.gama;
phai_function = (2 /(rho * cmu_p * T )) * ( 2 * cmu_p*(norm(e_rr)) + (norm(e_rr)).^(-rho) +cmu_p^2 * (norm(e_rr)).^(rho)) * e_rr;
phai_function_dot = (2 /(rho * cmu_p * T )) * ( 2 * cmu_p*(norm(e_rr)) + (norm(e_rr)).^(-rho) +cmu_p^2 * (norm(e_rr)).^(rho)) * e_dot;
s = e_dot + phai_function;
v_term = - (2 /(rho * cmu_p * T )) * (cmu_p +  (0.5 ^ (1-rho/2)) * ((norm(s))^(-rho)) + ...
    cmu_p^2 * (0.5 ^ (1+rho/2)) * ((norm(s))^(rho))) * s - z2_esm - phai_function_dot - (1 / 2) * (1 / (gama ^ 2)) *s ;
end