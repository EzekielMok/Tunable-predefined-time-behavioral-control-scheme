% Ezekiel
% 2024/10/10
clc,clear
close all
sim.t_total = 20;%s
sim.point_num=2000;
cx = zeros(sim.point_num,1);
cy= zeros(sim.point_num,1);
x0 = @(t_step) 50*sin(t_step );
y0 = @(t_step) 40*cos(t_step);
start_psi=-0.9*pi;
end_psi=0.45*pi;
k=0;
for theta=start_psi:(end_psi-start_psi)/(sim.point_num-1):end_psi
    k=k+1;
    cx(k) = x0(theta);
    cy(k) = y0(theta);
end
figure(1)
plot(cx, cy, Color='m',LineStyle='--',LineWidth=2)
axis equal
grid on
refer_path(:, 1:2) =[cx, cy];
for i=1:length(refer_path)
    if i==1
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        ddx = refer_path(3, 1) + refer_path(1, 1) - 2 * refer_path(2, 1);
        ddy = refer_path(3, 2) + refer_path(1, 2) - 2 * refer_path(2, 2);
    elseif  i==length(refer_path)
        dx = refer_path(i, 1) - refer_path(i - 1, 1);
        dy = refer_path(i, 2) - refer_path(i - 1, 2);
        ddx = refer_path(i, 1) + refer_path(i - 2, 1) - 2 * refer_path(i - 1, 1);
        ddy = refer_path(i, 2) + refer_path(i - 2, 2) - 2 * refer_path(i - 1, 2);
    else
        dx = refer_path(i + 1, 1) - refer_path(i, 1);
        dy = refer_path(i + 1, 2) - refer_path(i, 2);
        ddx = refer_path(i + 1, 1) + refer_path(i - 1, 1) - 2 * refer_path(i, 1);
        ddy = refer_path(i + 1, 2) + refer_path(i - 1, 2) - 2 * refer_path(i, 2);
    end
    refer_path(i,3)=atan2(dy, dx);%
    refer_path(i,4)=(ddy * dx - ddx * dy) / ((dx ^ 2 + dy ^ 2) ^ (3 / 2));
    refer_path(i,5) = dx;
    refer_path(i,6) = dy;
end
for i=1:length(refer_path)
    if i==1
        dphai = refer_path(i + 1, 3) - refer_path(i, 3);
    elseif  i==length(refer_path)
        dphai = refer_path(i, 3) - refer_path(i - 1, 3);
    else
        dphai = refer_path(i + 1, 3) - refer_path(i, 3);
    end
    refer_path(i, 7) = dphai;%yaw
end
x_d = refer_path(:, 1);
y_d = refer_path(:, 2);
phai_d = refer_path(:, 3);
dx_d = refer_path(:, 5);
dy_d = refer_path(:, 6);
dphai_d = refer_path(:, 7);
sim.list_x_rd = refer_path(:, 1:3)';
sim.list_v_rd =refer_path(:, 5:7)';
Config.ang_str = deg2rad([360/2.5, 360/2.5 + 360/5, 360/2.5 + 2 * 360/5, 360/2.5 + 3 * 360/5, 360/2.5 + 4 * 360/5]);
Config.len_str =1.5.*2.5.* [sqrt(3), sqrt(3), sqrt(3), sqrt(3), sqrt(3)];
Config1.ang_str = deg2rad([360/2.5, 360/2.5 + 360/5, 360/2.5 + 2 * 360/5, 360/2.5 + 3 * 360/5, 360/2.5 + 4 * 360/5]);
Config1.len_str =3.*2.5.* [1.8, 3, 2.5, 3, 3.5];
st1=[Config1.len_str(1) * cos(Config1.ang_str(1));Config1.len_str(1) * sin(Config1.ang_str(1));0];
st2=[Config1.len_str(2) * cos(Config1.ang_str(2));Config1.len_str(2) * sin(Config1.ang_str(2));0];
st3=[Config1.len_str(3) * cos(Config1.ang_str(3));Config1.len_str(3) * sin(Config1.ang_str(3));0];
st4=[Config1.len_str(4) * cos(Config1.ang_str(4));Config1.len_str(4) * sin(Config1.ang_str(4));0];
st5=[Config1.len_str(5) * cos(Config1.ang_str(5));Config1.len_str(5) * sin(Config1.ang_str(5));0];
[trans_matrix, trans_matrix_dot] = return_T_phai(sim.list_x_rd(3,1), sim.list_v_rd(3,1));
sim.list_x_rd = refer_path(:, 1:3)';
sim.list_v_rd =refer_path(:, 5:7)';
sim.x_0 =sim.list_x_rd(:,1)+[0,0,0]';% 可以看做领航者的位置
sim.v_0 =  [0; 0; 0];%
sim.x_10 =sim.list_x_rd(:,1)+trans_matrix*st1+[25,-8,0]';%
sim.v_10 = [0; 0; 0];%
sim.x_20 =sim.list_x_rd(:,1)+trans_matrix*st2+[25,-5,0]';%
sim.v_20 =  [0; 0; 0];%
sim.x_30 =sim.list_x_rd(:,1)+trans_matrix*st3+[30,-2,0]';%
sim.v_30 =  [0; 0; 0];%
sim.x_40 =sim.list_x_rd(:,1)+trans_matrix*st4+[37.5,-10,0]';%
sim.v_40 = [0; 0; 0];%
sim.x_50 =sim.list_x_rd(:,1)+trans_matrix*st5+[25,0,0]';%
sim.v_50 =  [0; 0; 0];%
sim.x_10(3) =pi;%
sim.x_20(3) =pi;%
sim.x_30(3) =pi;%
sim.x_40(3) =pi;%
sim.x_50(3) =pi;%
sim.ts = sim.t_total / sim.point_num;
sim.t=linspace(0, sim.t_total, sim.point_num);
%% 避障距离和侦测距离
R_safe =12;
r_safe=5;
sim.detected_field1 = 40;
sim.detected_field2 = 10;
%% 实体化智能体类
agent0 = agentclass2dim(sim.x_0, sim.v_0, sim.list_v_rd, sim.list_x_rd, R_safe, 0.1);% 虚拟领航者
agent1 = agentclass2dim(sim.x_10, sim.v_10, sim.list_v_rd, sim.list_x_rd, r_safe, 1);
agent2 = agentclass2dim(sim.x_20, sim.v_20, sim.list_v_rd, sim.list_x_rd, r_safe, 1);
agent3 = agentclass2dim(sim.x_30, sim.v_30, sim.list_v_rd, sim.list_x_rd, r_safe, 1);
agent4 = agentclass2dim(sim.x_40, sim.v_40, sim.list_v_rd, sim.list_x_rd, r_safe, 1);
agent5 = agentclass2dim(sim.x_50, sim.v_50, sim.list_v_rd, sim.list_x_rd, r_safe, 1);
param.km_gain = 3.5;
param.rho_c = 0.5;
param.rho_c1=0.5;
param.mu_c = 0.6;
param.mu_c1 = 0.6;
param.T_c = 6;
param.T_c1 = 6;
param.A_matrix = [0, 0, 0, 0.4, 0;
    0, 0, 0.5,0,   0;
    0, 0.25, 0,0,0.35;
    0.2, 0, 0,0,0.2;
    0, 0, 0.7,0.4,0];%有向生成树图
param.B_matrix = [1, 0, 0, 0, 0]';%表征与领航者的直接通讯关系
param.K_matrix = diag([sum(param.A_matrix(1,:)),sum(param.A_matrix(2,:)), ...
    sum(param.A_matrix(3,:)),sum(param.A_matrix(4,:)),sum(param.A_matrix(5,:))]);
param.L_matrix = param.K_matrix - param.A_matrix;
param.H_matrix = param.L_matrix + param.B_matrix;
param.P_matrix =diag([0.5,0.5,1,1,0.5]);
param.lamda_PH=min(eig(param.P_matrix * param.H_matrix));
param.rho_epv = 0.5;
param.mu_epv= 0.9;
param.T_epv = 4;
param.lo1 = 0.5;
param.lo2 = 0.2;
param.kf_gain = 8;
param.kangle_gain=0.85;
param.rho_f = 0.5;
param.rho_f1=0.5;
param.mu_f = 0.6;
param.mu_f1 = 0.6;
param.T_f = 15;
param.T_f1 = 15;
%% 编队控制权重
param.k1 =1;
param.k2 = 1-param.k1;
param.alpha=12;
param.beta =12;
param.alpha1=5;
param.beta1 =5;
param.q =2;
param.ka_gain =1.2;
param.rho_a = 0.5;
param.rho_a1=0.5;
param.mu_a = 0.6;
param.mu_a1 = 0.6;
param.T_a = 4;
param.T_a1 = 4;
x1_esm=agent0.x+[2.0,0.8,0.2]';
x2_esm=agent0.x+[-0.9,-1.5,-0.2]';
x3_esm=agent0.x+[0.9,0.8,0.2]';
x4_esm=agent0.x+[-0.9,-1.2,-0.3]';
x5_esm=agent0.x+[1.8,0.8,0.3]';
v1_esm = agent0.v;
v2_esm = agent0.v;
v3_esm = agent0.v;
v4_esm=agent0.v;
v5_esm=agent0.v;
p_esm = [x1_esm, x2_esm, x3_esm, x4_esm, x5_esm];
pv_esm = [v1_esm, v2_esm, v3_esm, v4_esm, v5_esm];
int_c = [0,0,0]';
int_f1 = [0,0]';
int_f2 = [0,0]';
int_f3 = [0,0]';
int_f4 = [0,0]';
int_f5 = [0,0]';
int_a = [0,0]';
int_a1 = [0,0]';
int_a2 = [0,0]';
int_a3 = [0,0]';
int_a4 = [0,0]';
int_a5 = [0,0]';
% Ezekiel 20231205
%% 麦克娜姆轮动力学
R = 0.0375;
m = 4;
J_z = 0.1;
J_w = 0.2;
l_a = 0.32;
l_b = 0.2;
D_theta = 0.2;
G = 9.8;
J =[1, -1, -(l_a + l_b);
    1,  1, (l_a + l_b);
    1,  1, -(l_a + l_b);
    1, -1, (l_a + l_b)];
J_inv = pinv(J);
A_j = (1 / 8) * (m * R .^ 2);
B_j = (1 / 16) *(J_z * R .^ 2) / ((l_a  +  l_b) .^ 2);
M = [A_j + B_j + J_w,                     -B_j,                       B_j,              A_j - B_j;
    -B_j,        A_j + B_j + J_w,           A_j - B_j,                 B_j;
    B_j,                  A_j - B_j,           A_j + B_j + J_w,       -B_j;
    A_j - B_j,                    B_j,                 -B_j,                  A_j + B_j + J_w];
% 估计器参数
mu1 = 0.6;
mu2 = 0.2;
r1= 0.7;
r2= 0.7;
ctr_param.rho=0.5;
ctr_param.T = 4;
ctr_param.gama = 1.5;
ctr_param.cmu_p = 0.1;
n=length(refer_path);
k = 0;
z_10 = [0.01, 0.01, 0.01 ]';
z_20 = [0.01, 0.01, 0.01 ]';
z1_esm = z_10;
z2_esm = z_20;
z1_esm2 = z_10;
z2_esm2 = z_20;
z1_esm3 = z_10;
z2_esm3 = z_20;
z1_esm4 = z_10;
z2_esm4 = z_20;
z1_esm5 = z_10;
z2_esm5 = z_20;
z1 = z_10;
z2 = z_20;
z12 = z_10;
z22 = z_20;
z13 = z_10;
z23 = z_20;
z14 = z_10;
z24 = z_20;
z15 = z_10;
z25 = z_20;
tao = [0, 0, 0, 0]';
tao2 = [0, 0, 0, 0]';
tao3 = [0, 0, 0, 0]';
tao4 = [0, 0, 0, 0]';
tao5 = [0, 0, 0, 0]';
%% 扰动输入
H_d = @(t_step) (10.^(-2)) .* [1 + 2 * cos(0.3  *  t_step);
    2 + 1.5 * sin(0.3 * t_step) +  3 * cos(0.3 * t_step);
    3 + 2 * sin(0.2 *  t_step);
    2.5 + 1 * sin(0.2 *  t_step) ];
F_f = [0, 0, 0, 0]';
x0 = agent1.x;
x1 = agent1.v;
x02 = agent2.x;
x12 = agent2.v;
x03 = agent3.x;
x13 = agent3.v;
x04 = agent4.x;
x14 = agent4.v;
x05 = agent5.x;
x15 = agent5.v;
%% 障碍物属性
Ox0 = @(t_step) -50+t_step-14;
Oy0=  @(t_step) 16+4*sin(0.8*t_step);
Ox0_dot = @(t_step) 1;
Oy0_dot=  @(t_step) 4*0.8*cos(0.8*t_step);
Ox20 = @(t_step) 20;
Oy20=  @(t_step) 20;
Ox20_dot = @(t_step) 0;
Oy20_dot=  @(t_step) 0;
Ox10 = @(t_step) -50+t_step-14;
Oy10=  @(t_step) 22+4*sin(0.8*t_step);
Ox10_dot = @(t_step) 1;
Oy10_dot=  @(t_step) 4*0.8*cos(0.8*t_step);
Ox30 = @(t_step) 14;
Oy30=  @(t_step) -32;
Ox30_dot = @(t_step) 0;
Oy30_dot=  @(t_step) 0;
Ox40 = @(t_step) -15;
Oy40=  @(t_step) -50;
Ox40_dot = @(t_step) 0;
Oy40_dot=  @(t_step) 0;
Ox50 = @(t_step) 100;
Oy50=  @(t_step) 15;
Ox50_dot = @(t_step) 0;
Oy50_dot=  @(t_step) 0;
trace_x0 = nan+[sim.t;sim.t;sim.t];
trace_x1  = nan+[sim.t;sim.t;sim.t];
trace_x0_d  = nan+[sim.t;sim.t;sim.t];
trace_x1_d  = nan+[sim.t;sim.t;sim.t];
trace_s_c = nan+[sim.t;sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_f1 = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_f2 = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_f3 = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_f4 = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_f5 = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_s_a = nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_p_esm = nan+[sim.t; sim.t;sim.t; sim.t;sim.t; sim.t;sim.t; sim.t;sim.t; sim.t];
trace_O0= nan+[sim.t;sim.t];% 任务跟踪误差滑模面
trace_detected =nan+[sim.t];
trace_ang1 =nan+[sim.t];
trace_ang2 =nan+[sim.t];
trace_ang3 =nan+[sim.t];
trace_ang4 =nan+[sim.t];
trace_ang5 =nan+[sim.t];
trace_ang_desire =nan+[sim.t];
trace_angest =nan+[sim.t];
trace_barsigma_0=nan+[sim.t;sim.t;sim.t];
trace_obs= nan+[sim.t;sim.t];
trace_obs2= nan+[sim.t;sim.t];
trace_obsdistance= nan+[sim.t];
trace_obsdistance1= nan+[sim.t];
trace_obsdistance2= nan+[sim.t];
trace_obsdistance3= nan+[sim.t];
trace_obsdistance4= nan+[sim.t];
trace_obsdistance5= nan+[sim.t];
trace_esterror1=nan+[sim.t];
trace_esterror2=nan+[sim.t];
trace_esterror3=nan+[sim.t];
trace_esterror4=nan+[sim.t];
trace_esterror5=nan+[sim.t];
trace_taskerror1=nan+[sim.t];
trace_taskerror2=nan+[sim.t];
trace_taskerror3=nan+[sim.t];
trace_taskerror4=nan+[sim.t];
trace_taskerror5=nan+[sim.t];
trace_trackingerror1=nan+[sim.t];
trace_trackingerror2=nan+[sim.t];
trace_trackingerror3=nan+[sim.t];
trace_trackingerror4=nan+[sim.t];
trace_trackingerror5=nan+[sim.t];
trace_u1 = nan+[sim.t];
trace_u2 = nan+[sim.t];
trace_u3 = nan+[sim.t];
trace_u4 = nan+[sim.t];
trace_u5 = nan+[sim.t];
trace_1=nan+[sim.t;sim.t;sim.t;sim.t;sim.t];
trace_2=nan+[sim.t;sim.t;sim.t;sim.t;sim.t];
%% 主循环
for j = sim.t
    if j<sim.t_total/6
        param.k1 =1;
        param.T_f = 15;
        param.T_f1 = 15;
    else
        param.k1 =0.5;
        param.T_f = 7;
        param.T_f1 = 7;
    end
    k=k+1;
    %% 记录状态信息
    % 智能体0的轨迹
    agent0.record_x(end+1) = agent0.x(1);
    agent0.record_y(end+1) = agent0.x(2);
    agent0.record_psi(end+1) = agent0.x(3);
    % 智能体1的轨迹
    agent1.record_x(end+1) = agent1.x(1);
    agent1.record_y(end+1) = agent1.x(2);
    agent1.record_psi(end+1) = agent1.x(3);
    % 智能体2的轨迹
    agent2.record_x(end+1) = agent2.x(1);
    agent2.record_y(end+1) = agent2.x(2);
    agent2.record_psi(end+1) = agent2.x(3);
    % 智能体3的轨迹
    agent3.record_x(end+1) = agent3.x(1);
    agent3.record_y(end+1) = agent3.x(2);
    agent3.record_psi(end+1) = agent3.x(3);
    % 智能体4的轨迹
    agent4.record_x(end+1) = agent4.x(1);
    agent4.record_y(end+1) = agent4.x(2);
    agent4.record_psi(end+1) = agent4.x(3);
    % 智能体5的轨迹
    agent5.record_x(end+1) = agent5.x(1);
    agent5.record_y(end+1) = agent5.x(2);
    agent5.record_psi(end+1) = agent5.x(3);
    trace_O0(:, k) = [Ox0(j); Oy0(j)];
    phai=0;
    trans_matrixo=[cos(phai), -sin(phai);
    sin(phai), cos(phai)];
    sim.obstacle_x =trans_matrixo * [Ox0(j),Ox10(j); Oy0(j),Oy10(j)];
    sim.obstacle_v =[Ox0_dot(j),Ox10_dot(j); Oy0_dot(j),Oy10_dot(j)];
    sim.obstacle_x1 =[Ox20(j),Ox30(j),Ox40(j),Ox50(j); Oy20(j),Oy30(j),Oy40(j),Oy50(j)];
    sim.obstacle_v1 =[Ox20_dot(j),Ox30_dot(j),Ox40_dot(j),Ox50_dot(j); Oy20_dot(j),Oy30_dot(j),Oy40_dot(j),Oy50_dot(j)];
    % 障碍物检测
    [min_id, min_dis_detected,  criteria_detected, criteria_safe , obs_detected,  obs_safe]...
        = returnnearobstacle(agent0.x, sim.obstacle_x, sim.detected_field1,  agent0.R_safe);
    [min_id1, min_dis_detected1,  criteria_detected1, criteria_safe1 , obs_detected1,  obs_safe1]...
        = returnnearobstacle(agent1.x, sim.obstacle_x1, sim.detected_field2,  agent1.R_safe);
    [min_id2, min_dis_detected2,  criteria_detected2, criteria_safe2 , obs_detected2,  obs_safe2]...
        = returnnearobstacle(agent2.x, sim.obstacle_x1, sim.detected_field2,  agent2.R_safe);
    [min_id3, min_dis_detected3,  criteria_detected3, criteria_safe3 , obs_detected3,  obs_safe3]...
        = returnnearobstacle(agent3.x, sim.obstacle_x1, sim.detected_field2,  agent3.R_safe);
    [min_id4, min_dis_detected4,  criteria_detected4, criteria_safe4 , obs_detected4,  obs_safe4]...
        = returnnearobstacle(agent4.x, sim.obstacle_x1, sim.detected_field2,  agent4.R_safe);
    [min_id5, min_dis_detected5,  criteria_detected5, criteria_safe5, obs_detected5,  obs_safe5]...
        = returnnearobstacle(agent5.x, sim.obstacle_x1, sim.detected_field2,  agent5.R_safe);
    trace_obsdistance(k) =norm(agent0.x(1:2)-sim.obstacle_x);
    if ~isempty(criteria_safe)
        trace_detected(k) = 1;
        [det_num, bar_sigma_a,  jacobi_o, jacobi_ow, dsigma_a0, augmentobsv] =...
            dynamicobsavoid(param, agent0.x, agent0.v, sim.list_x_rd(:, k) , sim.obstacle_x, sim.obstacle_v);
        int_a_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a + return_sign2(bar_sigma_a, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a, 1+param.rho_a));
        int_a = int_a + int_a_dot * sim.ts;
        s_a0 = bar_sigma_a + int_a;
        v_o = PCABC(param, bar_sigma_a, s_a0, jacobi_o, jacobi_ow, augmentobsv,det_num);
    else
        int_a = [0;0];
        jacobi_o = [0,0,0;0,0,0];
        jacobi_ow = [0,0;0,0];
        v_o = [0,0,0]';
        trace_detected(k) = 0;
    end
    if ~isempty(criteria_safe1)
        [det_num1, bar_sigma_a1,  jacobi_o1, jacobi_ow1, dsigma_a1, augmentobsv1] =...
            dynamicobsavoid(param, agent1.x, agent1.v, p_esm(:, 1)+ sigma_p(:,1) , sim.obstacle_x1, sim.obstacle_v1);
        int_a1_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a1 + return_sign2(bar_sigma_a1, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a1, 1+param.rho_a));
        int_a1 = int_a1 + int_a1_dot * sim.ts;
        s_a1 = bar_sigma_a1 + int_a1;
        v_o1 = PCABC(param, bar_sigma_a1, s_a1, jacobi_o1, jacobi_ow1, augmentobsv1,det_num1);
    else
        int_a1 = [0;0];
        jacobi_o1 = [0,0,0;0,0,0];
        jacobi_ow1 = [0,0;0,0];
        v_o1 = [0,0,0]';
    end
    if ~isempty(criteria_safe2)
        [det_num2, bar_sigma_a2,  jacobi_o2, jacobi_ow2, dsigma_a2, augmentobsv2] =...
            dynamicobsavoid(param, agent2.x, agent2.v, p_esm(:, 2)+ sigma_p(:,2), sim.obstacle_x1, sim.obstacle_v1);
        int_a2_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a2 + return_sign2(bar_sigma_a2, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a2, 1+param.rho_a));
        int_a2 = int_a2 + int_a2_dot * sim.ts;
        s_a2 = bar_sigma_a2 + int_a2;
        v_o2 = PCABC(param, bar_sigma_a2, s_a2, jacobi_o2, jacobi_ow2, augmentobsv2,det_num2);
    else
        int_a2 = [0;0];
        jacobi_o2 = [0,0,0;0,0,0];
        jacobi_ow2 = [0,0;0,0];
        v_o2 = [0,0,0]';
    end
    if ~isempty(criteria_safe3)
        [det_num3, bar_sigma_a3,  jacobi_o3, jacobi_ow3, dsigma_a3, augmentobsv3] =...
            dynamicobsavoid(param, agent3.x, agent3.v, p_esm(:, 3)+ sigma_p(:,3), sim.obstacle_x1, sim.obstacle_v1);
        int_a3_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a3 + return_sign2(bar_sigma_a3, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a3, 1+param.rho_a));
        int_a3 = int_a3 + int_a3_dot * sim.ts;
        s_a3 = bar_sigma_a3 + int_a3;
        v_o3 = PCABC(param, bar_sigma_a3, s_a3, jacobi_o3, jacobi_ow3, augmentobsv3,det_num3);
    else
        int_a3 = [0;0];
        jacobi_o3 = [0,0,0;0,0,0];
        jacobi_ow3 = [0,0;0,0];
        v_o3 = [0,0,0]';
    end
    if ~isempty(criteria_safe4)
        [det_num4, bar_sigma_a4,  jacobi_o4, jacobi_ow4, dsigma_a4, augmentobsv4] =...
            dynamicobsavoid(param, agent4.x, agent4.v, p_esm(:, 4)+ sigma_p(:,4), sim.obstacle_x1, sim.obstacle_v1);
        int_a2_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a4 + return_sign2(bar_sigma_a4, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a4, 1+param.rho_a));
        int_a4 = int_a4 + int_a4_dot * sim.ts;
        s_a4 = bar_sigma_a4 + int_a4;
        v_o4 = PCABC(param, bar_sigma_a4, s_a4, jacobi_o4, jacobi_ow4, augmentobsv4, det_num4);
    else
        int_a4 = [0;0];
        jacobi_o4 = [0,0,0;0,0,0];
        jacobi_ow4 = [0,0;0,0];
        v_o4 = [0,0,0]';
    end
    if ~isempty(criteria_safe5)
        [det_num5, bar_sigma_a5,  jacobi_o5, jacobi_ow5, dsigma_a5, augmentobsv5] =...
            dynamicobsavoid(param, agent5.x, agent5.v, p_esm(:, 5)+ sigma_p(:,5), sim.obstacle_x1, sim.obstacle_v1);
        int_a5_dot = -(2 ./ (param.rho_a .* param.mu_a .* param.T_a)) * inv([param.ka_gain, 0; 0,  param.kangle_gain]) * (2 .* ...
            param.mu_a .* bar_sigma_a5 + return_sign2(bar_sigma_a5, ...
            1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a5, 1+param.rho_a));
        int_a5 = int_a5 + int_a5_dot * sim.ts;
        s_a5 = bar_sigma_a5 + int_a5;
        v_o5 = PCABC(param, bar_sigma_a5, s_a5, jacobi_o5, jacobi_ow5, augmentobsv5,det_num5);
    else
        int_a5 = [0;0];
        jacobi_o5 = [0,0,0;0,0,0];
        jacobi_ow5 = [0,0;0,0];
        v_o5 = [0,0,0]';
    end
    % 虚拟领航者的PIC行为
    bar_sigma_c = sim.list_x_rd(:, k) - agent0.x;
    trace_barsigma_0(:,k)=bar_sigma_c;
    int_c_dot = -(2 ./ ( param.km_gain .* param.rho_c .* param.mu_c .* param.T_c)).*(2 .* param.mu_c .* bar_sigma_c + return_sign2(bar_sigma_c, ...
        1-param.rho_c) + param.mu_c^2 .* return_sign2(bar_sigma_c, 1+param.rho_c));
    int_c = int_c + int_c_dot * sim.ts;
    s_c1 = bar_sigma_c + int_c;
    trace_s_c(:, k) = s_c1;
    [v_m0, jacobi_m0] = PIBC(param, bar_sigma_c, sim.list_v_rd(:, k) ./ sim.ts, s_c1);
    if  min_dis_detected <  agent0.R_safe
        agent0.v =  v_o + (eye(3, 3) - pinv(jacobi_o) * jacobi_o) *v_m0;
    else
        agent0.v =  v_m0 + (eye(3, 3) - pinv(jacobi_m0) * jacobi_m0) * v_o;
    end
    agent0.x =  agent0.x  + sim.ts .* agent0.v;
    [x1_esm_dot, v1_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v, 1);
    [x2_esm_dot, v2_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v, 2);
    [x3_esm_dot, v3_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v, 3);
    [x4_esm_dot, v4_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v, 4);
    [x5_esm_dot, v5_esm_dot] = retrunaikx(param, p_esm, pv_esm, agent0.x, agent0.v, 5);
    trace_esterror1(k) = norm(agent0.x -p_esm(:,1));
    trace_esterror2(k)  = norm(agent0.x -p_esm(:,2));
    trace_esterror3(k)  = norm(agent0.x -p_esm(:,3));
    trace_esterror4(k)  = norm(agent0.x -p_esm(:,4));
    trace_esterror5(k)  = norm(agent0.x -p_esm(:,5));
    p_esm_dot = [x1_esm_dot, x2_esm_dot, x3_esm_dot, x4_esm_dot, x5_esm_dot];
    pv_esm_dot= [v1_esm_dot, v2_esm_dot, v3_esm_dot, v4_esm_dot, v5_esm_dot];
    p_esm = p_esm + sim.ts .* p_esm_dot;
    pv_esm = pv_esm + sim.ts .* pv_esm_dot;
    % 各智能体对领航者的估计器
    sp1=[Config.len_str(1) * cos(Config.ang_str(1));Config.len_str(1) * sin(Config.ang_str(1));0];
    sp2=[Config.len_str(2) * cos(Config.ang_str(2));Config.len_str(2) * sin(Config.ang_str(2));0];
    sp3=[Config.len_str(3) * cos(Config.ang_str(3));Config.len_str(3) * sin(Config.ang_str(3));0];
    sp4=[Config.len_str(4) * cos(Config.ang_str(4));Config.len_str(4) * sin(Config.ang_str(4));0];
    sp5=[Config.len_str(5) * cos(Config.ang_str(5));Config.len_str(5) * sin(Config.ang_str(5));0];
    [trans_matrix, trans_matrix_dot] = return_T_phai(agent0.x(3), agent0.v(3));
    spv1=[0;0;0];
    sigma_p =trans_matrix * [sp1,sp2,sp3,sp4,sp5];
    sigma_v =trans_matrix_dot * [sp1,sp2,sp3,sp4,sp5];
    % 实际智能体1编队行为
    p_now = [x0, x02, x03,x04, x05];
    v_now = [x1, x12, x12, x14, x15];
    [bar_sigma_f1, sigmaf_i1, pf_i1, vf_i1] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm, sigma_p, sigma_v, 1);
    int_f1_dot = -(2 ./ (param.rho_f .* param.mu_f .* param.T_f)) * inv([param.kf_gain, 0; 0,  param.kangle_gain]) *  (2 .* param.mu_f ...
        .* bar_sigma_f1 + return_sign2(bar_sigma_f1, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f1, 1+param.rho_f));
    int_f1 = int_f1 + int_f1_dot * sim.ts;
    s_f1 = bar_sigma_f1 + int_f1;
    trace_s_f1(:, k) = s_f1;
    [v_fm1, jacobi_fm1] = PDFBC(param, p_now, bar_sigma_f1, pf_i1, vf_i1, sigma_p, s_f1, 1);
    if  min_dis_detected1 <  agent1.R_safe
        agent1.v =  v_o1 + (eye(3, 3) - pinv(jacobi_o1) * jacobi_o1) *v_fm1;
    else
        agent1.v =  v_fm1 + (eye(3, 3) - pinv(jacobi_fm1) * jacobi_fm1) * v_o1;
    end
    agent1.x =  x0  + sim.ts .* agent1.v;
% 实际智能体2编队行为
    [bar_sigma_f2, sigmaf_i2, pf_i2, vf_i2] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p, sigma_v, 2);
    int_f2_dot = -(2 ./ (param.rho_f .* param.mu_f .* param.T_f))* inv([param.kf_gain, 0; 0,  param.kangle_gain]) * (2 .* param.mu_f ...
        .* bar_sigma_f2 + return_sign2(bar_sigma_f2, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f2, 1+param.rho_f));
    int_f2 = int_f2 + int_f2_dot * sim.ts;
    s_f2 = bar_sigma_f2 + int_f2;
    trace_s_f2(:, k) = s_f2;
    [v_fm2, jacobi_fm2] = PDFBC(param, p_now, bar_sigma_f2, pf_i2, vf_i2, sigma_p, s_f2, 2);
    if  min_dis_detected2 <  agent2.R_safe
        agent2.v =  v_o2 + (eye(3, 3) - pinv(jacobi_o2) * jacobi_o2) *v_fm2;
    else
        agent2.v =  v_fm2 + (eye(3, 3) - pinv(jacobi_fm2) * jacobi_fm2) * v_o2;
    end
    agent2.x =  x02  + sim.ts .* agent2.v;
    % 实际智能体3编队行为
    [bar_sigma_f3, sigmaf_i3, pf_i3, vf_i3] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p, sigma_v, 3);
    int_f3_dot = -(2 ./ ( param.rho_f .* param.mu_f .* param.T_f))* inv([param.kf_gain, 0; 0,  param.kangle_gain]) * (2 .* param.mu_f ...
        .* bar_sigma_f3 + return_sign2(bar_sigma_f3, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f3, 1+param.rho_f));
    int_f3 = int_f3 + int_f3_dot * sim.ts;
    s_f3 = bar_sigma_f3 + int_f3;
    trace_s_f3(:, k) = s_f3;
    [v_fm3, jacobi_fm3] = PDFBC(param, p_now, bar_sigma_f3, pf_i3, vf_i3, sigma_p, s_f3, 3);
    if  min_dis_detected3 <  agent3.R_safe
        agent3.v =  v_o3 + (eye(3, 3) - pinv(jacobi_o3) * jacobi_o3) *v_fm3;
    else
        agent3.v =  v_fm3 + (eye(3, 3) - pinv(jacobi_fm3) * jacobi_fm3) * v_o3;
    end
    agent3.x =  x03 + sim.ts .* agent3.v;
    % 实际智能体4编队行为
    [bar_sigma_f4, sigmaf_i4, pf_i4, vf_i4] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p, sigma_v, 4);
    int_f4_dot = -(2 ./ (  param.rho_f .* param.mu_f .* param.T_f))* inv([param.kf_gain, 0; 0,  param.kangle_gain]) * (2 .* param.mu_f ...
        .* bar_sigma_f4 + return_sign2(bar_sigma_f4, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f4, 1+param.rho_f));
    int_f4 = int_f4 + int_f4_dot * sim.ts;
    s_f4 = bar_sigma_f4 + int_f4;
    trace_s_f4(:, k) = s_f4;
    [v_fm4, jacobi_fm4] = PDFBC(param, p_now, bar_sigma_f4, pf_i4, vf_i4, sigma_p, s_f4, 4);
    if  min_dis_detected4 <  agent4.R_safe
        agent4.v =  v_o4 + (eye(3, 3) - pinv(jacobi_o4) * jacobi_o4) *v_fm4;
    else
        agent4.v =  v_fm4 + (eye(3, 3) - pinv(jacobi_fm4) * jacobi_fm4) * v_o4;
    end
    agent4.x =  x04  + sim.ts .* agent4.v;
    % 实际智能体5编队行为
    [bar_sigma_f5, sigmaf_i5, pf_i5, vf_i5] = return_bar_sigmaf(param, p_now, v_now, p_esm, ...
        pv_esm,  sigma_p, sigma_v, 5);
    int_f5_dot = -(2 ./ ( param.rho_f .* param.mu_f .* param.T_f))* inv([param.kf_gain, 0; 0,  param.kangle_gain]) * (2 .* param.mu_f ...
        .* bar_sigma_f5 + return_sign2(bar_sigma_f5, ...
        1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f5, 1+param.rho_f));
    int_f5 = int_f5 + int_f5_dot * sim.ts;
    s_f5 = bar_sigma_f5 + int_f5;
    trace_s_f5(:, k) = s_f5;
    [v_fm5, jacobi_fm5] = PDFBC(param, p_now, bar_sigma_f5, pf_i5, vf_i5, sigma_p, s_f5, 5);
    if  min_dis_detected5 <  agent5.R_safe
        agent5.v =  v_o5 + (eye(3, 3) - pinv(jacobi_o5) * jacobi_o5) *v_fm5;
    else
        agent5.v =  v_fm5 + (eye(3, 3) - pinv(jacobi_fm5) * jacobi_fm5) * v_o5;
    end
    agent5.x =  x05 + sim.ts .* agent5.v;
    trace_taskerror1(k)=norm(bar_sigma_f1);
    trace_taskerror2(k)=norm(bar_sigma_f2);
    trace_taskerror3(k)=norm(bar_sigma_f3);
    trace_taskerror4(k)=norm(bar_sigma_f4);
    trace_taskerror5(k)=norm(bar_sigma_f5);
     if j > ctr_param.T 
       ctr_param.cmu_p = 12;
    else
        ctr_param.cmu_p = ctr_param.cmu_p +  12 / ctr_param.T * sim.ts;
     end
     %% 控制器1
    x0_d = agent1.x;
    x1_d = agent1.v;
    [T_phai, T_phai_dot] = return_T_phai(agent1.x(3), agent1.v(3));
    e_rr = x0 - x0_d;
    e_dot = x1 - x1_d;
     [A1, A2, e_ddt] = return_A1A2(tao, x0_d, x1_d, e_dot, H_d(j), M, T_phai,T_phai_dot, J, J_inv, F_f, D_theta, R);
    v_term = ctr_term(ctr_param, e_rr, e_dot, z2_esm);
    z1 =e_dot;
    e1 = z1 - z1_esm;
    z1_esm_dot = v_term + z2_esm + mu1 .* return_sign2(e1, r1);
    z2_esm_dot = mu2 .*return_sign2(e1, r2);
    z1_esm = z1_esm +  sim.ts .* z1_esm_dot;
    z2_esm = z2_esm +  sim.ts .* z2_esm_dot;
    B = R .* T_phai * J_inv * inv(M);
    tao = - pinv(B) * (-A1 * e_dot - A2) + pinv(B) * v_term;
    [x0_dot, x1_dot] = dynamic_model(tao, x1, H_d(j) , M, T_phai,T_phai_dot, J, J_inv, F_f , D_theta, R);
    x0 = x0 + sim.ts .* x0_dot;
    x1 = x1 + sim.ts .* x1_dot;
    %% 控制器2
    x0_d2 = agent2.x;
    x1_d2 = agent2.v;
    [T_phai2, T_phai_dot2] = return_T_phai(agent2.x(3), agent2.v(3));
    e_rr2 = x02 - x0_d2;
    e_dot2 = x12 - x1_d2;
     [A12, A22, e_ddt2] = return_A1A2(tao2, x0_d2, x1_d2, e_dot2, H_d(j), M, T_phai2,T_phai_dot2, J, J_inv, F_f, D_theta, R);
    v_term2 = ctr_term(ctr_param, e_rr2, e_dot2, z2_esm2);
    z12 =e_dot2;
    e12 = z12 - z1_esm2;
    z1_esm_dot2 = v_term2 + z2_esm2 + mu1 .* return_sign2(e12, r1);
    z2_esm_dot2 = mu2 .*return_sign2(e12, r2);
    z1_esm2 = z1_esm2 +  sim.ts .* z1_esm_dot2;
    z2_esm2 = z2_esm2 +  sim.ts .* z2_esm_dot2;
    B2 = R .* T_phai2 * J_inv * inv(M);
    tao2 = - pinv(B2) * (-A12 * e_dot2 - A22) + pinv(B2) * v_term2;
    [x0_dot2, x1_dot2] = dynamic_model(tao2, x12, H_d(j) , M, T_phai2,T_phai_dot2, J, J_inv, F_f , D_theta, R);
    x02 = x02 + sim.ts .* x0_dot2;
    x12 = x12 + sim.ts .* x1_dot2;
    %% 控制器3
    x0_d3 = agent3.x;
    x1_d3 = agent3.v;
    [T_phai3, T_phai_dot3] = return_T_phai(agent3.x(3), agent3.v(3));
    e_rr3 = x03 - x0_d3;
    e_dot3 = x13 - x1_d3;
     [A13, A23, e_ddt3] = return_A1A2(tao3, x0_d3, x1_d3, e_dot3, H_d(j), M, T_phai3,T_phai_dot3, J, J_inv, F_f, D_theta, R);
    v_term3 = ctr_term(ctr_param, e_rr3, e_dot3, z2_esm3);
    z13 =e_dot3;
    e13 = z13 - z1_esm3;
    z1_esm_dot3 = v_term3 + z2_esm3 + mu1 .* return_sign2(e13, r1);
    z2_esm_dot3 = mu2 .*return_sign2(e13, r2);
    z1_esm3 = z1_esm3 +  sim.ts .* z1_esm_dot3;
    z2_esm3 = z2_esm3 +  sim.ts .* z2_esm_dot3;
    B3 = R .* T_phai3 * J_inv * inv(M);
    tao3 = - pinv(B3) * (-A13 * e_dot3 - A23) + pinv(B3) * v_term3;
    [x0_dot3, x1_dot3] = dynamic_model(tao3, x13, H_d(j) , M, T_phai3,T_phai_dot3, J, J_inv, F_f , D_theta, R);
    x03 = x03 + sim.ts .* x0_dot3;
    x13 = x13 + sim.ts .* x1_dot3;
    %% 控制器4
    x0_d4 = agent4.x;
    x1_d4 = agent4.v;
    [T_phai4, T_phai_dot4] = return_T_phai(agent4.x(3), agent4.v(3));
    e_rr4 = x04 - x0_d4;
    e_dot4 = x14 - x1_d4;
     [A14, A24, e_ddt4] = return_A1A2(tao4, x0_d4, x1_d4, e_dot4, H_d(j), M, T_phai4,T_phai_dot4, J, J_inv, F_f, D_theta, R);
    v_term4 = ctr_term(ctr_param, e_rr4, e_dot4, z2_esm4);
    z14 =e_dot4;
    e14 = z14 - z1_esm4;
    z1_esm_dot4 = v_term4 + z2_esm4 + mu1 .* return_sign2(e14, r1);
    z2_esm_dot4 = mu2 .*return_sign2(e14, r2);
    z1_esm4 = z1_esm4 +  sim.ts .* z1_esm_dot4;
    z2_esm4 = z2_esm4 +  sim.ts .* z2_esm_dot4;
    B4 = R .* T_phai4 * J_inv * inv(M);
    tao4 = - pinv(B4) * (-A14 * e_dot4 - A24) + pinv(B4) * v_term4;
    [x0_dot4, x1_dot4] = dynamic_model(tao4, x14, H_d(j) , M, T_phai4,T_phai_dot4, J, J_inv, F_f , D_theta, R);
    x04 = x04 + sim.ts .* x0_dot4;
    x14 = x14 + sim.ts .* x1_dot4;
    %% 控制器5
    x0_d5 = agent5.x;
    x1_d5 = agent5.v;
    [T_phai5, T_phai_dot5] = return_T_phai(agent5.x(3), agent5.v(3));
    e_rr5 = x05 - x0_d5;
    e_dot5 = x15 - x1_d5;
     [A15, A25, e_ddt5] = return_A1A2(tao5, x0_d5, x1_d5, e_dot5, H_d(j), M, T_phai5,T_phai_dot5, J, J_inv, F_f, D_theta, R);
    v_term5= ctr_term(ctr_param, e_rr5, e_dot5, z2_esm5);
    z15 =e_dot5;
    e15 = z15 - z1_esm5;
    z1_esm_dot5 = v_term5 + z2_esm5 + mu1 .* return_sign2(e15, r1);
    z2_esm_dot5 = mu2 .*return_sign2(e15, r2);
    z1_esm5 = z1_esm5 +  sim.ts .* z1_esm_dot5;
    z2_esm5 = z2_esm5 +  sim.ts .* z2_esm_dot5;
    B5 = R .* T_phai5 * J_inv * inv(M);
    tao5 = - pinv(B5) * (-A15 * e_dot5 - A25) + pinv(B5) * v_term5;
    [x0_dot5, x1_dot5] = dynamic_model(tao5, x15, H_d(j) , M, T_phai5,T_phai_dot5, J, J_inv, F_f , D_theta, R);
    x05 = x05 + sim.ts .* x0_dot5;
    x15 = x15 + sim.ts .* x1_dot5;
    trace_ang1(k)=agent1.x(3);
    trace_ang2(k)=agent2.x(3);
    trace_ang3(k)=agent3.x(3);
    trace_ang4(k)=agent4.x(3);
    trace_angest(k)=p_esm(3,4);
    trace_ang5(k)=agent5.x(3);
    trace_ang_desire(k)=agent0.x(3);
    trace_obs(:,k)=[Ox0(j);Oy0(j)];
    trace_obs2(:,k)=[Ox10(j);Oy10(j)];
    trace_u1(k) = norm(tao);
    trace_u2(k) = norm(tao2);
    trace_u3(k) = norm(tao3);
    trace_u4(k) = norm(tao4);
    trace_u5(k) = norm(tao5);
    trace_trackingerror1(k)=norm(e_rr);
    trace_trackingerror2(k)=norm(e_rr2);
    trace_trackingerror3(k)=norm(e_rr3);
    trace_trackingerror4(k)=norm(e_rr4);
    trace_trackingerror5(k)=norm(e_rr5);
    trace_1(:,k)=[bar_sigma_f1(1),bar_sigma_f2(1),bar_sigma_f3(1),bar_sigma_f4(1),bar_sigma_f5(1)]';
    trace_2(:,k)=[bar_sigma_f1(2),bar_sigma_f2(2),bar_sigma_f3(2),bar_sigma_f4(2),bar_sigma_f5(2)]';
end
hold on
axis equal
% figure(22)
% hold on
% set(gcf, 'Position', [100, 700, 800, 600]);
% plot(agent0.record_x, agent0.record_y, Color='k',LineStyle='-',LineWidth=4)
% axis equal
% for step = 1:1:sim.point_num
%     index_line =step;
%     plot(agent1.record_x(1:step), agent1.record_y(1:step), Color='b',LineStyle='--',LineWidth=2)
%     plot(agent2.record_x(1:step), agent2.record_y(1:step), Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
%     plot(agent3.record_x(1:step), agent3.record_y(1:step), Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
%     plot(agent4.record_x(1:step), agent4.record_y(1:step), Color=[1 0.6 0],LineStyle='--',LineWidth=2)
%     plot(agent5.record_x(1:step), agent5.record_y(1:step), Color='r',LineStyle='--',LineWidth=2)
%     h6 = line([agent1.record_x( index_line),agent2.record_x( index_line)],[agent1.record_y( index_line), ...
%         agent2.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
%     h7 =line([agent2.record_x( index_line),agent3.record_x( index_line)],[agent2.record_y( index_line), ...
%         agent3.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
%     h8 =line([agent3.record_x( index_line),agent4.record_x( index_line)],[agent3.record_y( index_line), ...
%         agent4.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
%     h9 =line([agent4.record_x( index_line),agent5.record_x( index_line)],[agent4.record_y( index_line), ...
%         agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
%     h10 =line([agent1.record_x( index_line),agent5.record_x( index_line)],[agent1.record_y( index_line), ...
%         agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
%     [h1,th1,h2,h3]=Carplot_Ezekiel(agent1.record_x(index_line),agent1.record_y(index_line),agent1.record_psi(index_line),0,'k','b',[2.5,6],1);
%     [h21,th21,h22,h23]=Carplot_Ezekiel(agent2.record_x(index_line),agent2.record_y(index_line),agent2.record_psi(index_line),0,'k',[98,91,161]./255,[2.5,6],2);
%     [h31,th31,h32,h33]=Carplot_Ezekiel(agent3.record_x(index_line),agent3.record_y(index_line),agent3.record_psi(index_line),0,'k',[219,0,194]./255,[2.5,6],3);
%     [h41,th41,h42,h43]=Carplot_Ezekiel(agent4.record_x(index_line),agent4.record_y(index_line),agent4.record_psi(index_line),0,'k',[1 0.6 0],[2.5,6],4);
%     [h51,th51,h52,h53]=Carplot_Ezekiel(agent5.record_x(index_line),agent5.record_y(index_line),agent5.record_psi(index_line),0,'k','r',[2.5,6],5);
%     drawnow;
%     delete(h6);delete(h7);delete(h8);delete(h9);delete(h10);
%     for jj = 1:1:size(h2)
%         delete(h2{jj});
%         delete(h22{jj});
%         delete(h32{jj});
%         delete(h42{jj});
%         delete(h52{jj});
%     end
%     delete(h1);delete(th1);delete(h3);
%     delete(h21);delete(th21);delete(h23);
%     delete(h31);delete(th31);delete(h33);
%     delete(h41);delete(th41);delete(h43);
%     delete(h51);delete(th51);delete(h53);
% end

figure(2)
hold on
set(gcf, 'Position', [100, 700, 800, 600]);
axis([-65,65,-65,65])
plot(agent1.record_x, agent1.record_y, Color='b',LineStyle='--',LineWidth=2)
plot(agent2.record_x, agent2.record_y, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(agent3.record_x, agent3.record_y, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(agent4.record_x, agent4.record_y, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(agent5.record_x, agent5.record_y, Color='r',LineStyle='--',LineWidth=2)
plot(agent0.record_x, agent0.record_y, Color='k',LineStyle='-',LineWidth=4)
plot(trace_obs(1,:), trace_obs(2,:), Color='g',LineStyle='-',LineWidth=2)
plot(Ox30(1),Oy30(1),'ko',LineWidth=2)
plot(Ox20(1),Oy20(1),'ko',LineWidth=2)
plot(Ox40(1),Oy40(1),'ko',LineWidth=2)
plot(trace_obs2(1,:), trace_obs2(2,:), Color='g',LineStyle='-',LineWidth=2)
[hobs6]=obsdraw(1.5,1.5,trace_obs2(1,1),trace_obs2(2,1),'g');
[hobs7]=obsdraw(1.5,1.5,trace_obs2(1,end),trace_obs2(2,end),'g');
[hobs]=obsdraw(1.5,1.5,trace_obs(1,1),trace_obs(2,1),'g');
[hobs1]=obsdraw(1.5,1.5,trace_obs(1,end),trace_obs(2,end),'g');
plot(agent0.record_x(1), agent0.record_y(1), 'ks',LineWidth=4)
plot(agent0.record_x(end), agent0.record_y(end),'kd',LineWidth=4)
index_line =1;
h6 = line([agent1.record_x( index_line),agent2.record_x( index_line)],[agent1.record_y( index_line), ...
    agent2.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h7 =line([agent2.record_x( index_line),agent3.record_x( index_line)],[agent2.record_y( index_line), ...
    agent3.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h8 =line([agent3.record_x( index_line),agent4.record_x( index_line)],[agent3.record_y( index_line), ...
    agent4.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h9 =line([agent4.record_x( index_line),agent5.record_x( index_line)],[agent4.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h10 =line([agent1.record_x( index_line),agent5.record_x( index_line)],[agent1.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
Carplot_Ezekiel(agent1.record_x(index_line),agent1.record_y(index_line),agent1.record_psi(index_line),0,'k','b',[2.5,6],1);
Carplot_Ezekiel(agent2.record_x(index_line),agent2.record_y(index_line),agent2.record_psi(index_line),0,'k',[98,91,161]./255,[2.5,6],2);
Carplot_Ezekiel(agent3.record_x(index_line),agent3.record_y(index_line),agent3.record_psi(index_line),0,'k',[219,0,194]./255,[2.5,6],3);
Carplot_Ezekiel(agent4.record_x(index_line),agent4.record_y(index_line),agent4.record_psi(index_line),0,'k',[1 0.6 0],[2.5,6],4);
Carplot_Ezekiel(agent5.record_x(index_line),agent5.record_y(index_line),agent5.record_psi(index_line),0,'k','r',[2.5,6],5);
index_line =1300;
h6 = line([agent1.record_x( index_line),agent2.record_x( index_line)],[agent1.record_y( index_line), ...
    agent2.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h7 =line([agent2.record_x( index_line),agent3.record_x( index_line)],[agent2.record_y( index_line), ...
    agent3.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h8 =line([agent3.record_x( index_line),agent4.record_x( index_line)],[agent3.record_y( index_line), ...
    agent4.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h9 =line([agent4.record_x( index_line),agent5.record_x( index_line)],[agent4.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h10 =line([agent1.record_x( index_line),agent5.record_x( index_line)],[agent1.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
Carplot_Ezekiel(agent1.record_x(index_line),agent1.record_y(index_line),agent1.record_psi(index_line),0,'k','b',[2.5,6],1);
Carplot_Ezekiel(agent2.record_x(index_line),agent2.record_y(index_line),agent2.record_psi(index_line),0,'k',[98,91,161]./255,[2.5,6],2);
Carplot_Ezekiel(agent3.record_x(index_line),agent3.record_y(index_line),agent3.record_psi(index_line),0,'k',[219,0,194]./255,[2.5,6],3);
Carplot_Ezekiel(agent4.record_x(index_line),agent4.record_y(index_line),agent4.record_psi(index_line),0,'k',[1 0.6 0],[2.5,6],4);
Carplot_Ezekiel(agent5.record_x(index_line),agent5.record_y(index_line),agent5.record_psi(index_line),0,'k','r',[2.5,6],5);

index_line =sim.point_num-5;
h6 = line([agent1.record_x( index_line),agent2.record_x( index_line)],[agent1.record_y( index_line), ...
    agent2.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h7 =line([agent2.record_x( index_line),agent3.record_x( index_line)],[agent2.record_y( index_line), ...
    agent3.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h8 =line([agent3.record_x( index_line),agent4.record_x( index_line)],[agent3.record_y( index_line), ...
    agent4.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h9 =line([agent4.record_x( index_line),agent5.record_x( index_line)],[agent4.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h10 =line([agent1.record_x( index_line),agent5.record_x( index_line)],[agent1.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
Carplot_Ezekiel(agent1.record_x(index_line),agent1.record_y(index_line),agent1.record_psi(index_line),0,'k','b',[2.5,6],1);
Carplot_Ezekiel(agent2.record_x(index_line),agent2.record_y(index_line),agent2.record_psi(index_line),0,'k',[98,91,161]./255,[2.5,6],2);
Carplot_Ezekiel(agent3.record_x(index_line),agent3.record_y(index_line),agent3.record_psi(index_line),0,'k',[219,0,194]./255,[2.5,6],3);
Carplot_Ezekiel(agent4.record_x(index_line),agent4.record_y(index_line),agent4.record_psi(index_line),0,'k',[1 0.6 0],[2.5,6],4);
Carplot_Ezekiel(agent5.record_x(index_line),agent5.record_y(index_line),agent5.record_psi(index_line),0,'k','r',[2.5,6],5);
index_line =200;
h6 = line([agent1.record_x( index_line),agent2.record_x( index_line)],[agent1.record_y( index_line), ...
    agent2.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h7 =line([agent2.record_x( index_line),agent3.record_x( index_line)],[agent2.record_y( index_line), ...
    agent3.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h8 =line([agent3.record_x( index_line),agent4.record_x( index_line)],[agent3.record_y( index_line), ...
    agent4.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h9 =line([agent4.record_x( index_line),agent5.record_x( index_line)],[agent4.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1.5);
h10 =line([agent1.record_x( index_line),agent5.record_x( index_line)],[agent1.record_y( index_line), ...
    agent5.record_y(index_line)],'Color','k','LineStyle','-.', LineWidth=1);
Carplot_Ezekiel(agent1.record_x(index_line),agent1.record_y(index_line),agent1.record_psi(index_line),0,'k','b',[2.5,6],1);
Carplot_Ezekiel(agent2.record_x(index_line),agent2.record_y(index_line),agent2.record_psi(index_line),0,'k',[98,91,161]./255,[2.5,6],2);
Carplot_Ezekiel(agent3.record_x(index_line),agent3.record_y(index_line),agent3.record_psi(index_line),0,'k',[219,0,194]./255,[2.5,6],3);
Carplot_Ezekiel(agent4.record_x(index_line),agent4.record_y(index_line),agent4.record_psi(index_line),0,'k',[1 0.6 0],[2.5,6],4);
Carplot_Ezekiel(agent5.record_x(index_line),agent5.record_y(index_line),agent5.record_psi(index_line),0,'k','r',[2.5,6],5);
title('Trajectory')
xlabel('X(m)')
ylabel('Y(m)')
axis equal
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Barycenter','Dynamic obstacles','Static obstacles','Location','southEast');
grid on
plot(agent1.record_x, agent1.record_y, Color='b',LineStyle='--',LineWidth=2)
plot(agent2.record_x, agent2.record_y, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(agent3.record_x, agent3.record_y, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(agent4.record_x, agent4.record_y, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(agent5.record_x, agent5.record_y, Color='r',LineStyle='--',LineWidth=2)
figure(3)
hold on
set(gcf, 'Position', [900, 700, 800, 600]);
subplot(3,1,1)
plot(sim.t, trace_taskerror1, Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_taskerror2, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_taskerror3, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_taskerror4, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(sim.t, trace_taskerror5, Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('||\delta_{f,i}|| (m)')
title('The norm of behaviral task error 1')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','northEast');
axis([0,20,0,35])
subplot(3,1,2)
plot(sim.t, trace_1(1,:), Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_1(2,:), Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot( sim.t, trace_1(3,:), Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot( sim.t, trace_1(4,:), Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot( sim.t, trace_1(5,:), Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('\delta_{f,i}(1) (m)')
title('Behaviral task error 1')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','southEast');
axis([0,20,-38,1])
subplot(3,1,3)
plot(sim.t, trace_2(1,:), Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_2(2,:), Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot( sim.t, trace_2(3,:), Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot( sim.t, trace_2(4,:), Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot( sim.t, trace_2(5,:), Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('\delta_{f,i}(2) (rad)')
title('Behaviral task error 2')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','southEast');
axis([0,20,-1.5,0.5])
grid on

figure(4)
set(gcf, 'Position', [1600, 700, 800, 600]); 
subplot(3,1,1)
plot(sim.t, trace_esterror1, Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_esterror2, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_esterror3, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_esterror4, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(sim.t, trace_esterror5, Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('Task-space errors (m)')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','northEast');
title('The norm of estimating errors')
subplot(3,1,2)
plot(sim.t, trace_trackingerror1, Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_trackingerror2, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_trackingerror3, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_trackingerror4, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(sim.t, trace_trackingerror5, Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('Behavioral tracking errors (m)')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','northEast');
title('The norm of the behavioral tracking errors')
subplot(3,1,3)
plot(sim.t, trace_u1, Color='b',LineStyle='--',LineWidth=2)
hold on
plot(sim.t, trace_u2, Color=[98,91,161]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_u3, Color=[219,0,194]./255,LineStyle='--',LineWidth=2)
plot(sim.t, trace_u4, Color=[1 0.6 0],LineStyle='--',LineWidth=2)
plot(sim.t, trace_u5, Color='r',LineStyle='--',LineWidth=2)
grid on
xlabel('Time (sec)')
ylabel('||u|| (N·m)')
legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Location','northEast');
title('The norm of the control input')
% figure(5)
% set(gcf, 'Position', [100, 100, 800, 600]);

function [h1]=obsdraw(a,b,x_o,y_o, input)
theta = linspace(0, 2*pi, 40);
x_obs = a * cos(theta)+x_o;
y_obs = b * sin(theta)+y_o;
plot(x_obs, y_obs,'k',  'LineWidth', 2);
h1=fill(x_obs, y_obs,input); % 灰色填充
end
function [h1,th1,h2,h3]=Carplot_Ezekiel(x,y,psi,phy,box_color,wheel_color,text_index,car_index)
h1 =  PlotCarbox(x, y, psi, 'Color', box_color,LineWidth=2);
th1 = text(x+text_index(1), y+text_index(2), ['#car', num2str(car_index)], 'Color',[0.7, 0.7, 0.7]);
set(th1, {'HorizontalAlignment'},{'center'});
h2 = PlotCarWheels(x, y, psi,phy,'b', 'EdgeColor',wheel_color,'Linewidth',3);
h3 = plot(x , y, Color='b',LineStyle='-',LineWidth=4);
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
function [v_m, jacobi_m] = PIBC(param, bar_sigma_c , v_expected, s_c1)
[scale_x, ~]  = size(bar_sigma_c );
jacobi_m = eye(scale_x, scale_x);
Phai_c=(2 ./ (param.rho_c .* param.mu_c .* param.T_c))*(2 .* param.mu_c .* bar_sigma_c + return_sign2(bar_sigma_c, ...
    1-param.rho_c) + param.mu_c^2 .* return_sign2(bar_sigma_c, 1+param.rho_c))-(2. / ( param.km_gain .* param.rho_c1 .* ...
    param.mu_c1 .* param.T_c1)) * (2 .* param.mu_c1 .* s_c1 + return_sign2(s_c1, ...
    1-param.rho_c) + param.mu_c1^2 .* return_sign2(s_c1,1+param.rho_c));
v_m = pinv(jacobi_m) * (v_expected + [param.km_gain,0,0;0,param.km_gain,0;0,0,param.kangle_gain]* (Phai_c));
end
function [v_f, jacobi_f] = PDFBC(param, p_now, bar_sigma_f, pf_i, vf_i, sigma_x, s_f1, agent_id)
jacobi_f = [(1/norm(p_now(1:2, agent_id) - pf_i(1:2) - sigma_x(1:2, agent_id))) * (p_now(1:2, agent_id) - pf_i(1:2) - sigma_x(1:2, agent_id))',0;0,0,1];
Phai_f = (2 ./ (param.rho_f .* param.mu_f .* param.T_f))*(2 .* param.mu_f .* bar_sigma_f + return_sign2(bar_sigma_f, ...
    1-param.rho_f) + param.mu_f^2 .* return_sign2(bar_sigma_f, 1+param.rho_f))-(2. / ( param.kf_gain .* param.rho_f1 .* ...
    param.mu_f1 .* param.T_f1)) * (2 .* param.mu_f1 .* s_f1 + return_sign2(s_f1, ...
    1-param.rho_f) + param.mu_f1^2 .* return_sign2(s_f1,1+param.rho_f));
v_f = pinv(jacobi_f) * ([param.kf_gain,0;0,param.kangle_gain]* (Phai_f)- jacobi_f * vf_i );
end

function [sigma_a,  jacobi_dpwdt,  jacobi_o1] = return_sigma_a(param, x_w, x_ow)
kapa = (1 / (( param.alpha).^param.q)) .* (x_w(1) - x_ow(1))^param.q...
    +(1 / ((param.beta)^param.q)) * (x_w(2) - x_ow(2))^param.q;
sigma_a = [exp(-kapa); x_w(3)];
dx = -(1 / (( param.alpha).^param.q)) .* param.q *  (x_w(1) - x_ow(1))^(param.q-1) * exp(-kapa);
dy= -(1 / (( param.beta).^param.q)) .* param.q *  (x_w(2) - x_ow(2))^(param.q-1) * exp(-kapa);
if abs(dx) <0.001
    dx=0;
end
if abs(dy) <0.001
    dy=0;
end
dxow = (1 / (( param.alpha).^param.q)) .* param.q *  (x_w(1) - x_ow(1))^(param.q-1) * exp(-kapa);
dyow= (1 / (( param.beta).^param.q)) .* param.q *  (x_w(2) - x_ow(2))^(param.q-1) * exp(-kapa);
jacobi_dpwdt = [dxow, dyow;0,0];
jacobi_o1 = [dx, dy, 0; 0, 0, 1] ;
end
function [sigma_a,  jacobi_dpwdt,  jacobi_o1] = return_sigma_a1(param, x_w, x_ow)
kapa = (1 / (( param.alpha1).^param.q)) .* (x_w(1) - x_ow(1))^param.q...
    +(1 / ((param.beta1)^param.q)) * (x_w(2) - x_ow(2))^param.q;
sigma_a = [exp(-kapa); x_w(3)];
dx = -(1 / (( param.alpha1).^param.q)) .* param.q *  (x_w(1) - x_ow(1))^(param.q-1) * exp(-kapa);
dy= -(1 / (( param.beta1).^param.q)) .* param.q *  (x_w(2) - x_ow(2))^(param.q-1) * exp(-kapa);
if abs(dx) <0.001
    dx=0;
end
if abs(dy) <0.001
    dy=0;
end
dxow = (1 / (( param.alpha1).^param.q)) .* param.q *  (x_w(1) - x_ow(1))^(param.q-1) * exp(-kapa);
dyow= (1 / (( param.beta1).^param.q)) .* param.q *  (x_w(2) - x_ow(2))^(param.q-1) * exp(-kapa);
jacobi_dpwdt = [dxow, dyow;0,0];
jacobi_o1 = [dx, dy, 0; 0, 0, 1] ;
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
pf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (p_esm(:, agent_id)+sigma_x(:, agent_id)) + param.k2 .* sum_aijpj);
vf_i = (1/(param.k1+ param.k2 .* sumaij)) * (param.k1 .* (pv_esm(:, agent_id)+sigma_v(:, agent_id)) + param.k2 .* sumv_aijpj);
sigmaf_i = [norm(p_now(1:2, agent_id) - pf_i(1:2) - sigma_x(1:2, agent_id)), p_now(3)]';
bar_sigma_f = [0,pf_i(3)]' - sigmaf_i;
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
esm_x_dot =-(2 ./ (param.lamda_PH .* param.rho_epv .* param.mu_epv .* param.T_epv))*(2 .* param.mu_epv ...
    .* ep1 + return_sign2(ep1, 1-param.rho_epv) + param.mu_epv^2 .* return_sign2(ep1, 1+ ...
    param.rho_epv)) - param.lo1 .*sign(ep1) + v0;
esm_v_dot =-(2 ./ (param.lamda_PH .* param.rho_epv .* param.mu_epv .* param.T_epv))*(2 .* param.mu_epv ...
    .* ev1 + return_sign2(ev1, 1-param.rho_epv) + param.mu_epv^2 .* return_sign2(ev1, 1+ ...
    param.rho_epv)) - param.lo2 .*sign(ev1) ;
end

function [min_id, min_dis_detected,  criteria_detected, criteria_safe , obs_detected,  obs_safe]...
    = returnnearobstacle(x_now, simobs, detected_field,  d_safe)
[~, idobs] = size(simobs);
min_id = 0;
min_dis_detected = inf;
d_xobs = zeros(idobs, 2);
for ci=1:1:idobs
    d_xobs(ci, 1) = norm(x_now(1:2) - simobs(:, ci), 2);
    if d_xobs(ci) <= detected_field
        d_xobs(ci, 2) = 1;
        if d_xobs(ci) <=  d_safe
            d_xobs(ci, 2) = 2;
        end
        if d_xobs(ci, 1) <= min_dis_detected
            min_dis_detected = d_xobs(ci, 1);
            min_id = ci;
        end
    end
end
criteria_detected = find(d_xobs(:, 2) >= 1); % 传感范围内的所有障碍物点云
criteria_safe = find(d_xobs(:, 2) == 2); % 安全区范围内的所有障碍物点云
obs_detected = simobs(:, criteria_detected);
obs_safe = simobs(:, criteria_safe);
end
function [detected_num,bar_sigma_a,  jacobi_o,jacobi_dpwdt,dsigma_a1,augmentobsv] =dynamicobsavoid(param, x_now, v_now, x_desire,obs_detected, obstacle_vrd)
[~, detected_num] = size(obs_detected);
jacobi_o = zeros(2*detected_num, 3);
dsigma_a1 = zeros(2,detected_num);
jacobi_dpwdt = zeros(2*detected_num,2);
sigma_d = [zeros(1,detected_num); x_desire(3).*ones(1,detected_num)];
sigma_a1=zeros(2,detected_num);
augmentobsv=[obstacle_vrd(1) .* ones(1, detected_num);obstacle_vrd(2) .* ones(1, detected_num)];
for obs_id = 1:1:detected_num
    [sigma_a1(:, obs_id), jacobi_dpwdt1, jacobi_o1]...
        = return_sigma_a(param, x_now, obs_detected(:, obs_id));
    if norm( jacobi_o1) > 0.000001
        jacobi_o((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:) = jacobi_o1;
    else
        jacobi_o((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:) = [0, 0, 0;0, 0, 0];
    end
    if norm( jacobi_dpwdt1) > 0.000001
        jacobi_dpwdt((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:)  = jacobi_dpwdt1;
    else
        jacobi_dpwdt((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:)  = [0,0;0,0];
    end
    dsigma_a1(:, obs_id) =  jacobi_o((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:) *  v_now + jacobi_dpwdt((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:)  * obstacle_vrd(:, obs_id);
end
bar_sigma_a  = sigma_d - sigma_a1;
end

function v_o = PCABC(param, bar_sigma_a, s_a1, jacobi_o, jacobi_ow, obs_v, detected_num)
v_o = zeros(3,1);
Phai_a = (2 ./ (param.rho_a .* param.mu_a .* param.T_a))*(2 .* param.mu_a .* bar_sigma_a + return_sign2(bar_sigma_a, ...
    1-param.rho_a) + param.mu_a^2 .* return_sign2(bar_sigma_a, 1+param.rho_a))-(2. / ( param.ka_gain .* param.rho_a1 .* ...
    param.mu_a1 .* param.T_a1)) * (2 .* param.mu_a1 .* s_a1 + return_sign2(s_a1, ...
    1-param.rho_a) + param.mu_a1^2 .* return_sign2(s_a1,1+param.rho_a));
for obs_id = 1:1:detected_num
    v_o =v_o +  pinv(jacobi_o((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:)) * ([param.ka_gain,0 ; 0, param.kangle_gain]* (Phai_a(:,obs_id)) -jacobi_ow((2 * (obs_id-1)+1):(2 * (obs_id-1)+2),:)  * obs_v(:,obs_id));%
end
v_o= 1/detected_num.*v_o;
end
function [T_phai, T_phai_dot] = return_T_phai(phai, phai_dot)
T_phai = [cos(phai), -sin(phai), 0;
    sin(phai), cos(phai), 0;
    0,               0,            1];
T_phai_dot = [-sin(phai)/(cos(phai)^2 + sin(phai)^2),  cos(phai)/(cos(phai)^2 + sin(phai)^2), 0;
    -cos(phai)/(cos(phai)^2 + sin(phai)^2), -sin(phai)/(cos(phai)^2 + sin(phai)^2), 0;
    0,                                      0, 0] .* phai_dot;
end
function [A1, A2, e_ddt] = return_A1A2(tao, x0d_dot, x0d_ddot, e_dot, H_d, M, T_phai,T_phai_dot, J, J_inv, F_f, D_theta, R)
A1 = T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai);
A2 =(T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai)) * x0d_dot + (R.^2) .* T_phai * J_inv * inv(M) * F_f + x0d_ddot;
u = R .* T_phai * J_inv * inv(M) * tao;
kexi = R .* T_phai * J_inv * inv(M) * H_d;
e_ddt = -A1 * e_dot - A2 + u + kexi;
end
function [sign_r1] = return_sign(e1, r1)
sign_r1 = [((abs(e1(1)))^ r1) * sign(e1(1)), ((abs(e1(2)))^ r1) * sign(e1(2)), ((abs(e1(3)))^ r1) * sign(e1(3))]';
end
function [x0_dot, x1_dot] = dynamic_model(tao, x1, H_d , M, T_phai,T_phai_dot, J, J_inv, F_f, D_theta, R)
x0_dot = x1;
g= - (T_phai * T_phai_dot + D_theta .* (R .^2) * T_phai * J_inv * inv(M) * J * inv(T_phai)) * x1 - (R.^2) .* T_phai * J_inv * inv(M) * F_f;
u = R .* T_phai * J_inv * inv(M) * tao;
kexi = R .* T_phai * J_inv * inv(M) * H_d;
x1_dot = u + g + kexi;
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
