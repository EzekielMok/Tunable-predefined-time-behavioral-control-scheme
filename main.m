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

