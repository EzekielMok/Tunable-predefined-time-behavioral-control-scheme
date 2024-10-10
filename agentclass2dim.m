%% 创建每个智能体维护的类，包含状态，速度，期望速度，期望位置，威胁的障碍物，感知范围内的环境（包括车辆与障碍物）,通讯情况
classdef  agentclass2dim < handle	%定义类的开头agentclass为类名字
	properties %属性block开始
		x;
        v;
        v_d;
        x_rd;
        record_x;
        record_y;
        record_psi;
        record_taskstate;
        record_Cos_sim; 
         R_safe;
         r_robot;
    end			    %属性block结束
	methods		%方法block开始
		    function obj = agentclass2dim(x0, v0, v_d0, x_rd0, R, r)	%构造函数，特点也是和类同名
                  obj.x = x0;
                  obj.v =  v0;
                  obj.v_d = v_d0;
                  obj.x_rd = x_rd0; 
                  obj.record_x = [];
                  obj.record_y = [];
                  obj.record_psi = [];
                  obj.record_taskstate = [];
                   obj.record_Cos_sim = [];
                  obj. R_safe =  R;
                  obj. r_robot = r;
            end
	end			%方法block结束
end						%类定义结束