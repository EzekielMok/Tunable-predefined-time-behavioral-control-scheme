%% ����ÿ��������ά�����࣬����״̬���ٶȣ������ٶȣ�����λ�ã���в���ϰ����֪��Χ�ڵĻ����������������ϰ��,ͨѶ���
classdef  agentclass2dim < handle	%������Ŀ�ͷagentclassΪ������
	properties %����block��ʼ
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
    end			    %����block����
	methods		%����block��ʼ
		    function obj = agentclass2dim(x0, v0, v_d0, x_rd0, R, r)	%���캯�����ص�Ҳ�Ǻ���ͬ��
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
	end			%����block����
end						%�ඨ�����