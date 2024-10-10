classdef  KinematicModel<handle
    properties
        x;
        y;
        psi;
        v;
        dt;
        L;
        record_x;
        record_y;
        record_psi;
        record_phy;
    end
    methods
        function self=KinematicModel(x, y, psi, v, dt, L)
            self.x=x;
            self.y=y;
            self.psi=psi;
            self.v = v;
            self.L = L;
            % 实现是离散的模型
            self.dt = dt;
            self.record_x = [];
            self.record_y= [];
            self.record_psi= [];
            self.record_phy= [];
        end
end
end