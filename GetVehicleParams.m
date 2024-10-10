function VehicleParams = GetVehicleParams()

VehicleParams.Lw = 2 * 2; % wheel base
VehicleParams.Lf = 0.96 * 2; % front hang
VehicleParams.Lr = 0.929 * 2; % rear hang
VehicleParams.Lb = 1.942 * 2; % width
VehicleParams.Htr=1.5;
VehicleParams.Ll = VehicleParams.Lw + VehicleParams.Lf + VehicleParams.Lr; % length

VehicleParams.f2x = 1/4 * (3*VehicleParams.Lw + 3*VehicleParams.Lf - VehicleParams.Lr);
VehicleParams.r2x = 1/4 * (VehicleParams.Lw + VehicleParams.Lf - 3*VehicleParams.Lr);
VehicleParams.radius = 1/2 * sqrt((VehicleParams.Lw + VehicleParams.Lf + VehicleParams.Lr) ^ 2 / 4 + VehicleParams.Lb ^ 2);

VehicleParams.a_max = 0.5;
VehicleParams.v_max = 2.5;
VehicleParams.phi_max = 0.7;
VehicleParams.omega_max = 0.5;

% for wheel visualization
VehicleParams.wheel_radius = 0.32*2;
VehicleParams.wheel_width = 0.22*2;

end