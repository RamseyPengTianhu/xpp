% function a = para_load
% b =1
% c=2
% d=4
% fprintf('parameter loaded\n');
% end



stick_mass = 0.00001;

trunk_width = 0.194;
trunk_length = 0.267;
trunk_height = 0.114;
hip_radius = 0.046;
hip_length = 0.04;
thigh_shoulder_radius = 0.041;
thigh_shoulder_length = 0.032;
thigh_width = 0.0245;
thigh_height = 0.034;
calf_width = 0.016;
calf_height = 0.016;
foot_radius = 0.02;
stick_radius = 0.01;
stick_length = 0.2;

%kinematic value 
thigh_offset = 0.08505;
thigh_length = 0.2;
calf_length = 0.2;

%leg offset from trunk center value
leg_offset_x = 0.183;
leg_offset_y = 0.047;
trunk_offset_z = 0.01675;
hip_offset = 0.065;

%joint limits -->
damping = 0;
friction = 0;
hip_max = 46;
hip_min = -46;
hip_velocity_max = 52.4;
hip_torque_max = 20;
thigh_max = 240;
thigh_min = -60;
thigh_velocity_max = 28.6;
thigh_torque_max = 55;
calf_max = -52.5;
calf_min = -154.5;
calf_velocity_max = 28.6;
calf_torque_max = 55;

% dynamics inertial value -->
% trunk -->
trunk_mass = 4.713;
trunk_com_x = 0.012731;
trunk_com_y = 0.002186;
trunk_com_z = 0.000515;
trunk_ixx = 0.016839930;
trunk_ixy = 0.000083902;
trunk_ixz = 0.000597679;
trunk_iyy = 0.056579028;
trunk_iyz = 0.000025134;
trunk_izz = 0.064713601;

%-- hip (left front) -->
hip_mass = 0.696;
hip_com_x = -0.003311;
hip_com_y = 0.000635;
hip_com_z = 0.000031;
hip_ixx = 0.000469246*1e6;
hip_ixy = -0.000009409;
hip_ixz = -0.000000342;
hip_iyy = 0.000807490;
hip_iyz = -0.000000466;
hip_izz = 0.000552929;

%    <!-- thigh -->
thigh_mass = 1.013;
thigh_com_x = -0.003237;
thigh_com_y = -0.022327;
thigh_com_z = -0.027326;
thigh_ixx = 0.005529065;
thigh_ixy = 0.000004825;
thigh_ixz = 0.000343869;
thigh_iyy = 0.005139339;
thigh_iyz = 0.000022448;
thigh_izz = 0.001367788;

%     <!-- calf -->
calf_mass = 0.166;
calf_com_x = 0.006435;
calf_com_y = 0.0;
calf_com_z = -0.107388;
calf_ixx = 0.002997972;
calf_ixy = 0.0;
calf_ixz = -0.000141163;
calf_iyy = 0.003014022;
calf_iyz = 0.0;
calf_izz = 0.000032426;

%     <!-- foot -->
foot_mass = 0.06;
