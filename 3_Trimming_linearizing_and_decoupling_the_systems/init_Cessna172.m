%% Init data for Cessna 172
% source: https://doi.org/10.1371/journal.pone.0165017
% source: https://www.atlantis-press.com/article/2629.pdf

% Wing Data
S = 16.1651;            % ref wing area, in [m]
b = 10.9118;            % wing span, in [m]
c = 1.4935;             % mean aerodynamic chord, in [m]

% Mass properties Data
Mass_kg = 1043.3;
CG_mac = 0.3;
Ixx = 1285.3;   % in [kgm**2]
Iyy = 1824.9;   % in [kgm**2]
Izz = 2666.9;   % in [kgm**2]
Ixz = 0;        % in [kgm**2]
y_CG = 0.0;     % in [m] (+ is to the right of ref. plane)
z_CG = 0.2;     % in [m] (+ is below the ref. plane)
g = 9.80665;    % in [m/s**2]
InertiaTensor = [Ixx 0.0 -Ixz; 0.0 Iyy 0.0; -Ixz 0.0 Izz];

% Engine Data
V_ref = 51.4;       % in [m/s]
rho_ref = 1.225;    % in [kg/m**3]
nv = -1.0;
nrho = 0.75;
Alpha_F = 1.0;      % in [deg]
X_F = 1.0;          % in [m]
Z_F = 0.0;          % in [m]
T_max = 2070.0;     % in [N]
use_linear_thrust = 0;

%% Aerodynamic Data %%

% Lift Coefficient Derivatives
CL0 = 0.31;
CL_Alpha = 5.143;       % in [1/rad]
CL_Elev = 0.43;         % in [1/rad]
CL_AlphaDot = 0.0;      % in [s/rad]
CL_q = 3.9;             % in [s/rad]

% Drag Coefficient Derivatives
CD0 = 0.031;
CD_Alpha = 0.13;        % in [1/rad]
CD_Elev = 0.06;         % in [1/rad]

% Pitching Moment Derivatives
Cm0 = -0.015;
Cm_Alpha = -0.89;       % in [1/rad]
Cm_Elev = -1.28;        % in [1/rad]
Cm_AlphaDot = -7.27;    % in [s/rad]
Cm_q = -12.4;           % in [s/rad]

% Side Force Coefficient Derivatives
CY_Beta = -0.31;        % in [1/rad]
CY_Ail = 0.0;           % in [1/rad]
CY_Rud = 0.187;         % in [1/rad]
CY_p = -0.037;          % in [s/rad]
CY_r = 0.21;            % in [s/rad]

% Rolling Moment Derivatives
Cl_Beta = -0.089;       % in [1/rad]
Cl_Ail = -0.178;        % in [1/rad]
Cl_Rud = 0.0147;        % in [1/rad]
Cl_p = -0.47;           % in [s/rad]
Cl_r = 0.096;           % in [s/rad]

% Yawing moment Derivatives
Cn_Beta = 0.065;        % in [1/rad]
Cn_Ail = -0.053;        % in [1/rad]
Cn_Rud = -0.0657;       % in [1/rad]
Cn_p = -0.03;           % in [s/rad]
Cn_r = -0.099;          % in [s/rad]
