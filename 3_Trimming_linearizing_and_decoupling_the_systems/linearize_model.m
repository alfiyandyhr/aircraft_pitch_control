% Initialization of Cessna 172
init_Cessna172;

% Trim at cruise at altitude 5000 ft
% ZI_m = -0.3048*5000; Gamma_rad = 0.0;
[op_cruise,opreport_cruise] = trim_cond(-0.3048*5000,0.0);
x_cruise = getstatestruct(opreport_cruise); % states at cruise
u_cruise = getinputstruct(opreport_cruise); % inputs at cruise
[u_elev0, u_ail0, u_rud0, u_throt0] = u_cruise.signals.values;

% Load non-linear aircraft dynamic model
nlsys = load_system("AircraftDynamicSimNonLinear");

state_path = 'AircraftDynamicSimNonLinear/AircraftSixDOFDynamics/AircraftSixDOFModel/Equations_of_Motion/';

StateOrder = {append(state_path,'InertialData/XI_m');
              append(state_path,'InertialData/YI_m');
              append(state_path,'InertialData/ZI_m');
              append(state_path,'EulerAngles/PHI_rad');
              append(state_path,'EulerAngles/THETA_rad');
              append(state_path,'EulerAngles/PSI_rad');
              append(state_path,'BodyVelocities/u_mps');
              append(state_path,'BodyVelocities/v_mps');
              append(state_path,'BodyVelocities/w_mps');
              append(state_path,'BodyRates/p_radps');
              append(state_path,'BodyRates/q_radps');
              append(state_path,'BodyRates/r_radps')};

sys = linearize("AircraftDynamicSimNonLinear",opreport_cruise,"StateOrder",StateOrder);

syms x y z phi theta psi u v w p q r;
syms x_dot y_dot z_dot phi_dot theta_dot psi_dot u_dot v_dot w_dot p_dot q_dot r_dot;
syms delta_e delta_a delta_r delta_t;
syms alpha beta gamma zeta v_TAS rho;

states = [x;y;z;phi;theta;psi;u;v;w;p;q;r];
states_dot = [x_dot;y_dot;z_dot;
              phi_dot;theta_dot;psi_dot;
              u_dot;v_dot;w_dot;
              p_dot;q_dot;r_dot];
inputs = [delta_e;delta_a;delta_r;delta_t];
outputs = [x;y;z;phi;theta;psi;u;v;w;p;q;r;alpha;beta;gamma;zeta;v_TAS;rho];

A = round(sys.A,4);
B = round(sys.B,4);
C = round(sys.C,4);
D = round(sys.D,4);

A_latex = latex(vpa(sym(A),4));
B_latex = latex(vpa(sym(B),4));
C_latex = latex(vpa(sym(C),4));
D_latex = latex(vpa(sym(D),4));

states_dot_ = A*states + B*inputs;
outputs_ = C*states + D*inputs;
states_dot_ = vpa(states_dot_,4);
outputs_ = vpa(outputs_,4);

states_dot_latex = latex(states_dot_);
outputs_latex = latex(outputs_);

states_diff_eq = states_dot == states_dot_;
outputs_eq = outputs == outputs_;

states_diff_eq = vpa(states_diff_eq,4);
outputs_eq = vpa(outputs_eq,4);

% Aircraft longitudinal dynamics
longitudinal_eqs = vpa(states_diff_eq([1,3,5,7,9,11]),4);
longitudinal_eqs_latex = latex(longitudinal_eqs);
A_long = [0, 0, 0, 1, 0, 0;
          0, 0, -62.39, 0, 1, 0;
          0, 0, 0, 0, 0, 1;
          0, -0.0001, -9.807, -0.0477, 0.2388, 0;
          0, -0.0022, 0, -0.3152, -2.64, 60.9;
          0, 0, 0, 0.0005, -0.2494, -3.971];
B_long = [0, 0;
          0, 0;
          0, 0;
          1.91, 1.462;
          -13.69, 0.0255;
          -33.99, -0.0146];
C_long = [1, 0, 0, 0, 0, 0;       % x
          0, 1, 0, 0, 0, 0;       % z
          0, 0, 1, 0, 0, 0;       % theta
          0, 0, 0, 1, 0, 0;       % u
          0, 0, 0, 0, 1, 0;       % w
          0, 0, 0, 0, 0, 1;       % q
          0, 0, 0, 0, 0.016, 0;   % alpha
          0, 0, 1, 0,-0.016, 0;   % gamma
          0, 0, 0, 1, 0, 0;       % v_TAS
          0, 0.0002, 0, 0, 0, 0]; % rho
D_long = [0, 0; 0, 0; 0, 0; 0, 0; 0, 0;
          0, 0; 0, 0; 0, 0; 0, 0; 0, 0];
long_sys = ss(A_long,B_long,C_long,D_long);

% Aircraft lateral dynamics
lateral_eqs = vpa(states_diff_eq([2,4,6,8,10,12]),4);
lateral_eqs_latex = latex(lateral_eqs);
A_lat = [0, 0, 62.39, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 9.807, 0, -0.1582, -0.103, -61.8;
         0, 0, 0, -0.3765, -11.57, 2.272;
         0, 0, 0, 0.137, -0.3595, -1.159];
B_lat = [0, 0;
         0, 0;
         0, 0;
         0, -5.953;
         -50.19, 3.178;
         -7.202, -8.754];
C_lat = [1, 0, 0, 0, 0, 0;       % y
         0, 1, 0, 0, 0, 0;       % phi
         0, 0, 1, 0, 0, 0;       % psi
         0, 0, 0, 1, 0, 0;       % v
         0, 0, 0, 0, 1, 0;       % p
         0, 0, 0, 0, 0, 1;       % r
         0, 0, 0, 0.016, 0, 0;   % beta
         0, 0, 1, 0.016, 0, 0];  % zeta
D_lat = [0, 0; 0, 0; 0, 0; 0, 0;
         0, 0; 0, 0; 0, 0; 0, 0;];
lat_sys = ss(A_lat,B_lat,C_lat,D_lat);

% Elevator doublet simulation (non-linear vs linear)
TF = 60;           % Final time
t = [0 10 10.01 12 12.01 14 14.01 TF]; % Simulation times - [s]
d_e = 1.0/180*pi;
ut_input = [0.000 u_elev0     u_ail0 u_rud0 u_throt0;
            10.00 u_elev0     u_ail0 u_rud0 u_throt0;
            10.01 u_elev0+d_e u_ail0 u_rud0 u_throt0;
            12.00 u_elev0+d_e u_ail0 u_rud0 u_throt0;
            12.01 u_elev0-d_e u_ail0 u_rud0 u_throt0;
            14.00 u_elev0-d_e u_ail0 u_rud0 u_throt0;
            14.01 u_elev0     u_ail0 u_rud0 u_throt0;
            TF  u_elev0 u_ail0 u_rud0 u_throt0;];
simIn = Simulink.SimulationInput('AircraftDynamicSimNonLinear');
simIn = setModelParameter(simIn, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
simIn = setInitialState(simIn, getstatestruct(opreport_cruise));
simIn = setExternalInput(simIn, ut_input);
simOut = sim(simIn);
tout1 = simOut.tout;
xout1 = simOut.xout;
yout1 = simOut.yout;

t2 = 0:0.01:TF;
u21 = zeros(size(t2));
u21(t2<=12 & t2>=10) = d_e;
u21(t2<=14 & t2>=12) = -d_e;
u22 = zeros(size(t2));
xinit2 = zeros(1,6);
[yout2, tout2, xout2] = lsim(long_sys,[u21;u22],t2,xinit2);

% Rudder doublet simulation (non-linear vs linear)
TF = 60;           % Final time
t = [0 10 10.01 12 12.01 14 14.01 TF]; % Simulation times - [s]
d_r = 1.0/180*pi;
ut_input2 = [0.000 u_elev0 u_ail0 u_rud0     u_throt0;
             10.00 u_elev0 u_ail0 u_rud0     u_throt0;
             10.01 u_elev0 u_ail0 u_rud0+d_r u_throt0;
             12.00 u_elev0 u_ail0 u_rud0+d_r u_throt0;
             12.01 u_elev0 u_ail0 u_rud0-d_r u_throt0;
             14.00 u_elev0 u_ail0 u_rud0-d_r u_throt0;
             14.01 u_elev0 u_ail0 u_rud0     u_throt0;
             TF    u_elev0 u_ail0 u_rud0     u_throt0;];
simIn2 = Simulink.SimulationInput('AircraftDynamicSimNonLinear');
simIn2 = setModelParameter(simIn, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
simIn2 = setInitialState(simIn, getstatestruct(opreport_cruise));
simIn2 = setExternalInput(simIn, ut_input2);
simOut2 = sim(simIn2);
tout3 = simOut2.tout;
xout3 = simOut2.xout;
yout3 = simOut2.yout;

t4 = 0:0.01:TF;
u41 = zeros(size(t4));
u42 = zeros(size(t4));
u42(t4<=12 & t4>=10) = d_r;
u42(t4<=14 & t4>=12) = -d_r;
xinit4 = zeros(1,6);
[yout4, tout4, xout4] = lsim(lat_sys,[u41;u42],t4,xinit4);

% Parsing data for elevator doublet on non-linear system
XI_1       = yout1{1}.Values.Data(:,1);
ZI_1       = yout1{1}.Values.Data(:,3);
Theta_1    = yout1{2}.Values.Data(:,2);
u_1        = yout1{3}.Values.Data(:,1);
w_1        = yout1{3}.Values.Data(:,3);
q_1        = yout1{4}.Values.Data(:,2); 
Alpha_1    = yout1{5}.Values.Data;
Gamma_1    = yout1{7}.Values.Data;
v_TAS_1    = yout1{9}.Values.Data;
Density_1  = yout1{10}.Values.Data;

% Parsing data for eleavtor doublet on linear system
XI_2       = yout2(:,1) + 0.0;
ZI_2       = yout2(:,2) + -1524;
Theta_2    = yout2(:,3) + 0.0;
u_2        = yout2(:,4) + 62.3866;
w_2        = yout2(:,5) + 0.0;
q_2        = yout2(:,6) + 0.0;
Alpha_2    = yout2(:,7) + 0.0;
Gamma_2    = yout2(:,8) + 0.0;
v_TAS_2    = yout2(:,9) + 62.3866;
Density_2  = yout2(:,10) + 1.0557;

% Parsing data for rudder doublet on non-linear system
YI_1       = yout3{1}.Values.Data(:,2);
Phi_1      = yout3{2}.Values.Data(:,1);
Psi_1      = yout3{2}.Values.Data(:,3);
v_1        = yout3{3}.Values.Data(:,2);
p_1        = yout3{4}.Values.Data(:,1);  
r_1        = yout3{4}.Values.Data(:,3);
Beta_1     = yout3{6}.Values.Data;
Zeta_1     = yout3{8}.Values.Data;

% Parsing data for rudder doublet on linear system
YI_2       = yout4(:,1) + 0.0;
Phi_2      = yout4(:,2) + 0.0;
Psi_2      = yout4(:,3) + 0.0;
v_2        = yout4(:,4) + 0.0;
p_2        = yout4(:,5) + 0.0;
r_2        = yout4(:,6) + 0.0;
Beta_2     = yout4(:,7) + 0.0;
Zeta_2     = yout4(:,8) + 0.0;

% Plot
figure(1);
tiledlayout(3,3);
nexttile; plot(t, ut_input(:,2)*180/pi, t2, (u21+u_elev0)*180/pi); xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid;
nexttile; plot(t, ut_input(:,5), t2, u22+u_throt0); xlabel('Time - [s]'); ylabel('Throttle'); grid;
nexttile; plot(tout3, round(Theta_1*180/pi,2), tout2, round(Theta_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid;
nexttile; plot(tout3, round(v_TAS_1*1.94384,2),tout2, round(v_TAS_2*1.94384,2)); xlabel('Time - [s]'); ylabel('True Airspeed - [knot]'); grid;
nexttile; plot(tout3, round(-ZI_1*3.28084,2), tout2, round(-ZI_2*3.28084,2)); xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid;
nexttile; plot(tout3, round(Density_1,6), tout2, round(Density_2,6)); xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid;
nexttile; plot(tout3, round(Gamma_1*180/pi,2), tout2, round(Gamma_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid;
nexttile; plot(tout3, round(Alpha_1*180/pi,2), tout2, round(Alpha_2*180/pi,2)); xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid;
nexttile; plot(tout3, round(q_1*180/pi,2), tout2, round(q_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid;
l1 = legend('Non-linear','Linear','Orientation','horizontal','fontsize',14);
l1.Layout.Tile = 'north';
sgtitle('Longitudinal dynamics under elevator doublet','fontsize',18);

figure(2);
tiledlayout(3,3);
nexttile; plot(t, ut_input2(:,4)*180/pi, t2, (u42+u_rud0)*180/pi); xlabel('Time - [s]'); ylabel('Rudder - [deg]'); grid;
nexttile; plot(t, ut_input2(:,3)*180/pi, t2, (u41+u_ail0)*180/pi); xlabel('Time - [s]'); ylabel('Aileron - [deg]'); grid;
nexttile; plot(tout1, round(Psi_1*180/pi,2), tout4, round(Psi_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Yaw angle - [deg]'); grid;
nexttile; plot(tout1, round(Phi_1*180/pi,2),tout4, round(Phi_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Roll angle - [deg]'); grid;
nexttile; plot(tout1, round(p_1*180/pi,2), tout4, round(p_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Roll rate - [degps]'); grid;
nexttile; plot(tout1, round(r_1*180/pi,2), tout4, round(r_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Yaw rate - [degps]'); grid;
nexttile; plot(tout1, round(Zeta_1*180/pi,2), tout4, round(Zeta_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Track angle - [deg]'); grid;
nexttile; plot(tout1, round(Beta_1*180/pi,2), tout4, round(Beta_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Sideslip angle - [deg]'); grid;
nexttile; plot(tout1, round(YI_1,2), tout4, round(YI_2,2)); xlabel('Time - [s]'); ylabel('Inertial Position Y - [m]'); grid;
l2 = legend('Non-linear','Linear','Orientation','horizontal','fontsize',14);
l2.Layout.Tile = 'north';
sgtitle('Lateral dynamics under rudder doublet','fontsize',18);