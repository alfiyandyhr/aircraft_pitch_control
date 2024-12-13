% Initialization of Cessna 172
init_Cessna172;

% Trim at cruise at altitude 5000 ft
% ZI_m = -0.3048*5000; Gamma_rad = 0.0;
[op_cruise,opreport_cruise] = trim_cond(-0.3048*5000,0.0);
x_cruise = getstatestruct(opreport_cruise); % states at cruise
u_cruise = getinputstruct(opreport_cruise); % inputs at cruise

% Trim at climb at altitude 5000 ft
% ZI_m = -0.3048*5000; Gamma_rad = 1.0 /180*pi
[op_climb,opreport_climb] = trim_cond(-0.3048*5000,1.0/180*pi);
x_climb = getstatestruct(opreport_climb); % states at climb
u_climb = getinputstruct(opreport_climb); % inputs at climb

% Simulation times - [s]
TF = 500;
t = [0 99 100 TF];

% Constant input for constant cruise
ut_cruise = zeros(size(t,2), size(u_cruise.signals,2)+1);
for i=1:size(t,2)
    ut_cruise(i,1) = t(i);
    for j=1:size(u_cruise.signals,2)
        ut_cruise(i,j+1) = u_cruise.signals(j).values;
    end
end

% Constant input for constant climb
ut_climb = zeros(size(t,2), size(u_climb.signals,2)+1);
for i=1:size(t,2)
    ut_climb(i,1) = t(i);
    for j=1:size(u_climb.signals,2)
        ut_climb(i,j+1) = u_climb.signals(j).values;
    end
end

% Transition input from cruise to climb
ut_transition = zeros(size(t,2), size(u_cruise.signals,2)+1);
for i=1:size(t,2)
    ut_transition(i,1) = t(i);
end
ut_transition(1,2)=ut_cruise(1,2);%t=0
ut_transition(1,5)=ut_cruise(1,5);
ut_transition(2,2)=ut_cruise(2,2);%t=99
ut_transition(2,5)=ut_cruise(2,5);
ut_transition(3,2)=ut_cruise(3,2)-2.0*pi/180;%t=100
ut_transition(3,5)=ut_cruise(3,5);
ut_transition(4,2)=ut_cruise(4,2)-2.0*pi/180;%t=TF
ut_transition(4,5)=ut_cruise(4,5);

% Simulation commands for cruise
mdl = 'AircraftDynamicSimNonLinear';
simIn1 = Simulink.SimulationInput(mdl);
simIn1 = setModelParameter(simIn1, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
simIn1 = setInitialState(simIn1, x_cruise);
simIn1 = setExternalInput(simIn1, ut_cruise);
simOut1 = sim(simIn1);
tout1 = simOut1.tout;
xout1 = simOut1.xout;
yout1 = simOut1.yout;

% Simulation commands for climb
mdl = 'AircraftDynamicSimNonLinear';
simIn2 = Simulink.SimulationInput(mdl);
simIn2 = setModelParameter(simIn2, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
simIn2 = setInitialState(simIn2, x_climb);
simIn2 = setExternalInput(simIn2, ut_climb);
simOut2 = sim(simIn2);
tout2 = simOut2.tout;
xout2 = simOut2.xout;
yout2 = simOut2.yout;

% Simulation commands for transition
mdl = 'AircraftDynamicSimNonLinear';
simIn3 = Simulink.SimulationInput(mdl);
simIn3 = setModelParameter(simIn3, 'Solver','ode4', 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
simIn3 = setInitialState(simIn3, x_cruise);
simIn3 = setExternalInput(simIn3, ut_transition);
simOut3 = sim(simIn3);
tout3 = simOut3.tout;
xout3 = simOut3.xout;
yout3 = simOut3.yout;

% Parsing data for cruise
XI_1       = yout1{1}.Values.Data(:,1);
YI_1       = yout1{1}.Values.Data(:,2);
ZI_1       = yout1{1}.Values.Data(:,3);
Phi_1      = yout1{2}.Values.Data(:,1);
Theta_1    = yout1{2}.Values.Data(:,2);
Psi_1      = yout1{2}.Values.Data(:,3);
u_1        = yout1{3}.Values.Data(:,1);
v_1        = yout1{3}.Values.Data(:,2);
w_1        = yout1{3}.Values.Data(:,3);
p_1        = yout1{4}.Values.Data(:,1); 
q_1        = yout1{4}.Values.Data(:,2); 
r_1        = yout1{4}.Values.Data(:,3);
Alpha_1    = yout1{5}.Values.Data;
Beta_1     = yout1{6}.Values.Data;
Gamma_1    = yout1{7}.Values.Data;
Zeta_1     = yout1{8}.Values.Data;
v_TAS_1    = yout1{9}.Values.Data;
Density_1  = yout1{10}.Values.Data;

% Parsing data for climb
XI_2       = yout2{1}.Values.Data(:,1);
YI_2       = yout2{1}.Values.Data(:,2);
ZI_2       = yout2{1}.Values.Data(:,3);
Phi_2      = yout2{2}.Values.Data(:,1);
Theta_2    = yout2{2}.Values.Data(:,2);
Psi_2      = yout2{2}.Values.Data(:,3);
u_2        = yout2{3}.Values.Data(:,1);
v_2        = yout2{3}.Values.Data(:,2);
w_2        = yout2{3}.Values.Data(:,3);
p_2        = yout2{4}.Values.Data(:,1); 
q_2        = yout2{4}.Values.Data(:,2); 
r_2        = yout2{4}.Values.Data(:,3);
Alpha_2    = yout2{5}.Values.Data;
Beta_2     = yout2{6}.Values.Data;
Gamma_2    = yout2{7}.Values.Data;
Zeta_2     = yout2{8}.Values.Data;
v_TAS_2    = yout2{9}.Values.Data;
Density_2  = yout2{10}.Values.Data;

% Parsing data for transition
XI_3       = yout3{1}.Values.Data(:,1);
YI_3       = yout3{1}.Values.Data(:,2);
ZI_3       = yout3{1}.Values.Data(:,3);
Phi_3      = yout3{2}.Values.Data(:,1);
Theta_3    = yout3{2}.Values.Data(:,2);
Psi_3      = yout3{2}.Values.Data(:,3);
u_3        = yout3{3}.Values.Data(:,1);
v_3        = yout3{3}.Values.Data(:,2);
w_3        = yout3{3}.Values.Data(:,3);
p_3        = yout3{4}.Values.Data(:,1); 
q_3        = yout3{4}.Values.Data(:,2); 
r_3        = yout3{4}.Values.Data(:,3);
Alpha_3    = yout3{5}.Values.Data;
Beta_3     = yout3{6}.Values.Data;
Gamma_3    = yout3{7}.Values.Data;
Zeta_3     = yout3{8}.Values.Data;
v_TAS_3    = yout3{9}.Values.Data;
Density_3  = yout3{10}.Values.Data;

% Plot

figure(1);
sgtitle('Steady-cruise condition');
subplot(3,3,1); plot(t, ut_cruise(:,2)*180/pi); xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid;
subplot(3,3,2); plot(t, ut_cruise(:,5)); xlabel('Time - [s]'); ylabel('Throttle'); grid;
subplot(3,3,3); plot(tout1, round(Theta_1*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid;
subplot(3,3,4); plot(tout1, round(v_TAS_1*1.94384,2)); xlabel('Time - [s]'); ylabel('KTAS'); grid;
subplot(3,3,5); plot(tout1, round(-ZI_1*3.28084,2)); xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid;
subplot(3,3,6); plot(tout1, round(Density_1,2)); xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid;
subplot(3,3,7); plot(tout1, round(Gamma_1*180/pi,2)); xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid;
subplot(3,3,8); plot(tout1, round(Alpha_1*180/pi,2)); xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid;
subplot(3,3,9); plot(tout1, round(q_1*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid;

figure(2);
sgtitle('Steady-climb condition');
subplot(3,3,1); plot(t, ut_climb(:,2)*180/pi); xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid;
subplot(3,3,2); plot(t, ut_climb(:,5)); xlabel('Time - [s]'); ylabel('Throttle'); grid;
subplot(3,3,3); plot(tout1, Theta_2*180/pi); xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid;
subplot(3,3,4); plot(tout1, round(v_TAS_2*1.94384,2)); xlabel('Time - [s]'); ylabel('KTAS'); grid;
subplot(3,3,5); plot(tout1, round(-ZI_2*3.28084,2)); xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid;
subplot(3,3,6); plot(tout1, Density_2); xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid;
subplot(3,3,7); plot(tout1, Gamma_2*180/pi); xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid;
subplot(3,3,8); plot(tout1, round(Alpha_2*180/pi,2)); xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid;
subplot(3,3,9); plot(tout1, round(q_2*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid;

figure(3);
sgtitle('Cruise-to-climb transition');
subplot(3,3,1); plot(t, ut_transition(:,2)*180/pi); xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid;
subplot(3,3,2); plot(t, ut_transition(:,5)); xlabel('Time - [s]'); ylabel('Throttle'); grid;
subplot(3,3,3); plot(tout1, round(Theta_3*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid;
subplot(3,3,4); plot(tout1, round(v_TAS_3*1.94384,2)); xlabel('Time - [s]'); ylabel('KTAS'); grid;
subplot(3,3,5); plot(tout1, round(-ZI_3*3.28084,2)); xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid;
subplot(3,3,6); plot(tout1, Density_3); xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid;
subplot(3,3,7); plot(tout1, round(Gamma_3*180/pi,2)); xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid;
subplot(3,3,8); plot(tout1, round(Alpha_3*180/pi,2)); xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid;
subplot(3,3,9); plot(tout1, round(q_3*180/pi,2)); xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid;