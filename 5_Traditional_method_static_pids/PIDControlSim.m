% Aircraft longitudinal dynamics
% Assumption: SISO (elevator to pitch)
A = [0, 0, 0, 1, 0, 0;
     0, 0, -62.39, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, -0.0001, -9.807, -0.0477, 0.2388, 0;
     0, -0.0022, 0, -0.3152, -2.64, 60.9;
     0, 0, 0, 0.0005, -0.2494, -3.971];
B = [0; 0; 0; 1.91; -13.69; -33.99];
C = [1, 0, 0, 0, 0, 0;         % x
     0, 1, 0, 0, 0, 0;         % z
     0, 0, 1, 0, 0, 0;         % theta
     0, 0, 0, 1, 0, 0;         % u
     0, 0, 0, 0, 1, 0;         % w
     0, 0, 0, 0, 0, 1;         % q
     0, 0, 0, 0, 0.016, 0;     % alpha
     0, 0, 1, 0,-0.016, 0;     % gamma
     0, 0, 0, 1, 0, 0;         % v_TAS
     0, 0.0002, 0, 0, 0, 0];   % rho
D = 0;

% PID Controller
Kp_list = [-1,-1,-1,-1,-1.3];
Ki_list = [-1,-0.8,-0.6,-0.3,-0.3];
Kd_list = [0,0,0,0,-0.1];
N = 100;

% Run simulation
r = 0.2; % desired pitch angle in rad
TF = 10; % final time

simOut = cell(1,size(Kp_list,2));
y = cell(1,size(Kp_list,2));
t = cell(1,size(Kp_list,2));
x = cell(1,size(Kp_list,2));
for i = 1:size(Kp_list,2)
    Kp = Kp_list(i);
    Ki = Ki_list(i);
    Kd = Kd_list(i);
    mdl = 'ClosedLoop_PID';
    simIn = Simulink.SimulationInput(mdl);
    simIn = setModelParameter(simIn, 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
    simIn = setExternalInput(simIn, [0 r; TF r]); 
    simOut{i} = sim(simIn);
    y{i} = simOut{i}.yout;
    t{i} = simOut{i}.tout;
    x{i} = simOut{i}.xout;
end

% Output initialization
Elevator = cell(1,size(Kp_list,2));
Throttle = cell(1,size(Kp_list,2));
XI = cell(1,size(Kp_list,2));
ZI = cell(1,size(Kp_list,2));
Theta = cell(1,size(Kp_list,2));
u = cell(1,size(Kp_list,2));
w = cell(1,size(Kp_list,2));
q = cell(1,size(Kp_list,2));
Alpha = cell(1,size(Kp_list,2));
Gamma = cell(1,size(Kp_list,2));
v_TAS = cell(1,size(Kp_list,2));
Density = cell(1,size(Kp_list,2));

% Parsing output
for i = 1:size(Kp_list,2)
    t_i = t{i};
    y_i = y{i};
    Elevator{i} = simOut{i}.Elevator_rad.Data *180/pi; % in deg
    Throttle{i} = 0.6792 * ones(size(t_i,1),1);
    XI{i}       = y_i{1}.Values.Data;
    ZI{i}       = y_i{2}.Values.Data * (-3.28084); % in Altitude ft
    Theta{i}    = y_i{3}.Values.Data *180/pi; % in deg
    u{i}        = y_i{4}.Values.Data;
    w{i}        = y_i{5}.Values.Data;
    q{i}        = y_i{6}.Values.Data *180/pi; % in deg/s
    Alpha{i}    = y_i{7}.Values.Data *180/pi; % in deg
    Gamma{i}    = y_i{8}.Values.Data *180/pi; % in deg
    v_TAS{i}    = y_i{9}.Values.Data *1.94384; % knot
    Density{i}  = y_i{10}.Values.Data;
end

% PID controller manual tuning with time filter Tf=100
% Design:
% rise time <2s
% settling time <10s
% overshoot <10%
% steady-state error <2%
info_list = zeros(5,7); % [Kp Ki Kd risetime settlingtime overshoot steadystateerror]
for i = 1:size(Kp_list,2)
    info_list(i,1)=Kp_list(i);
    info_list(i,2)=Ki_list(i);
    info_list(i,3)=Kd_list(i);
    info = stepinfo(Theta{i},t{i});
    info_list(i,4)=info.RiseTime;
    info_list(i,5)=info.SettlingTime;
    info_list(i,6)=info.Overshoot;
    info_list(i,7)=abs((Theta{i}(end)-r*180/pi)/(r*180/pi))*100;
end

info_list

% Input Output Plot
figure(1);
tiledlayout(3,3);
nexttile;
plot(t{1}, Elevator{1}, t{2}, Elevator{2}, t{3}, Elevator{3}, t{4}, Elevator{4}, t{5}, Elevator{5});
xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid; xlim([0 2]); ylim([-35 10]);
nexttile;
plot(t{1}, Throttle{1}, t{2}, Throttle{2}, t{3}, Throttle{3}, t{4}, Throttle{4}, t{5}, Throttle{5});
xlabel('Time - [s]'); ylabel('Throttle'); grid; xlim([0 4]); ylim([-0.5 1]);
nexttile;
plot(t{1}, Theta{1}, t{2}, Theta{2}, t{3}, Theta{3}, t{4}, Theta{4}, t{5}, Theta{5});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 4]);
nexttile;
plot(t{1}, v_TAS{1}, t{2}, v_TAS{2}, t{3}, v_TAS{3}, t{4}, v_TAS{4}, t{5}, v_TAS{5});
xlabel('Time - [s]'); ylabel('True Airspeed - [knot]'); grid; xlim([0 4]); ylim([109 122]);
nexttile;
plot(t{1}, ZI{1}, t{2}, ZI{2}, t{3}, ZI{3}, t{4}, ZI{4}, t{5}, ZI{5});
xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid; xlim([0 4]); ylim([4990 5150]);
nexttile;
plot(t{1}, Density{1}, t{2}, Density{2}, t{3}, Density{3}, t{4}, Density{4}, t{5}, Density{5});
xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid; xlim([0 4]);
nexttile;
plot(t{1}, Gamma{1}, t{2}, Gamma{2}, t{3}, Gamma{3}, t{4}, Gamma{4}, t{5}, Gamma{5});
xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid; xlim([0 4]); ylim([-1 14]);
nexttile;
plot(t{1}, Alpha{1}, t{2}, Alpha{2}, t{3}, Alpha{3}, t{4}, Alpha{4}, t{5}, Alpha{5});
xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid; xlim([0 4]); ylim([-1 9]);
nexttile;
plot(t{1}, q{1}, t{2}, q{2}, t{3}, q{3}, t{4}, q{4}, t{5}, q{5});
xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid; xlim([0 2]); ylim([-15 48]);
l1 = legend('PID\_1','PID\_2','PID\_3','PID\_4','PID\_5','Orientation','horizontal','fontsize',12);
l1.Layout.Tile = 'north';
sgtitle('Pitch control input-output via PID with different static gains','fontsize',18);

% Tracking performance
figure(2);
plot(t{1}, Theta{1}, t{2}, Theta{2}, t{3}, Theta{3}, t{4}, Theta{4}, t{5}, Theta{5});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 6]); ylim([-1 15]);
legend('PID-1','PID-2','PID-3','PID-4','PID-5','Location','SouthEast');
title('Pitch control via PID with different static gains')