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

% Time filter
N = 100;

% Actor net architectures
n_neurons_list = [64, 128, 256];
batchsize_list = [64, 128, 256];

% Run simulation - run only once !!!
r = 0.2; % desired pitch angle in rad
TF = 10; % final time
run_sim = true;
if run_sim
    display('Note: dont forget to set "run_sim=false" on line 32 so that you run it once!');
    simOut = cell(1,9);
    y = cell(1,9);
    t = cell(1,9);
    x = cell(1,9);
    for i = 1:3
        for j = 1:3
            mdl = 'ClosedLoop_AdaptivePID_N' + string(n_neurons_list(i)) + '_B' + string(batchsize_list(j));
            simIn = Simulink.SimulationInput(mdl);
            simIn = setModelParameter(simIn, 'FixedStep','0.01', 'StopTime','TF', 'SaveState','on');
            simIn = setExternalInput(simIn, [0 r; TF r]);
            k = i*3-3+j;
            simOut{k} = sim(simIn);
            y{k} = simOut{k}.yout;
            t{k} = simOut{k}.tout;
            x{k} = simOut{k}.xout;
        end
    end
end

% Output initialization
Elevator = cell(1,9);
Throttle = cell(1,9);
XI = cell(1,9);
ZI = cell(1,9);
Theta = cell(1,9);
u = cell(1,9);
w = cell(1,9);
q = cell(1,9);
Alpha = cell(1,9);
Gamma = cell(1,9);
v_TAS = cell(1,9);
Density = cell(1,9);
Kp_track = cell(1,9);
Ki_track = cell(1,9);
Kd_track = cell(1,9);

% Parsing output
for i = 1:9
    t_i = t{i};
    y_i = y{i};
    Elevator{i} = simOut{i}.Elevator_rad.Data *180/pi; % in deg
    Throttle{i} = 0.6792 * ones(size(t_i,1),1);
    XI{i}       = reshape(y_i{1}.Values.Data,[],1);
    ZI{i}       = reshape(y_i{2}.Values.Data * (-3.28084),[],1); % in Altitude ft
    Theta{i}    = reshape(y_i{3}.Values.Data *180/pi,[],1); % in deg
    u{i}        = reshape(y_i{4}.Values.Data,[],1);
    w{i}        = reshape(y_i{5}.Values.Data,[],1);
    q{i}        = reshape(y_i{6}.Values.Data *180/pi,[],1); % in deg/s
    Alpha{i}    = reshape(y_i{7}.Values.Data *180/pi,[],1); % in deg
    Gamma{i}    = reshape(y_i{8}.Values.Data *180/pi,[],1); % in deg
    v_TAS{i}    = reshape(y_i{9}.Values.Data *1.94384,[],1); % knot
    Density{i}  = reshape(y_i{10}.Values.Data,[],1);
    Kp_track{i} = simOut{i}.Kp.Data;
    Ki_track{i} = simOut{i}.Ki.Data;
    Kd_track{i} = simOut{i}.Kd.Data;
end

% PID controller adaptive tuning with time filter Tf=100
% Design:
% rise time <2s
% settling time <10s
% overshoot <10%
% steady-state error <2%
info_list = zeros(9,6); % [nneurons batchsize risetime settlingtime overshoot steadystateerror]
for i=1:3
    for j=1:3
        k = i*3-3+j;
        info_list(k,1)=n_neurons_list(i);
        info_list(k,2)=batchsize_list(j);
        info = stepinfo(Theta{k},t{k});
        info_list(k,3)=info.RiseTime;
        info_list(k,4)=info.SettlingTime;
        info_list(k,5)=info.Overshoot;
        info_list(k,6)=abs((Theta{k}(end)-r*180/pi)/(r*180/pi))*100;
    end
end

info_list

% Tracking performance
figure(1);
plot(t{1}, Theta{1}, t{5}, Theta{5}, t{7}, Theta{7});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 7]); ylim([-1 15]);
legend('N64\_B64','N128\_B128','N256\_B64','Location','SouthEast');
title('Pitch control via RL-based adaptive PIDs');

figure(2);
plot(t{3}, Theta{3}, t{6}, Theta{6}, t{9}, Theta{9});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 7]); ylim([-1 15]);
legend('N64\_B256','N128\_B256','N256\_B256','Location','SouthEast');
title('Pitch control via RL-based adaptive PIDs');

figure(3);
plot(t{2}, Theta{2}, t{4}, Theta{4}, t{8}, Theta{8});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 7]); ylim([-1 15]);
legend('N64\_B128','N128\_B64','N256\_B128','Location','SouthEast');
title('Pitch control via RL-based adaptive PIDs');

% Tracking the trajectory of Kp, Ki, Kd
figure(4);
tiledlayout(3,1);
nexttile;
plot(t{1}, Kp_track{1}, t{5}, Kp_track{5}, t{7}, Kp_track{7});
xlabel('Time - [s]'); ylabel('Kp'); grid; xlim([0 4]); ylim([-2.75, -1.5]);
nexttile;
plot(t{1}, Ki_track{1}, t{5}, Ki_track{5}, t{7}, Ki_track{7});
xlabel('Time - [s]'); ylabel('Ki'); grid; xlim([0 4]); ylim([-2.0, -0.75]);
nexttile;
plot(t{1}, Kd_track{1}, t{5}, Kd_track{5}, t{7}, Kd_track{7});
xlabel('Time - [s]'); ylabel('Kd'); grid; xlim([0 4]); ylim([-1.4, 0.3]);
l4 = legend('N64\_B64','N128\_B128','N256\_B64','Orientation','horizontal');
l4.Layout.Tile = 'north';
sgtitle('Adaptive PID gains trajectories');

figure(5);
tiledlayout(3,1);
nexttile;
plot(t{3}, Kp_track{3}, t{6}, Kp_track{6}, t{9}, Kp_track{9});
xlabel('Time - [s]'); ylabel('Kp'); grid; xlim([0 4]); ylim([-2.5, -1.25]);
nexttile;
plot(t{3}, Ki_track{3}, t{6}, Ki_track{6}, t{9}, Ki_track{9});
xlabel('Time - [s]'); ylabel('Ki'); grid; xlim([0 4]); ylim([-2.0, 0.25]);
nexttile;
plot(t{3}, Kd_track{3}, t{6}, Kd_track{6}, t{9}, Kd_track{9});
xlabel('Time - [s]'); ylabel('Kd'); grid; xlim([0 4]); ylim([-1.5, 0.25]);
l5 = legend('N64\_B256','N128\_B256','N256\_B256','Orientation','horizontal');
l5.Layout.Tile = 'north';
sgtitle('Adaptive PID gains trajectories');

figure(6);
tiledlayout(3,1);
nexttile;
plot(t{2}, Kp_track{2}, t{4}, Kp_track{4}, t{8}, Kp_track{8});
xlabel('Time - [s]'); ylabel('Kp'); grid; xlim([0 4]); ylim([-2.25, -0.75]);
nexttile;
plot(t{2}, Ki_track{2}, t{4}, Ki_track{4}, t{8}, Ki_track{8});
xlabel('Time - [s]'); ylabel('Ki'); grid; xlim([0 4]); ylim([-2.25, -0.25]);
nexttile;
plot(t{2}, Kd_track{2}, t{4}, Kd_track{4}, t{8}, Kd_track{8});
xlabel('Time - [s]'); ylabel('Kd'); grid; xlim([0 4]); ylim([-1.25, 0.25]);
l6 = legend('N64\_B128','N128\_B64','N256\_B128','Orientation','horizontal');
l6.Layout.Tile = 'north';
sgtitle('Adaptive PID gains trajectories');

% Input Output Plot
figure(7);
tiledlayout(3,3);
nexttile;
plot(t{1}, Elevator{1}, t{5}, Elevator{5}, t{7}, Elevator{7});
xlabel('Time - [s]'); ylabel('Elevator - [deg]'); grid; xlim([0 2]); ylim([-35 35]);
nexttile;
plot(t{1}, Throttle{1}, t{5}, Throttle{5}, t{7}, Throttle{7});
xlabel('Time - [s]'); ylabel('Throttle'); grid; xlim([0 4]); ylim([-0.5 1]);
nexttile;
plot(t{1}, Theta{1}, t{5}, Theta{5}, t{7}, Theta{7});
xlabel('Time - [s]'); ylabel('Pitch angle - [deg]'); grid; xlim([0 4]);
nexttile;
plot(t{1}, v_TAS{1}, t{5}, v_TAS{5}, t{7}, v_TAS{7});
xlabel('Time - [s]'); ylabel('True Airspeed - [knot]'); grid; xlim([0 4]); ylim([109 122]);
nexttile;
plot(t{1}, ZI{1}, t{5}, ZI{5}, t{7}, ZI{7});
xlabel('Time - [s]'); ylabel('Altitude - [ft]'); grid; xlim([0 4]); ylim([4990 5150]);
nexttile;
plot(t{1}, Density{1}, t{5}, Density{5}, t{7}, Density{7});
xlabel('Time - [s]'); ylabel('Density - [kgpm3]'); grid; xlim([0 4]);
nexttile;
plot(t{1}, Gamma{1}, t{5}, Gamma{5}, t{7}, Gamma{7});
xlabel('Time - [s]'); ylabel('Path angle - [deg]'); grid; xlim([0 4]); ylim([-1 14]);
nexttile;
plot(t{1}, Alpha{1}, t{5}, Alpha{5}, t{7}, Alpha{7});
xlabel('Time - [s]'); ylabel('AoA - [deg]'); grid; xlim([0 4]); ylim([-1 10]);
nexttile;
plot(t{1}, q{1}, t{5}, q{5}, t{7}, q{7});
xlabel('Time - [s]'); ylabel('Pitch rate - [deg/s]'); grid; xlim([0 2]); ylim([-15 110]);
l1 = legend('N64\_B64','N128\_B128','N256\_B64','Orientation','horizontal','fontsize',12);
l1.Layout.Tile = 'north';
sgtitle('Pitch control input-output via RL-based adaptive PIDs','fontsize',18);