% Aircraft longitudinal dynamics
% Assumption: SISO (elevator to pitch)
A = [0, 0, 0, 1, 0, 0;
     0, 0, -62.39, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, -0.0001, -9.807, -0.0477, 0.2388, 0;
     0, -0.0022, 0, -0.3152, -2.64, 60.9;
     0, 0, 0, 0.0005, -0.2494, -3.971];
B = [0; 0; 0; 1.91; -13.69; -33.99];
C = [0, 0, 1, 0, 0, 0];  % for theta only
D = 0;

% Open-loop tf
sys_ol = ss(A,B,C,D);
tf_ol = tf(sys_ol);

%controlSystemDesigner(sys_ol)

% PID controller manual tuning with time filter Tf=100
% Design:
% rise time <2s
% settling time <10s
% overshoot <10%
% steady-state error <2%
Kp = [-1,-1,-1,-1,-1];
Ki = [-1,-0.8,-0.6,-0.3,-0.3];
Kd = [0,0,0,0,-0.1];
Tf = 0.01;
info_list = zeros(5,6); % [Kp Ki Kd risetime settlingtime overshoot]
for i = 1:size(Kp,2)
    info_list(i,1)=Kp(i);
    info_list(i,2)=Ki(i);
    info_list(i,3)=Kd(i);
    C_pid = pid(Kp(i),Ki(i),Kd(i),Tf);
    sys_pid = feedback(C_pid*sys_ol,1);
    info = stepinfo(0.2*sys_pid);
    info_list(i,4)=info.RiseTime;
    info_list(i,5)=info.SettlingTime;
    info_list(i,6)=info.Overshoot;
end

info_list
display('Note: the control input is not bounded here. Use "PIDControlSim.m" instead!!!');