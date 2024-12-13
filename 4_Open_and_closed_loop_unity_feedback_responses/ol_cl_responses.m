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

% Open-loop step response
fig1 = figure(1);
fig1.Renderer = 'painters';
t = 0:0.01:200;
step(0.2*sys_ol,t);
ylabel('Pitch angle (rad)');
title('Open-loop Step Response');
grid on;

% Closed-loop step response
fig2 = figure(2);
fig2.Renderer = 'painters';
t = 0:0.01:2;
step(0.2*feedback(sys_ol,1),t);
ylabel('Pitch angle (rad)');
title('Closed-loop Step Response');
grid on;

% Root locus analysis
% C_pid = pid(-1,-0.3,-0.1,0.01);
% sys_pid = feedback(C_pid*sys_ol,1);
% rlocus(sys_pid)