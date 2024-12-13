%%% ------ Equations of Motion ------ %%%
syms x y z x_dot y_dot z_dot
syms phi theta psi phi_dot theta_dot psi_dot
syms u v w u_dot v_dot w_dot
syms p q r p_dot q_dot r_dot
syms F_aero_x F_aero_y_ F_aero_z F_thrust_x F_thrust_y F_thrust_z
syms M_aero_x M_aero_y M_aero_z M_thrust_x M_thrust_y M_thrust_z
syms F_ext_x F_ext_y F_ext_z M_ext_x M_ext_y M_ext_z
syms m g
syms I_xx I_yy I_zz I_xz I_zx

L_phi   = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
L_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
L_psi   = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
L_EtoB_  = simplify(L_phi*L_theta*L_psi);
L_BtoE_  = simplify(inv(L_psi)*inv(L_theta)*inv(L_phi));

L_phi_eq = latex(L_phi);
L_theta_eq = latex(L_theta);
L_psi_eq = latex(L_psi);

xyz_dot_ = L_BtoE_ * [u;v;w];
x_dot_ = xyz_dot_(1);
y_dot_ = xyz_dot_(2);
z_dot_ = xyz_dot_(3);

W_e_ = [0; 0; m*g];
W_b_ = L_EtoB_ * W_e_;

F_ext = [F_aero_x; F_aero_y_; F_aero_z] + [F_thrust_x; F_thrust_y; F_thrust_z] + W_b_;
M_ext = [M_aero_x; M_aero_y; M_aero_z] + [M_thrust_x; M_thrust_y; M_thrust_z];

omega_B_ = [phi_dot - psi_dot*sin(theta);
            theta_dot*cos(phi)+psi_dot*cos(theta)*sin(phi);
            psi_dot*cos(phi)*cos(theta) - theta_dot*sin(phi)];

body_to_euler_rates = [phi_dot; theta_dot; psi_dot] == [p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    q*cos(phi)-r*sin(phi); (q*sin(phi)+r*cos(phi))/cos(theta)];
body_to_euler_rates_eq = latex(body_to_euler_rates);

F_ext_ = m * ([u_dot; v_dot; w_dot] + cross([p;q;r],[u;v;w]));
I_B_ = [I_xx 0 -I_xz; 0 I_yy 0; -I_zx 0 I_zz];
M_ext_ = I_B_ * [p_dot; q_dot; r_dot] + cross([p; q; r],I_B_*[p; q; r]);
p_dot_eq = M_ext_x == M_ext_(1);
q_dot_eq = M_ext_y == M_ext_(2);
r_dot_eq = M_ext_z == M_ext_(3);
p_dot_ = solve(p_dot_eq,p_dot);
q_dot_ = solve(q_dot_eq,q_dot);
r_dot_ = solve(subs(r_dot_eq,p_dot,p_dot_),r_dot);
p_dot_ = subs(p_dot_,r_dot,r_dot_);

% Summary
syms f S_phi S_theta S_psi C_phi C_theta C_psi T_theta
states = [x;y;z;phi;theta;psi;u;v;w;p;q;r];
states_dot = [x_dot;y_dot;z_dot;
              phi_dot;theta_dot;psi_dot;
              u_dot;v_dot;w_dot;
              p_dot;q_dot;r_dot];
states_dot_ = [x_dot_;y_dot_;z_dot_;
               p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
               q*cos(phi)-r*sin(phi);
               (q*sin(phi)+r*cos(phi))/cos(theta);
               F_ext_x/m - q*w + r*v;
               F_ext_y/m + p*w - r*u;
               F_ext_z/m - p*v + q*u;
               p_dot_; q_dot_; r_dot_];
states_dot_ = simplify(states_dot_);
states_dot_ = subs(states_dot_,{sin(phi) sin(theta) sin(psi)},{S_phi S_theta S_psi});
states_dot_ = subs(states_dot_,{cos(phi) cos(theta) cos(psi)},{C_phi C_theta C_psi});
states_dot_ = subs(states_dot_,{tan(theta)},{T_theta});

latex(states_dot_)

%%% ------ Atmosphere ------ %%%
syms g R k P_0 T_0 P T h P_11 T_11 h_11 gamma M a_0 delta_P v_TAS

T1 = T_0 + k*h;                         % for h <= 11 km
T2 = T_0 + k*11000;                     % for 11 km < h < 20 km
P1 = P_0 * (T/T_0)^(-g/(R*k));          % for h <= 11 km
P2 = P_11*exp((-g/(R*T_11)*(h-h_11)));  % for 11 km < h < 20 km
v_sound = sqrt(gamma*R*T);
M_ = v_TAS/v_sound;
a0_ = sqrt(gamma*R*T_0);
delta_P_ = P*(((gamma-1)/2*M^2+1)^(gamma/(gamma-1))-1);
v_CAS = sqrt(2*a_0^2/(gamma-1)*((delta_P/P_0+1)^((gamma-1)/gamma)-1));

P1_eq = latex(P1);
P2_eq = latex(P2);
T1_eq = latex(T1);
T2_eq = latex(T2);
rho_air_eq = latex(P/(R*T));
v_sound_eq = latex(v_sound);
delta_P_eq = latex(delta_P_);
v_CAS_eq = latex(v_CAS);

%%% ------ True Airspeed and Without Wind Effect ------ %%%
syms v_w_x v_w_y v_w_z L_EtoB
syms U V W
v_TAS_ = sqrt(U^2 + V^2 + W^2);
alpha_ = atan2(W,U);
beta_ = atan2(V,v_TAS);
v_TAS_eq = latex(v_TAS_);
alpha_eq = latex(alpha_);
beta_eq = latex(beta_);

%%% ------ Aerodynamics ------ %%%
syms c b S x_CG y_CG z_CG CG_mac;
syms Q rho_air v_TAS p q r;
syms alpha alpha_dot beta delta_a delta_e delta_r;
syms C_L C_D C_Y C_l C_m C_n;
syms C_X_b C_Y_b C_Z_b C_l_b C_m_b C_n_b
syms F_X F_Y F_Z M_l M_m M_n
syms C_L_0 C_L_alpha C_L_delta_e C_L_alpha_dot C_L_q;
syms C_D_0 C_D_alpha C_D_delta_e;
syms C_Y_beta C_Y_delta_a C_Y_delta_r C_Y_p C_Y_r;
syms C_l_beta C_l_delta_a C_l_delta_r C_l_p C_l_r;
syms C_m_0 C_m_alpha C_m_delta_e C_m_alpha_dot C_m_q;
syms C_n_beta C_n_delta_a C_n_delta_r C_n_p C_n_r;

% CL, CD, CY, Cl, Cm, Cn
CL_tot = C_L_0 + C_L_alpha*alpha + C_L_delta_e*delta_e + C_L_alpha_dot*alpha_dot*c/(2*v_TAS) + C_L_q*q*c/(2*v_TAS);
CD_tot = C_D_0 + C_D_alpha*alpha + C_D_delta_e*delta_e;
CY_tot = C_Y_beta*beta + C_Y_delta_a*delta_a + C_Y_delta_r*delta_r + C_Y_p*p*b/(2*v_TAS) + C_Y_r*r*b/(2*v_TAS);
Cl_tot = C_l_beta*beta + C_l_delta_a*delta_a + C_l_delta_r*delta_r + C_l_p*p*b/(2*v_TAS) + C_l_r*r*b/(2*v_TAS);
Cm_tot = C_m_0 + C_m_alpha*alpha + C_m_delta_e*delta_e + C_m_alpha_dot*alpha_dot*c/(2*v_TAS) + C_m_q*q*c/(2*v_TAS);
Cn_tot = C_n_beta*beta + C_n_delta_a*delta_a + C_n_delta_r*delta_r + C_n_p*p*b/(2*v_TAS) + C_n_r*r*b/(2*v_TAS);

CL_eq = latex(CL_tot);
CD_eq = latex(CD_tot);
CY_eq = latex(CY_tot);
Cl_eq = latex(Cl_tot);
Cm_eq = latex(Cm_tot);
Cn_eq = latex(Cn_tot);

% CXb, CYb, CZb, Clb, Cmb, Cnb
CXb_ =   C_L*sin(alpha) - C_D*cos(alpha);
CYb_ =   C_Y;
CZb_ = - C_L*cos(alpha) - C_D*sin(alpha);
Clb_ =   C_l*cos(alpha) - C_n*sin(alpha);
Cmb_ =   C_m;
Cnb_ =   C_n*cos(alpha) + C_l*sin(alpha);

CXb_eq = latex(CXb_);
CYb_eq = latex(CYb_);
CZb_eq = latex(CZb_);
Clb_eq = latex(Clb_);
Cmb_eq = latex(Cmb_);
Cnb_eq = latex(Cnb_);

% Dynamic pressure
Q_ = 0.5*rho_air*v_TAS^2;
Q_eq = latex(Q_);

% FX, FY, FZ, MX, MY, MZ
F_aero_x_ = Q*S*C_X_b;
F_aero_y_ = Q*S*C_Y_b;
F_aero_z_ = Q*S*C_Z_b;
M_aero_x_ = Q*S*b*C_l_b - F_Y*z_CG - F_Z*y_CG;
M_aero_y_ = Q*S*c*C_m_b + F_X*z_CG - F_Z*(CG_mac-0.25)*c;
M_aero_z_ = Q*S*b*C_n_b + F_X*y_CG + F_Y*(CG_mac-0.25)*c;

F_aero_x_eq = latex(F_aero_x_);
F_aero_y_eq = latex(F_aero_y_);
F_aero_z_eq = latex(F_aero_z_);
M_aero_x_eq = latex(M_aero_x_);
M_aero_y_eq = latex(M_aero_y_);
M_aero_z_eq = latex(M_aero_z_);

%%% ------ Propulsion ------ %%%
syms delta_T T_max v_ref rho_ref n_v n_rho v_TAS rho_air alpha_F T X_F Z_F

T_ = delta_T * T_max * (v_TAS/v_ref)^n_v * (rho_air/rho_ref)^n_rho;
F_thrust_x_ = T_*cos(alpha_F);
F_thrust_y_ = 0.0;
F_thrust_z_ = T_*sin(alpha_F);
M_thrust_x_ = 0.0;
M_thrust_y_ = T*cos(alpha_F)*Z_F - T*sin(alpha_F)*X_F;
M_thrust_z_ = 0.0;

%%% ------ Longitudinal Dynamics ------ %%%
syms m g I_yy
syms u w q theta u_dot w_dot q_dot theta_dot

F_aero_x_ = subs(F_aero_x_,{C_X_b,Q,},{CXb_,Q_});
F_aero_x_ = subs(F_aero_x_,{C_D,C_L},{CD_tot,CL_tot});
F_aero_z_ = subs(F_aero_z_,{C_Z_b,Q,},{CZb_,Q_});
F_aero_z_ = subs(F_aero_z_,{C_D,C_L},{CD_tot,CL_tot});
M_aero_y_ = subs(M_aero_y_,{F_X,F_Z,C_m_b,Q},{F_aero_x_,F_aero_z_,Cmb_,Q_});
M_aero_y_ = subs(M_aero_y_,{C_m},{Cm_tot});
M_thrust_y_ = subs(M_thrust_y_,{T},{T_});

eq1 = F_aero_x_ + F_thrust_x_ == m*(u_dot+q*w) + m*g*sin(theta);
eq2 = F_aero_z_ + F_thrust_z_ == m*(w_dot-q*u) - m*g*cos(theta);
eq3 = M_aero_y_ + M_thrust_y_ == I_yy * q_dot;
eq4 = theta_dot == q;

% eq1 = collect(eq1,[u w alpha theta u_dot w_dot alpha_dot theta_dot]);
% eq2 = collect(eq2,[u w alpha theta u_dot w_dot alpha_dot theta_dot]);
% eq3 = collect(eq3,[alpha alpha_dot q q_dot]);

eq1 = subs(eq1,{S,rho_air,v_TAS},{16.1651,1.0557,49.8560});
eq1 = subs(eq1,{m,g,c},{1043.3,9.80665,1.4935});
eq1 = subs(eq1,{C_L_0,C_L_alpha,C_L_delta_e,C_L_alpha_dot,C_L_q},{0.31,5.143,0.43,0.0,3.9});
eq1 = subs(eq1,{C_D_0,C_D_alpha,C_D_delta_e},{0.031,0.13,0.06});
eq1 = subs(eq1,{T_max,alpha_F,rho_ref,v_ref,n_rho,n_v},{2070.0,1.0*pi/180,1.225,51.4,0.75,-1.0});
eq1 = solve(eq1,u_dot);
u_dot_ = vpa(simplify(eq1),4); % u_dot

eq2 = subs(eq2,{S,rho_air,v_TAS},{16.1651,1.0557,49.8560});
eq2 = subs(eq2,{m,g,c},{1043.3,9.80665,1.4935});
eq2 = subs(eq2,{C_L_0,C_L_alpha,C_L_delta_e,C_L_alpha_dot,C_L_q},{0.31,5.143,0.43,0.0,3.9});
eq2 = subs(eq2,{C_D_0,C_D_alpha,C_D_delta_e},{0.031,0.13,0.06});
eq2 = subs(eq2,{T_max,alpha_F,rho_ref,v_ref,n_rho,n_v},{2070.0,1.0*pi/180,1.225,51.4,0.75,-1.0});
eq2 = solve(eq2,w_dot);
w_dot_ = vpa(simplify(eq2),4); % w_dot

eq3 = subs(eq3,{S,rho_air,v_TAS},{16.1651,1.0557,49.8560});
eq3 = subs(eq3,{C_L_0,C_L_alpha,C_L_delta_e,C_L_alpha_dot,C_L_q},{0.31,5.143,0.43,0.0,3.9});
eq3 = subs(eq3,{C_D_0,C_D_alpha,C_D_delta_e},{0.031,0.13,0.06});
eq3 = subs(eq3,{C_m_0,C_m_alpha,C_m_delta_e,C_m_alpha_dot,C_m_q},{-0.015,-0.89,-1.28,-7.27,-12.4});
eq3 = subs(eq3,{T_max,alpha_F,rho_ref,v_ref,n_rho,n_v},{2070.0,1.0*pi/180,1.225,51.4,0.75,-1.0});
eq3 = subs(eq3,{I_yy c y_CG z_CG X_F Z_F CG_mac},{1824.9 1.4935 0.0 0.2 1.0 0.0 0.3});
eq3 = solve(eq3,q_dot);
q_dot_ = vpa(simplify(eq3),4); % q_dot

% Small angle approximations
u_dot_ = subs(u_dot_,{sin(theta), sin(alpha), cos(theta), cos(alpha)},{theta, 0, 1, 1});
u_dot_ = vpa(simplify(u_dot_),4); % u_dot
w_dot_ = subs(w_dot_,{sin(theta), sin(alpha), cos(theta), cos(alpha)},{theta, 0, 1, 1});
w_dot_ = vpa(simplify(w_dot_),4); % w_dot
q_dot_ = subs(q_dot_,{sin(theta), sin(alpha), cos(theta), cos(alpha)},{theta, 0, 1, 1});
q_dot_ = vpa(simplify(q_dot_),4); % q_dot

syms alpha_(t) w_(t) u_(t)
% Alpha = atan2(w/u)
eq1 = tan(alpha_)==w_/u_;
eq1 = diff(eq1,t);
eq1 = subs(eq1,diff(alpha_,t),alpha_dot);
eq1 = subs(eq1,diff(u_,t),u_dot);
eq1 = subs(eq1,diff(w_,t),w_dot);
eq1 = subs(eq1,u_,u);
eq1 = subs(eq1,w_,w);
eq1 = subs(eq1,alpha_,alpha);
eq1 = alpha_dot == w_dot/u-u_dot*w/u^2;
assume(u,'positive');
eq1 = subs(eq1,{u_dot w_dot},{u_dot_, w_dot_});
eq1 = solve(eq1,alpha_dot);
alpha_dot_ = vpa(simplify(eq1),4); % alpha_dot
%latex(alpha_dot_)









