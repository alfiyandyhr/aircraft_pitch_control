function [op,opreport] = trim_cond(ZI_m,Gamma_rad)
    %% Specify the model name
    model = 'AircraftDynamicSimNonLinear';

    %% Create the operating point specification object.
    opspec = operspec(model);

    %% Set the constraints on the states in the model.
    % State 1 : p_radps
    % State 2 : q_radps
    % State 3 : r_radps
    % State 4 : u_mps
    opspec.States(4).x = 50;
    opspec.States(4).SteadyState = true;
    % State 5 : v_mps
    % State 6 : w_mps
    % State 7 : PHI_rad
    % State 8 : PSI_rad
    % State 9 : THETA_rad
    % State 10: XI_m
    opspec.States(10).SteadyState = false;
    % State 11: YI_m
    opspec.States(11).SteadyState = false;
    % State 12: ZI_m
    opspec.States(12).SteadyState = false;
    
    %% Set the constraints on the inputs in the model.
    % Input 1 : Elevator_rad
    opspec.Inputs(1).Min = -30*pi/180;
    opspec.Inputs(1).Max = 30*pi/180;
    % Input 2 : Aileron_rad
    opspec.Inputs(2).Min = -30*pi/180;
    opspec.Inputs(2).Max = 30*pi/180;
    % Input 3 : Rudder_rad
    opspec.Inputs(3).Min = -30*pi/180;
    opspec.Inputs(3).Max = 30*pi/180;
    % Input 4 : Throttle
    opspec.Inputs(4).Min = 0;
    opspec.Inputs(4).Max = 1;

    %% Set the constraints on the outputs in the model.
    % Output 1 : InertialPositions_m
    opspec.Outputs(1).y = [0;0;ZI_m];
    opspec.Outputs(1).Known = [false;false;true];
    % Output 2 : EulerAngles_rad
    opspec.Outputs(2).y = [0;Gamma_rad;0];
    opspec.Outputs(2).Known = [true;true;true];
    % Output 3 : BodyVelocities_mps
    opspec.Outputs(3).y = [0;0;0];
    opspec.Outputs(3).Known = [false;true;true];
    % Output 4 : BodyRates_radps
    opspec.Outputs(4).y = [0;0;0];
    opspec.Outputs(4).Known = [true;true;true];
    % Output 5 : Alpha_rad
    % Output 6 : Beta_rad
    opspec.Outputs(6).y = 0.0;
    opspec.Outputs(6).Known = true;
    % Output 7 : Gamma_rad
    opspec.Outputs(7).y = Gamma_rad;
    opspec.Outputs(7).Known = true;
    % Output 8 : Zeta_rad
    % Output 9 : v_TAS_mps
    % Output 10: Density_kgpm3
    
    %% Create the options
    opt = findopOptions('DisplayReport','iter');

    %% Perform the operating point search.
    [op,opreport] = findop(model,opspec,opt);
end