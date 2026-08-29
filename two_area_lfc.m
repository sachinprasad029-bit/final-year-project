%% Two-Area Load Frequency Control (LFC) Simulation
% Replicates a two-area interconnected power system with PI secondary
% control (AGC), governor droop, and tie-line coupling -- matching the
% structure of the Simulink model (Governor -> Turbine -> Power Network,
% with droop feedback, bias factors B1/B2, and a tie-line integrator).
%
% Disturbance can be injected as:
%   - a POWER disturbance (physical load step, Delta P_L) -- the standard
%     LFC case, entering the swing-equation summing junction directly.
%   - a FREQUENCY disturbance (a synthetic offset added to the MEASURED
%     frequency signal seen by the governor droop and AGC/ACE loops --
%     useful for testing controller robustness to a bad frequency
%     measurement/relay. It is NOT injected into the physical swing
%     equation, since a "true" delta-f disturbance has no direct physical
%     actuator -- this is a modeling choice, documented here so it's not
%     mistaken for a load disturbance.)
%
% and can be applied to Area 1 only, Area 2 only, or both.
%
% NOTE: Governor/turbine/power-network time constants, droop (1/R), and
% bias (B) values below are transcribed from the Simulink diagram. The
% PI gains (Kp/Ki) are PLACEHOLDERS -- tune them to match whatever gains
% are actually inside your Simulink "PID" blocks.

clear; clc; close all;

%% ---------------- USER SETTINGS (asked interactively) ----------------
Tsim = 50;   % simulation stop time (s)

% --- 1) Disturbance TYPE ---
typeChoice = menu('Select disturbance type:', ...
                   'Power disturbance (Delta P)', ...
                   'Frequency disturbance (Delta f)');
if typeChoice == 1
    dist.type = 'power';
else
    dist.type = 'freq';
end

% --- 2) Which AREA gets the disturbance ---
areaChoice = menu('Select which area is disturbed:', ...
                   'Area 1 only', 'Area 2 only', 'Both areas');
switch areaChoice
    case 1, dist.area = 'area1';
    case 2, dist.area = 'area2';
    case 3, dist.area = 'both';
end

% --- 3) Magnitudes (only asked for the area(s) actually selected) ---
if strcmp(dist.type,'power')
    unitStr = 'p.u. (load step, Delta P_L)';
else
    unitStr = 'p.u. (frequency-measurement offset, Delta f)';
end

dist.mag1 = 0;  dist.mag2 = 0;
if any(strcmp(dist.area, {'area1','both'}))
    dist.mag1 = str2double(inputdlg( ...
        sprintf('Enter disturbance magnitude for AREA 1 [%s]:', unitStr), ...
        'Area 1 magnitude', 1, {'0.1875'}));
end
if any(strcmp(dist.area, {'area2','both'}))
    dist.mag2 = str2double(inputdlg( ...
        sprintf('Enter disturbance magnitude for AREA 2 [%s]:', unitStr), ...
        'Area 2 magnitude', 1, {'-0.20'}));
end

% --- 4) Step time ---
dist.tstep = str2double(inputdlg( ...
    'Enter the time (s) the disturbance step occurs:', ...
    'Step time', 1, {'5'}));

fprintf('\n--- Simulation settings ---\n');
fprintf('Disturbance type : %s\n', dist.type);
fprintf('Disturbed area   : %s\n', dist.area);
fprintf('Magnitude Area 1 : %g\n', dist.mag1);
fprintf('Magnitude Area 2 : %g\n', dist.mag2);
fprintf('Step time        : %g s\n\n', dist.tstep);

%% ---------------- SYSTEM PARAMETERS ----------------
% Area 1
Tg1 = 0.2;   Tt1 = 0.5;   H1 = 5;    D1 = 1;      R1 = 1/20;     B1 = 20.6;
% Area 2
Tg2 = 0.3;   Tt2 = 0.6;   H2 = 4;    D2 = 0.9;    R2 = 1/16;     B2 = 16.92;
% Tie-line synchronizing gain (matches the gain-"2" block feeding the 1/s
% integrator in the Simulink diagram)
Ktie = 2;

% Secondary (AGC/PID) control gains -- read directly from the Simulink
% "PID Controller" block masks (formula shown there: P + I/s + D*s)
Kp1 = 0.7;  Ki1 = 1;  Kd1 = 0.7;   % Area 1 PID Controller
Kp2 = 0.7;  Ki2 = 1;  Kd2 = 1;     % Area 2 PID Controller1

p = struct('Tg1',Tg1,'Tt1',Tt1,'H1',H1,'D1',D1,'R1',R1,'B1',B1, ...
           'Tg2',Tg2,'Tt2',Tt2,'H2',H2,'D2',D2,'R2',R2,'B2',B2, ...
           'Ktie',Ktie,'Kp1',Kp1,'Ki1',Ki1,'Kd1',Kd1, ...
           'Kp2',Kp2,'Ki2',Ki2,'Kd2',Kd2,'dist',dist);

%% ---------------- SIMULATE ----------------
% State vector x = [Pv1 Pg1 df1 Pv2 Pg2 df2 Ptie z1 z2]
%   Pv  - governor output       Pg - turbine (mechanical power) output
%   df  - frequency deviation   Ptie - tie-line power deviation
%   z   - AGC integral state (for the Ki term of each area's PI control)
x0 = zeros(9,1);
% Tight tolerances + forcing the solver through the exact step time help
% it resolve the sharp D-term kick at t = dist.tstep cleanly.
opts = odeset('RelTol',1e-7,'AbsTol',1e-10,'MaxStep',0.05);
tspan = unique([0:0.02:Tsim, dist.tstep, Tsim]);
[t,x] = ode45(@(t,x) lfc_odefun(t,x,p), tspan, x0, opts);

Pv1=x(:,1); Pg1=x(:,2); df1=x(:,3); %#ok<NASGU>
Pv2=x(:,4); Pg2=x(:,5); df2=x(:,6); %#ok<NASGU>
Ptie=x(:,7);

%% ---------------- PLOT 1: FREQUENCY DEVIATIONS ----------------
figure('Name','Frequency Deviation');
plot(t, df1, 'y', 'LineWidth', 1.4); hold on;
plot(t, df2, 'b', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)'); ylabel('\Delta f (p.u.)');
title(sprintf('Frequency Deviation -- disturbance: %s, area: %s', dist.type, dist.area));
legend('\Delta f_1 (Area 1)','\Delta f_2 (Area 2)','Location','best');

%% ---------------- PLOT 2: GENERATION & TIE-LINE POWER ----------------
figure('Name','Generation and Tie-line Power');
plot(t, Pg1, 'y', 'LineWidth', 1.4); hold on;
plot(t, Pg2, 'b', 'LineWidth', 1.4);
plot(t, Ptie,'Color',[1 0.5 0],'LineStyle','--','LineWidth', 1.4);
grid on;
xlabel('Time (s)'); ylabel('Power (p.u.)');
title(sprintf('P_{g1}, P_{g2}, and Tie-line Power -- disturbance: %s, area: %s', dist.type, dist.area));
legend('P_{g1} (Area 1)','P_{g2} (Area 2)','P_{tie}','Location','best');

%% ==================== ODE FUNCTION ====================
function dx = lfc_odefun(t,x,p)
    Pv1=x(1); Pg1=x(2); df1=x(3);
    Pv2=x(4); Pg2=x(5); df2=x(6);
    Ptie=x(7); z1=x(8); z2=x(9);

    %% -- Disturbance signals (step at t = dist.tstep) --
    PL1 = 0; PL2 = 0;   % physical load disturbance
    fd1 = 0; fd2 = 0;   % synthetic frequency-measurement disturbance

    if t >= p.dist.tstep
        switch p.dist.area
            case 'area1', a1 = true;  a2 = false;
            case 'area2', a1 = false; a2 = true;
            case 'both',  a1 = true;  a2 = true;
            otherwise, error('dist.area must be ''area1'', ''area2'', or ''both''');
        end
        switch p.dist.type
            case 'power'
                if a1, PL1 = p.dist.mag1; end
                if a2, PL2 = p.dist.mag2; end
            case 'freq'
                if a1, fd1 = p.dist.mag1; end
                if a2, fd2 = p.dist.mag2; end
            otherwise, error('dist.type must be ''power'' or ''freq''');
        end
    end

    %% -- Measured frequency (physical + any injected measurement disturbance) --
    df1_meas = df1 + fd1;
    df2_meas = df2 + fd2;

    %% -- ACE (algebraic in the states -- no need to know Pc to compute it) --
    ACE1 = Ptie + p.B1*df1_meas;
    ACE2 = -Ptie + p.B2*df2_meas;

    %% -- Power network / swing equation and tie-line derivatives --
    % These depend only on states + PL (not on Pc), so they can be computed
    % BEFORE the PID control law -- which lets us get an EXACT analytic
    % derivative of ACE for the D-term below, instead of approximating
    % the ideal (unfiltered) P+I/s+D*s PID with a filtered derivative.
    ddf1  = (Pg1 - PL1 - Ptie - p.D1*df1)/(2*p.H1);
    ddf2  = (Pg2 - PL2 + Ptie - p.D2*df2)/(2*p.H2);
    dPtie = p.Ktie*(df1 - df2);

    % Analytic d(ACE)/dt, using the PHYSICAL frequency derivatives above.
    % NOTE: for a 'freq'-type disturbance, fd1/fd2 are pure step offsets
    % added only to the P and I paths of ACE (via df_meas) -- they are
    % deliberately excluded here, because differentiating a true step
    % gives a Dirac impulse, which is not something ode45 can integrate.
    % The D-term therefore reacts to genuine power/frequency dynamics
    % only, which is the physically meaningful case anyway.
    dACE1 = dPtie + p.B1*ddf1;
    dACE2 = -dPtie + p.B2*ddf2;

    %% -- Full PID secondary control (P + I/s + D*s, matches Simulink mask) --
    Pc1 = -(p.Kp1*ACE1 + p.Ki1*z1 + p.Kd1*dACE1);
    Pc2 = -(p.Kp2*ACE2 + p.Ki2*z2 + p.Kd2*dACE2);

    %% -- Governor (with droop feedback from measured frequency) --
    dPv1 = (Pc1 - (1/p.R1)*df1_meas - Pv1)/p.Tg1;
    dPv2 = (Pc2 - (1/p.R2)*df2_meas - Pv2)/p.Tg2;

    %% -- Turbine --
    dPg1 = (Pv1 - Pg1)/p.Tt1;
    dPg2 = (Pv2 - Pg2)/p.Tt2;

    %% -- AGC integral states --
    dz1 = ACE1;
    dz2 = ACE2;

    dx = [dPv1; dPg1; ddf1; dPv2; dPg2; ddf2; dPtie; dz1; dz2];
end
