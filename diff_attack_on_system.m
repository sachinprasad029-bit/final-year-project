%% Two-Area LFC Simulation with Cyberattack Injection
% Extends the base two-area LFC model with selectable cyberattacks on the
% TELEMETERED (measured) frequency and/or tie-line power signals that feed
% the ACE calculation, governor droop, and PID/AGC control.
%
% KEY IDEA: the physical states (df1, df2, Ptie -- what's actually
% happening in the plant) are always propagated correctly by the swing
% equation. The attack only corrupts the MEASURED copy of these signals
% (df1_meas, df2_meas, Ptie_meas) that the controller sees -- exactly like
% a compromised sensor/communication channel in a real SCADA system.
%
% Attack types implemented:
%   fdia        - False Data Injection: adds a false bias to the signal
%   delay       - Time-delays the signal by attack.delay seconds
%   replay      - Loops a recorded pre-attack segment instead of live data
%   noise       - Adds random Gaussian noise to the signal
%   dos         - Freezes the signal at its last known value (blackout)
%   packetdrop  - Randomly drops individual samples (intermittent freeze)
%
% NOTE: a fixed-step RK4 integrator is used INSTEAD OF ode45, because
% delay/replay/DoS/packet-drop all need a predictable, indexable history
% buffer of past samples -- something an adaptive-step solver can't give
% cleanly. The measured signal is held constant (zero-order-hold) across
% the 4 RK4 sub-stages of each step, which is a standard, minor
% simplification for discrete-attack simulation.

clear; clc; close all;

%% ==================== USER SETTINGS: SIMULATION ====================
Tsim = 50;   % simulation stop time (s)
dt   = 0.005; % fixed integration step (s)

%% ==================== USER SETTINGS: LOAD DISTURBANCE ====================
typeChoice = menu('Select disturbance type:', ...
                   'Power disturbance (Delta P)', ...
                   'Frequency disturbance (Delta f)');
if typeChoice == 1
    dist.type = 'power';
else
    dist.type = 'freq';
end

areaChoice = menu('Select which area gets the LOAD disturbance:', ...
                   'Area 1 only', 'Area 2 only', 'Both areas', 'None');
switch areaChoice
    case 1, dist.area = 'area1';
    case 2, dist.area = 'area2';
    case 3, dist.area = 'both';
    case 4, dist.area = 'none';
end

if strcmp(dist.type,'power'); unitStr = 'p.u. (load step, Delta P_L)';
else;                          unitStr = 'p.u. (frequency-measurement offset, Delta f)';
end

dist.mag1 = 0; dist.mag2 = 0;
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
dist.tstep = str2double(inputdlg( ...
    'Enter the time (s) the LOAD disturbance step occurs:', ...
    'Step time', 1, {'5'}));

%% ==================== USER SETTINGS: CYBERATTACK ====================
atkYN = menu('Simulate a cyberattack on the measurement/communication channel?', ...
             'No attack', 'Yes, configure attack');
attack.enable = (atkYN == 2);
attack.target = 'none';
attack.area   = 'none';

if attack.enable
    typeIdx = menu('Select attack type:', ...
        'FDIA (false data injection)','Delay','Replay','Noise','DoS (blackout)','Packet drop');
    typeList = {'fdia','delay','replay','noise','dos','packetdrop'};
    attack.type = typeList{typeIdx};

    targetIdx = menu('Select ATTACKED signal:', ...
        'Frequency (Delta f)', 'Tie-line power (Delta P / Ptie)', 'Both');
    switch targetIdx
        case 1, attack.target = 'freq';
        case 2, attack.target = 'tie';
        case 3, attack.target = 'both';
    end

    if ~strcmp(attack.target,'tie')
        areaIdx = menu('Which area''s FREQUENCY measurement is attacked?', ...
            'Area 1','Area 2','Both areas');
        switch areaIdx
            case 1, attack.area = 'area1';
            case 2, attack.area = 'area2';
            case 3, attack.area = 'both';
        end
    else
        attack.area = 'both';   % tie-line measurement is a single shared quantity
    end

    tw = inputdlg({'Attack start time (s):','Attack end time (s):'}, ...
                  'Attack window', 1, {'15','30'});
    attack.tstart = str2double(tw{1});
    attack.tend   = str2double(tw{2});

    switch attack.type
        case 'fdia'
            v = inputdlg('FDIA bias magnitude (p.u., added to the measurement):', ...
                'FDIA parameters', 1, {'0.05'});
            attack.bias = str2double(v{1});
        case 'delay'
            v = inputdlg('Delay duration (s):', 'Delay parameters', 1, {'0.5'});
            attack.delay = str2double(v{1});
        case 'replay'
            v = inputdlg('Length of the pre-attack segment to record & loop (s):', ...
                'Replay parameters', 1, {'2'});
            attack.replay_len = str2double(v{1});
        case 'noise'
            v = inputdlg('Noise standard deviation (p.u.):', 'Noise parameters', 1, {'0.02'});
            attack.noise_std = str2double(v{1});
        case 'dos'
            % no extra parameter -- signal is simply frozen for the whole window
        case 'packetdrop'
            v = inputdlg('Packet drop probability per sample (0-1):', ...
                'Packet drop parameters', 1, {'0.2'});
            attack.drop_prob = str2double(v{1});
    end
else
    attack.type = 'none';
end

% Fill in defaults for any unset attack fields (keeps process_channel() safe)
if ~isfield(attack,'bias'),        attack.bias = 0;        end
if ~isfield(attack,'delay'),       attack.delay = 0;       end
if ~isfield(attack,'replay_len'),  attack.replay_len = 1;  end
if ~isfield(attack,'noise_std'),   attack.noise_std = 0;   end
if ~isfield(attack,'drop_prob'),   attack.drop_prob = 0;   end

fprintf('\n--- Simulation settings ---\n');
fprintf('Load disturbance : type=%s area=%s  mag1=%g mag2=%g  tstep=%g s\n', ...
        dist.type, dist.area, dist.mag1, dist.mag2, dist.tstep);
if attack.enable
    fprintf('Attack           : type=%s target=%s area=%s window=[%g, %g] s\n', ...
        attack.type, attack.target, attack.area, attack.tstart, attack.tend);
else
    fprintf('Attack           : none\n');
end
fprintf('\n');

%% ==================== SYSTEM PARAMETERS ====================
% Area 1
Tg1 = 0.2;   Tt1 = 0.5;   H1 = 5;    D1 = 1;      R1 = 1/20;     B1 = 20.6;
% Area 2
Tg2 = 0.3;   Tt2 = 0.6;   H2 = 4;    D2 = 0.9;    R2 = 1/16;     B2 = 16.92;
% Tie-line synchronizing gain
Ktie = 2;
% Secondary (AGC/PID) control gains
Kp1 = 0.7;  Ki1 = 1;  Kd1 = 0.7;   % Area 1
Kp2 = 0.7;  Ki2 = 1;  Kd2 = 1;     % Area 2

p = struct('Tg1',Tg1,'Tt1',Tt1,'H1',H1,'D1',D1,'R1',R1,'B1',B1, ...
           'Tg2',Tg2,'Tt2',Tt2,'H2',H2,'D2',D2,'R2',R2,'B2',B2, ...
           'Ktie',Ktie,'Kp1',Kp1,'Ki1',Ki1,'Kd1',Kd1, ...
           'Kp2',Kp2,'Ki2',Ki2,'Kd2',Kd2);

%% ==================== BUILD PER-CHANNEL ATTACK CONFIGS ====================
% Three independent "telemetry channels" can be attacked: df1, df2, Ptie.
% Each gets its own enable flag depending on the target/area chosen above.
atk1 = attack;  % channel: Area 1 frequency measurement
atk1.enable = attack.enable && ...
    (strcmp(attack.target,'freq') || strcmp(attack.target,'both')) && ...
    (strcmp(attack.area,'area1')  || strcmp(attack.area,'both'));

atk2 = attack;  % channel: Area 2 frequency measurement
atk2.enable = attack.enable && ...
    (strcmp(attack.target,'freq') || strcmp(attack.target,'both')) && ...
    (strcmp(attack.area,'area2')  || strcmp(attack.area,'both'));

atkT = attack;  % channel: shared tie-line power measurement
atkT.enable = attack.enable && ...
    (strcmp(attack.target,'tie') || strcmp(attack.target,'both'));

%% ==================== SIMULATE ====================
N = round(Tsim/dt) + 1;
t = (0:N-1)*dt;

buf_len = max([round(2/dt), round(attack.delay/dt)+5, round(attack.replay_len/dt)+5]);
ch1 = init_channel(buf_len);
ch2 = init_channel(buf_len);
chT = init_channel(buf_len);

% State vector x = [Pv1 Pg1 df1 Pv2 Pg2 df2 Ptie z1 z2]
X = zeros(9, N);
DF1_MEAS  = zeros(1,N);
DF2_MEAS  = zeros(1,N);
PTIE_MEAS = zeros(1,N);

x = zeros(9,1);
for k = 1:N-1
    tk = t(k);

    % ---- Load (physical) disturbance, same as base model ----
    PL1 = 0; PL2 = 0;
    if tk >= dist.tstep && ~strcmp(dist.area,'none')
        switch dist.area
            case 'area1', a1 = true;  a2 = false;
            case 'area2', a1 = false; a2 = true;
            case 'both',  a1 = true;  a2 = true;
        end
        switch dist.type
            case 'power'
                if a1, PL1 = dist.mag1; end
                if a2, PL2 = dist.mag2; end
            case 'freq'
                % handled as a measurement-side offset via a temporary
                % pseudo-attack path is unnecessary here; for a pure
                % frequency-type load test, prefer the 'fdia' cyberattack
                % option above, which offers the same effect with full
                % windowing/area control.
        end
    end

    df1_true  = x(3);
    df2_true  = x(6);
    Ptie_true = x(7);

    % ---- Apply cyberattack to each telemetry channel ----
    [df1_meas,  ch1] = process_channel(df1_true,  tk, ch1, dt, atk1);
    [df2_meas,  ch2] = process_channel(df2_true,  tk, ch2, dt, atk2);
    [Ptie_meas, chT] = process_channel(Ptie_true, tk, chT, dt, atkT);

    DF1_MEAS(k)  = df1_meas;
    DF2_MEAS(k)  = df2_meas;
    PTIE_MEAS(k) = Ptie_meas;

    % ---- RK4 integration step (measured signals held constant, ZOH, across sub-stages) ----
    k1 = lfc_deriv(x,           df1_meas, df2_meas, Ptie_meas, PL1, PL2, p);
    k2 = lfc_deriv(x + dt/2*k1, df1_meas, df2_meas, Ptie_meas, PL1, PL2, p);
    k3 = lfc_deriv(x + dt/2*k2, df1_meas, df2_meas, Ptie_meas, PL1, PL2, p);
    k4 = lfc_deriv(x + dt*k3,   df1_meas, df2_meas, Ptie_meas, PL1, PL2, p);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    X(:,k+1) = x;
end
DF1_MEAS(N)  = DF1_MEAS(N-1);
DF2_MEAS(N)  = DF2_MEAS(N-1);
PTIE_MEAS(N) = PTIE_MEAS(N-1);

Pv1 = X(1,:); Pg1 = X(2,:); df1 = X(3,:);
Pv2 = X(4,:); Pg2 = X(5,:); df2 = X(6,:);
Ptie = X(7,:);

%% ==================== PLOT 1: TRUE FREQUENCY DEVIATIONS ====================
figure('Name','Frequency Deviation (true physical state)');
plot(t, df1, 'y', 'LineWidth', 1.4); hold on;
plot(t, df2, 'b', 'LineWidth', 1.4);
shade_attack_window(attack);
grid on;
xlabel('Time (s)'); ylabel('\Delta f (p.u.)');
title(sprintf('True \\Delta f -- load: %s/%s, attack: %s on %s/%s', ...
      dist.type, dist.area, attack.type, attack.target, attack.area));
legend('\Delta f_1 (Area 1)','\Delta f_2 (Area 2)','Location','best');

%% ==================== PLOT 2: TRUE vs MEASURED FREQUENCY (attack visibility) ====================
figure('Name','True vs Measured Frequency (attack effect)');
subplot(2,1,1);
plot(t, df1, 'y', 'LineWidth', 1.4); hold on;
plot(t, DF1_MEAS, 'r--', 'LineWidth', 1.0);
shade_attack_window(attack);
grid on; xlabel('Time (s)'); ylabel('\Delta f_1 (p.u.)');
title('Area 1: true (solid) vs what the controller sees (dashed)');
legend('True \Delta f_1','Measured \Delta f_1','Location','best');

subplot(2,1,2);
plot(t, df2, 'b', 'LineWidth', 1.4); hold on;
plot(t, DF2_MEAS, 'r--', 'LineWidth', 1.0);
shade_attack_window(attack);
grid on; xlabel('Time (s)'); ylabel('\Delta f_2 (p.u.)');
title('Area 2: true (solid) vs what the controller sees (dashed)');
legend('True \Delta f_2','Measured \Delta f_2','Location','best');

%% ==================== PLOT 3: GENERATION & TIE-LINE POWER ====================
figure('Name','Generation and Tie-line Power');
plot(t, Pg1, 'y', 'LineWidth', 1.4); hold on;
plot(t, Pg2, 'b', 'LineWidth', 1.4);
plot(t, Ptie,'Color',[1 0.5 0],'LineStyle','--','LineWidth', 1.4);
plot(t, PTIE_MEAS, 'r:', 'LineWidth', 1.2);
shade_attack_window(attack);
grid on;
xlabel('Time (s)'); ylabel('Power (p.u.)');
title(sprintf('P_{g1}, P_{g2}, true & measured P_{tie} -- attack: %s on %s/%s', ...
      attack.type, attack.target, attack.area));
legend('P_{g1} (Area 1)','P_{g2} (Area 2)','True P_{tie}','Measured P_{tie}','Location','best');

%% ==================== PLOT 4: GOVERNOR OUTPUT (controller response) ====================
figure('Name','Governor Output (valve/gate position)');
plot(t, Pv1, 'y', 'LineWidth', 1.4); hold on;
plot(t, Pv2, 'b', 'LineWidth', 1.4);
shade_attack_window(attack);
grid on;
xlabel('Time (s)'); ylabel('P_v (p.u.)');
title('Governor valve/gate position -- shows controller reaction to the attack');
legend('P_{v1} (Area 1)','P_{v2} (Area 2)','Location','best');

%% ==================== HELPER FUNCTIONS ====================
function ch = init_channel(buf_len)
    ch.hist = zeros(1,buf_len);
    ch.replay_seg = [];
    ch.replay_started = false;
    ch.replay_t0 = 0;
end

function [meas, ch] = process_channel(true_val, t, ch, dt, atk)
    active = atk.enable && (t >= atk.tstart) && (t <= atk.tend);
    meas = true_val;

    if active
        switch atk.type
            case 'fdia'
                meas = true_val + atk.bias;

            case 'delay'
                nshift = max(1, round(atk.delay/dt));
                nshift = min(nshift, numel(ch.hist));
                meas = ch.hist(end-nshift+1);

            case 'replay'
                if ~ch.replay_started
                    seglen = max(1, min(round(atk.replay_len/dt), numel(ch.hist)));
                    ch.replay_seg = ch.hist(end-seglen+1:end);
                    ch.replay_started = true;
                    ch.replay_t0 = t;
                end
                idx = mod(round((t-ch.replay_t0)/dt), numel(ch.replay_seg)) + 1;
                meas = ch.replay_seg(idx);

            case 'noise'
                meas = true_val + atk.noise_std*randn();

            case 'dos'
                meas = ch.hist(end);

            case 'packetdrop'
                if rand() < atk.drop_prob
                    meas = ch.hist(end);
                else
                    meas = true_val;
                end
        end
    else
        ch.replay_started = false;   % reset so a re-entered window records fresh
    end

    ch.hist = [ch.hist(2:end), meas];
end

function dx = lfc_deriv(x, df1_meas, df2_meas, Ptie_meas, PL1, PL2, p)
    Pv1=x(1); Pg1=x(2); df1=x(3);
    Pv2=x(4); Pg2=x(5); df2=x(6);
    Ptie=x(7); z1=x(8); z2=x(9); %#ok<NASGU>

    % ACE uses the (possibly attacked) MEASURED tie-line power and frequency
    ACE1 = Ptie_meas + p.B1*df1_meas;
    ACE2 = -Ptie_meas + p.B2*df2_meas;

    % Physical swing equation & tie-line ALWAYS use the TRUE states
    ddf1  = (Pg1 - PL1 - Ptie - p.D1*df1)/(2*p.H1);
    ddf2  = (Pg2 - PL2 + Ptie - p.D2*df2)/(2*p.H2);
    dPtie = p.Ktie*(df1 - df2);

    % Analytic d(ACE)/dt for the PID D-term (based on true physical rates)
    dACE1 = dPtie + p.B1*ddf1;
    dACE2 = -dPtie + p.B2*ddf2;

    Pc1 = -(p.Kp1*ACE1 + p.Ki1*z1 + p.Kd1*dACE1);
    Pc2 = -(p.Kp2*ACE2 + p.Ki2*z2 + p.Kd2*dACE2);

    dPv1 = (Pc1 - (1/p.R1)*df1_meas - Pv1)/p.Tg1;
    dPv2 = (Pc2 - (1/p.R2)*df2_meas - Pv2)/p.Tg2;

    dPg1 = (Pv1 - Pg1)/p.Tt1;
    dPg2 = (Pv2 - Pg2)/p.Tt2;

    dz1 = ACE1;
    dz2 = ACE2;

    dx = [dPv1; dPg1; ddf1; dPv2; dPg2; ddf2; dPtie; dz1; dz2];
end

function shade_attack_window(attack)
    if attack.enable
        xline(attack.tstart, 'k--', 'Attack start', 'LabelVerticalAlignment','bottom');
        xline(attack.tend,   'k--', 'Attack end',   'LabelVerticalAlignment','bottom');
    end
end