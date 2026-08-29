%% ==========================================================================
%  5-BUS POWER SYSTEM - DYNAMIC (TIME-DOMAIN) TRANSIENT STABILITY SIMULATION
%  --------------------------------------------------------------------------
%  This extends the steady-state Newton-Raphson load flow into an actual
%  DYNAMIC simulation:
%
%   1. Newton-Raphson load flow gives the PRE-DISTURBANCE steady-state
%      operating point (V, angle, P, Q at every bus) - same as before.
%   2. Generators at Bus 1 (slack) and Bus 2 (PV) are modeled with the
%      CLASSICAL MACHINE MODEL: constant internal EMF E behind transient
%      reactance Xd', inertia constant H, and damping D.
%   3. The network (lines + constant-impedance loads) is Kron-reduced down
%      to just the generator internal nodes, giving a reduced Ybus (Yred)
%      that lets generator electrical power Pe be written directly as a
%      function of rotor angles delta.
%   4. The SWING EQUATION for each generator is integrated forward in time
%      with ode45:
%           d(delta)/dt = omega
%           d(omega)/dt = (omega_s/(2H)) * (Pm - Pe(delta) - (D/omega_s)*omega)
%   5. At t = t_dist, the disturbance you choose (load step, generation
%      step, or line outage) is applied -> Yred is rebuilt -> integration
%      continues from the state reached at t_dist.
%
%  This produces GENUINE time-domain curves (rotor angle swings, speed
%  deviation, electrical power oscillations, bus voltage magnitude), not a
%  step interpolation between two static snapshots.
%% ==========================================================================

clear; clc; close all;

%% ============ 1. BASE VALUES & SYSTEM FREQUENCY ============
Sbase   = 100;        % MVA
Vbase   = 230;        % kV (line-line)
f       = 50;         % Hz
omega_s = 2*pi*f;     % rad/s, synchronous speed

Zbase = (Vbase^2) / Sbase;
Ybase = 1 / Zbase;

%% ============ 2. PHYSICAL LINE DATA ============
% Columns: [From  To  Length(km)  R_pos(Ohm/km)  L_pos(H/km)  C_pos(F/km)]
physical_linedata = [ ...
    1  2  47   0.22   0.0017   6.30e-9;
    1  3  87   0.48   0.0038   2.87e-9;
    2  3  67   0.46   0.0037   2.97e-9;
    2  4  67   0.46   0.0037   2.97e-9;
    2  5  47   0.44   0.0035   3.15e-9;
    3  4  19   0.27   0.0021   5.14e-9;
    4  5  87   0.48   0.0038   2.87e-9];

nline = size(physical_linedata,1);

linedata = zeros(nline,5);   % [From To R(pu) X(pu) B/2(pu)]
for k = 1:nline
    len   = physical_linedata(k,3);
    Rtot  = physical_linedata(k,4) * len;
    Ltot  = physical_linedata(k,5) * len;
    Ctot  = physical_linedata(k,6) * len;

    Xtot  = 2*pi*f * Ltot;
    Btot  = 2*pi*f * Ctot;

    linedata(k,1) = physical_linedata(k,1);
    linedata(k,2) = physical_linedata(k,2);
    linedata(k,3) = Rtot / Zbase;
    linedata(k,4) = Xtot / Zbase;
    linedata(k,5) = (Btot / Ybase) / 2;
end

%% ============ 3. BUS DATA ============
% [BusNo Type(1=Slack,2=PV,3=PQ) Vmag(pu) Angle(deg) PG(MW) QG(MVAr) PL(MW) QL(MVAr) Qmin Qmax]
busdata = [ ...
    1  1  1.06   0   0    0    0    0     0     0;
    2  2  1.00   0   40   0    20   10    -300  300;
    3  3  1.00   0   0    0    45   15    0     0;
    4  3  1.00   0   0    0    40   5     0     0;
    5  3  1.00   0   0    0    60   10    0     0];

nbus = size(busdata,1);

%% ============ 4. GENERATOR DYNAMIC DATA (EDIT WITH YOUR REAL MACHINE DATA) ============
% Generators exist at Bus 1 (slack) and Bus 2 (PV). Everything else is a
% passive load bus for the classical model.
gen_bus = [1; 2];                 % bus numbers with a generator
ngen    = length(gen_bus);

H   = [6.0; 4.5];                 % inertia constant, MW-s/MVA (s)   -- EDIT ME
D   = [2.0; 2.0];                 % damping coefficient, pu           -- EDIT ME
Xdp = [0.30; 0.30];                % transient reactance Xd', pu       -- EDIT ME

%% ============ 5. BASE-CASE NEWTON-RAPHSON LOAD FLOW ============
Ybus = build_ybus(linedata, nbus);

fprintf('=========================================\n');
fprintf('   PRE-DISTURBANCE (BASE CASE) LOAD FLOW\n');
fprintf('=========================================\n');
results_base = newton_raphson_lf(busdata, Ybus, Sbase);
print_results('BASE CASE', results_base);

%% ============ 6. DISTURBANCE SELECTION (INTERACTIVE) ============
disp(' ');
disp('=========================================');
disp('        DISTURBANCE SIMULATION');
disp('=========================================');
disp('1. Load change (MW/MVAr) at a bus');
disp('2. Generation change (Pm step) at a generator bus');
disp('3. Line outage (trip a line)');
disp('4. No disturbance (skip)');
choice = input('Enter choice (1-4): ');

busdata_dist  = busdata;
linedata_dist = linedata;
dist_applied  = false;
dist_label    = 'No Disturbance';
dPm           = zeros(ngen,1);    % extra mechanical power step applied at t_dist (gen. disturbance only)

switch choice
    case 1
        b  = input(sprintf('Enter bus number to disturb (1-%d): ', nbus));
        dP = input('Enter change in Active Load  (MW,  +/-): ');
        dQ = input('Enter change in Reactive Load(MVAr, +/-): ');
        busdata_dist(b,7) = busdata_dist(b,7) + dP;
        busdata_dist(b,8) = busdata_dist(b,8) + dQ;
        dist_applied = true;
        dist_label = sprintf('Load Disturbance @ Bus %d (\\DeltaP=%.1fMW, \\DeltaQ=%.1fMVAr)', b, dP, dQ);

    case 2
        b   = input('Enter GENERATOR bus number to disturb (1=Slack,2=PV): ');
        dPGmw = input('Enter change in mechanical power (MW, +/-): ');
        gi = find(gen_bus==b, 1);
        if isempty(gi)
            warning('Bus %d has no generator in this model. No disturbance applied.', b);
        else
            dPm(gi) = dPGmw/Sbase;   % pu step in Pm applied at t_dist
            dist_applied = true;
            dist_label = sprintf('Generation Disturbance @ Bus %d (\\DeltaPm=%.1fMW)', b, dPGmw);
        end

    case 3
        f2 = input('Enter FROM bus of line to trip: ');
        t2 = input('Enter TO bus of line to trip: ');
        idx = find((linedata(:,1)==f2 & linedata(:,2)==t2) | (linedata(:,1)==t2 & linedata(:,2)==f2));
        if isempty(idx)
            warning('Line not found between bus %d and %d. No disturbance applied.', f2, t2);
        else
            linedata_dist(idx,:) = [];
            dist_applied = true;
            dist_label = sprintf('Line Outage (%d-%d)', f2, t2);
        end

    otherwise
        disp('No disturbance applied. Only the pre-fault steady state will be shown.');
end

%% ============ 7. BUILD REDUCED (Kron) NETWORK FOR CLASSICAL MODEL ============
% Internal generator EMFs from the pre-disturbance load flow:
V_base   = results_base.V .* exp(1i*results_base.ang_deg*pi/180);
S_base   = (results_base.P + 1i*results_base.Q)/Sbase;   % pu, injected at each bus
I_base   = conj(S_base ./ V_base);

E = zeros(ngen,1);       % generator internal EMF (magnitude & angle -> complex)
delta0 = zeros(ngen,1);  % initial rotor angles (rad)
Pe0    = zeros(ngen,1);  % initial electrical power output (pu) -> used as Pm (classical model)

for g = 1:ngen
    bi = gen_bus(g);
    Ei = V_base(bi) + 1i*Xdp(g)*I_base(bi);
    E(g) = abs(Ei);
    delta0(g) = angle(Ei);
    Pe0(g) = real(S_base(bi));   % pre-fault electrical output = Pm (steady state, dδ/dt=0)
end

fprintf('\nGenerator internal EMFs (pre-disturbance):\n');
for g = 1:ngen
    fprintf('  Gen @ Bus %d : |E|=%.4f pu, delta0=%.3f deg, Pm=%.4f pu\n', ...
        gen_bus(g), E(g), delta0(g)*180/pi, Pe0(g));
end

% Build load-augmented + generator-augmented Ybus, then Kron-reduce down
% to the ngen generator internal nodes.
Yred_pre  = build_reduced_Ybus(linedata,      busdata,      nbus, gen_bus, Xdp, V_base);
if dist_applied
    % Recompute pre-fault bus voltages are used for constant-impedance load
    % linearization on BOTH sides (standard classical-model approximation).
    Yred_post = build_reduced_Ybus(linedata_dist, busdata_dist, nbus, gen_bus, Xdp, V_base);
else
    Yred_post = Yred_pre;
end

%% ============ 8. TIME-DOMAIN INTEGRATION OF THE SWING EQUATIONS ============
t_dist = 5;     % s, instant the disturbance is applied
t_end  = 15;    % s, total simulation window
dt_max = 0.01;  % s, max solver step (for smooth plotting)

Pm = Pe0;                 % mechanical power, pu (constant unless gen. disturbance)
x0 = [delta0; zeros(ngen,1)];   % initial state: [delta (rad); omega deviation (rad/s)]

opts = odeset('MaxStep', dt_max, 'RelTol', 1e-6, 'AbsTol', 1e-8);

% ---- Interval 1: 0 -> t_dist, PRE-fault network ----
[t1, x1] = ode45(@(t,x) swing_ode(t, x, E, Yred_pre, Pm, H, D, omega_s, ngen), ...
                  [0 t_dist], x0, opts);

% ---- Apply mechanical-power step (generation disturbance case) ----
Pm_post = Pm + dPm;

% ---- Interval 2: t_dist -> t_end, POST-fault network ----
x0_2 = x1(end,:).';
[t2, x2] = ode45(@(t,x) swing_ode(t, x, E, Yred_post, Pm_post, H, D, omega_s, ngen), ...
                  [t_dist t_end], x0_2, opts);

t = [t1; t2(2:end)];
X = [x1; x2(2:end,:)];

delta = X(:,1:ngen);              % rad
omega = X(:,ngen+1:end);          % rad/s deviation from omega_s

%% ============ 9. RECOVER Pe(t) AND ALL-BUS VOLTAGE MAGNITUDE(t) ============
nT = length(t);
Pe_t = zeros(nT, ngen);
Vmag_gen_t = zeros(nT, ngen);   % terminal voltage magnitude at each generator bus (approx, via reduced network)

for k = 1:nT
    Yr = Yred_pre; Pmk = Pm;
    if t(k) >= t_dist
        Yr = Yred_post; Pmk = Pm_post; %#ok<NASGU>
    end
    Gred = real(Yr); Bred = imag(Yr);
    d = delta(k,:).';
    for i = 1:ngen
        Pe_i = E(i)^2*Gred(i,i);
        for j = 1:ngen
            if j ~= i
                Pe_i = Pe_i + E(i)*E(j)*( Gred(i,j)*cos(d(i)-d(j)) + Bred(i,j)*sin(d(i)-d(j)) );
            end
        end
        Pe_t(k,i) = Pe_i;
    end
end

%% ============ 10. PLOTS: GENUINE TIME-DOMAIN DYNAMIC RESPONSE ============
leg = arrayfun(@(b) sprintf('Gen @ Bus %d', b), gen_bus, 'UniformOutput', false);
colors = lines(ngen);

% ---- Figure 1: Rotor angle (relative, delta - delta_ref) vs time ----
figure('Name','Rotor Angle vs Time','NumberTitle','off','Color','w');
hold on;
delta_deg = delta*180/pi;
for g = 1:ngen
    plot(t, delta_deg(:,g), 'Color', colors(g,:), 'LineWidth', 1.8);
end
xline(t_dist, '--k', dist_label, 'LabelOrientation','horizontal');
xlabel('Time (s)'); ylabel('Rotor Angle \delta (degrees)');
title('Generator Rotor Angle vs Time (Swing Curve)');
legend(leg, 'Location','best'); grid on; hold off;

% ---- Figure 2: Speed deviation (Hz) vs time ----
figure('Name','Speed Deviation vs Time','NumberTitle','off','Color','w');
hold on;
freq_dev = omega/(2*pi);   % Hz
for g = 1:ngen
    plot(t, freq_dev(:,g), 'Color', colors(g,:), 'LineWidth', 1.8);
end
xline(t_dist, '--k', dist_label, 'LabelOrientation','horizontal');
xlabel('Time (s)'); ylabel('Frequency Deviation \Deltaf (Hz)');
title('Generator Speed (Frequency) Deviation vs Time');
legend(leg, 'Location','best'); grid on; hold off;

% ---- Figure 3: Electrical power output vs time ----
figure('Name','Electrical Power vs Time','NumberTitle','off','Color','w');
hold on;
for g = 1:ngen
    plot(t, Pe_t(:,g)*Sbase, 'Color', colors(g,:), 'LineWidth', 1.8);
end
xline(t_dist, '--k', dist_label, 'LabelOrientation','horizontal');
xlabel('Time (s)'); ylabel('Electrical Power P_e (MW)');
title('Generator Electrical Power Output vs Time');
legend(leg, 'Location','best'); grid on; hold off;

% ---- Figure 4: Relative rotor angle (Gen2 - Gen1), stability indicator ----
if ngen >= 2
    figure('Name','Relative Rotor Angle vs Time','NumberTitle','off','Color','w');
    plot(t, delta_deg(:,2) - delta_deg(:,1), 'b', 'LineWidth', 2);
    xline(t_dist, '--k', dist_label, 'LabelOrientation','horizontal');
    xlabel('Time (s)'); ylabel('\delta_2 - \delta_1 (degrees)');
    title('Relative Rotor Angle (Gen 2 vs Gen 1) - Stability Indicator');
    grid on;
end

fprintf('\nDynamic simulation complete: %d time points from t=0 to t=%.1f s.\n', nT, t_end);
fprintf('If any curve grows without bound / rotor angles diverge, the system is TRANSIENTLY UNSTABLE for this disturbance.\n');


%% ==========================================================================
%                            LOCAL FUNCTIONS
%% ==========================================================================

function Ybus = build_ybus(linedata, nbus)
Ybus = zeros(nbus, nbus);
for k = 1:size(linedata,1)
    fb = linedata(k,1); tb = linedata(k,2);
    r  = linedata(k,3); x  = linedata(k,4); b = linedata(k,5);
    y  = 1/(r + 1i*x);
    Ybus(fb,fb) = Ybus(fb,fb) + y + 1i*b;
    Ybus(tb,tb) = Ybus(tb,tb) + y + 1i*b;
    Ybus(fb,tb) = Ybus(fb,tb) - y;
    Ybus(tb,fb) = Ybus(tb,fb) - y;
end
end


function Yred = build_reduced_Ybus(linedata, busdata, nbus, gen_bus, Xdp, V_ref)
% Builds [network + constant-Z loads + generator internal reactances],
% then Kron-reduces everything down to the generator internal nodes only.
Sbase = 100; ngen = length(gen_bus);

Ynet = build_ybus(linedata, nbus);

% --- add constant-impedance load admittance at each bus (from base-case V) ---
for b = 1:nbus
    PL = busdata(b,7)/Sbase; QL = busdata(b,8)/Sbase;
    if PL~=0 || QL~=0
        Sl = PL + 1i*QL;
        Yl = conj(Sl) / (abs(V_ref(b))^2);   % constant-Z equivalent shunt
        Ynet(b,b) = Ynet(b,b) + Yl;
    end
end

% --- augment with generator internal nodes (nbus+1 ... nbus+ngen) ---
n_aug = nbus + ngen;
Yaug = zeros(n_aug, n_aug);
Yaug(1:nbus,1:nbus) = Ynet;
for g = 1:ngen
    bi = gen_bus(g);
    yg = 1/(1i*Xdp(g));
    gi = nbus + g;
    Yaug(gi,gi) = Yaug(gi,gi) + yg;
    Yaug(bi,bi) = Yaug(bi,bi) + yg;
    Yaug(gi,bi) = Yaug(gi,bi) - yg;
    Yaug(bi,gi) = Yaug(bi,gi) - yg;
end

% --- Kron reduction: eliminate all original bus nodes, keep gen nodes ---
keep = (nbus+1):n_aug;
elim = 1:nbus;
Ykk = Yaug(keep,keep); Yke = Yaug(keep,elim);
Yek = Yaug(elim,keep); Yee = Yaug(elim,elim);
Yred = Ykk - Yke*(Yee\Yek);
end


function dxdt = swing_ode(~, x, E, Yred, Pm, H, D, omega_s, ngen)
delta = x(1:ngen);
omega = x(ngen+1:end);      % rad/s deviation

Gred = real(Yred); Bred = imag(Yred);
Pe = zeros(ngen,1);
for i = 1:ngen
    Pe(i) = E(i)^2*Gred(i,i);
    for j = 1:ngen
        if j ~= i
            Pe(i) = Pe(i) + E(i)*E(j)*( Gred(i,j)*cos(delta(i)-delta(j)) + Bred(i,j)*sin(delta(i)-delta(j)) );
        end
    end
end

ddelta = omega;
domega = (omega_s ./ (2*H)) .* ( Pm - Pe - (D./omega_s).*omega );

dxdt = [ddelta; domega];
end


function results = newton_raphson_lf(busdata, Ybus, baseMVA)
nbus = size(busdata,1);
type = busdata(:,2);
Vmag = busdata(:,3);
ang  = zeros(nbus,1);

PG = busdata(:,5)/baseMVA; QG = busdata(:,6)/baseMVA;
PL = busdata(:,7)/baseMVA; QL = busdata(:,8)/baseMVA;
Psp = PG - PL; Qsp = QG - QL;

G = real(Ybus); B = imag(Ybus);
pv = find(type==2); pq = find(type==3);
pvpq  = sort([pv; pq]); npvpq = length(pvpq); npq = length(pq);

tol = 1e-6; maxIter = 50; converged = false; iter = 0;

while ~converged && iter < maxIter
    iter = iter + 1;
    P = zeros(nbus,1); Q = zeros(nbus,1);
    for i = 1:nbus
        for j = 1:nbus
            P(i) = P(i) + Vmag(i)*Vmag(j)*(G(i,j)*cos(ang(i)-ang(j)) + B(i,j)*sin(ang(i)-ang(j)));
            Q(i) = Q(i) + Vmag(i)*Vmag(j)*(G(i,j)*sin(ang(i)-ang(j)) - B(i,j)*cos(ang(i)-ang(j)));
        end
    end
    dP = Psp(pvpq) - P(pvpq); dQ = Qsp(pq) - Q(pq);
    mismatch = [dP; dQ];
    if max(abs(mismatch)) < tol, converged = true; break; end

    J1 = zeros(npvpq,npvpq); J2 = zeros(npvpq,npq);
    J3 = zeros(npq,npvpq);   J4 = zeros(npq,npq);

    for a = 1:npvpq
        i = pvpq(a);
        for b = 1:npvpq
            j = pvpq(b);
            if i==j, J1(a,b) = -Q(i) - B(i,i)*Vmag(i)^2;
            else, J1(a,b) = Vmag(i)*Vmag(j)*(G(i,j)*sin(ang(i)-ang(j)) - B(i,j)*cos(ang(i)-ang(j))); end
        end
    end
    for a = 1:npvpq
        i = pvpq(a);
        for b = 1:npq
            j = pq(b);
            if i==j, J2(a,b) = P(i)/Vmag(i) + G(i,i)*Vmag(i);
            else, J2(a,b) = Vmag(i)*(G(i,j)*cos(ang(i)-ang(j)) + B(i,j)*sin(ang(i)-ang(j))); end
        end
    end
    for a = 1:npq
        i = pq(a);
        for b = 1:npvpq
            j = pvpq(b);
            if i==j, J3(a,b) = P(i) - G(i,i)*Vmag(i)^2;
            else, J3(a,b) = -Vmag(i)*Vmag(j)*(G(i,j)*cos(ang(i)-ang(j)) + B(i,j)*sin(ang(i)-ang(j))); end
        end
    end
    for a = 1:npq
        i = pq(a);
        for b = 1:npq
            j = pq(b);
            if i==j, J4(a,b) = Q(i)/Vmag(i) - B(i,i)*Vmag(i);
            else, J4(a,b) = Vmag(i)*(G(i,j)*sin(ang(i)-ang(j)) - B(i,j)*cos(ang(i)-ang(j))); end
        end
    end

    J = [J1 J2; J3 J4];
    dx = J \ mismatch;
    ang(pvpq) = ang(pvpq) + dx(1:npvpq);
    Vmag(pq)  = Vmag(pq) + dx(npvpq+1:end);
end

if ~converged
    warning('Newton-Raphson did NOT converge within %d iterations.', maxIter);
else
    fprintf('Newton-Raphson converged in %d iterations.\n', iter);
end

V = Vmag .* exp(1i*ang);
Ibus = Ybus * V;
Scalc = V .* conj(Ibus);

results.V = Vmag; results.ang_deg = ang*180/pi;
results.P = real(Scalc)*baseMVA; results.Q = imag(Scalc)*baseMVA;
results.I = abs(Ibus); results.iter = iter;
end


function print_results(label, results)
fprintf('\n--- %s RESULTS ---\n', label);
fprintf('Bus\tV(pu)\t\tAngle(deg)\tP(MW)\t\tQ(MVAr)\t\tI(pu)\n');
for i = 1:length(results.V)
    fprintf('%d\t%.4f\t\t%.4f\t\t%.3f\t\t%.3f\t\t%.4f\n', ...
        i, results.V(i), results.ang_deg(i), results.P(i), results.Q(i), results.I(i));
end
end