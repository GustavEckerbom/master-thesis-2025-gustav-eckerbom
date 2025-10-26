% ==========================================
% Simplified Camera Power Estimation (GUM)
% Includes Active and Idle Scene Comparison
% ==========================================

% Files per probing point
voltage_files = {
    'PP6_V_5V.xlsx', 
    'PP8_V_5V.xlsx', 
    'PP9_V_5V.xlsx'
};

current_files_idle = {
    'PP6_I_500uA.xlsx', 
    'PP8_I_5mA.xlsx', 
    'PP9_I_5mA.xlsx'
};

current_files_active = {
    'PP6_I_500uA.xlsx', 
    'PP8_I_5mA_Active.xlsx', 
    'PP9_I_5mA.xlsx'
};

probes = {'PP6', 'PP8', 'PP9'};
current_ranges = {'500uA', '5mA', '5mA'};

% Containers
P_idle = zeros(1, 3);
U_idle = zeros(1, 3);
P_active = zeros(1, 3);
U_active = zeros(1, 3);

fprintf('===== Power Analysis Per Probing Point =====\n');

for i = 1:3
    % --- Idle Scene ---
    [P_idle(i), U_idle(i), ~, ~] = compute_power(voltage_files{i}, current_files_idle{i}, current_ranges{i}, probes{i});
    
    % --- Active Scene (only PP8 changes)
    if strcmp(probes{i}, 'PP8')
        [P_active(i), U_active(i), ~, ~] = compute_power(voltage_files{i}, current_files_active{i}, current_ranges{i}, probes{i});
    else
        P_active(i) = P_idle(i);
        U_active(i) = U_idle(i);
    end
    
    fprintf('\n[%s]\n', probes{i});
    fprintf('  Idle  Power:  %.6f W ± %.6f W\n', P_idle(i), U_idle(i));
    fprintf('  Active Power: %.6f W ± %.6f W\n', P_active(i), U_active(i));
end

% === Totals ===
P_total_idle = sum(P_idle);
U_total_idle = sqrt(sum(U_idle.^2));

P_total_active = sum(P_active);
U_total_active = sqrt(sum(U_active.^2));

fprintf('\n===== TOTAL CAMERA POWER =====\n');
fprintf('Idle Scene:   %.6f W ± %.6f W (%.3f mW ± %.3f mW)\n', ...
    P_total_idle, U_total_idle, P_total_idle*1000, U_total_idle*1000);
fprintf('Active Scene: %.6f W ± %.6f W (%.3f mW ± %.3f mW)\n', ...
    P_total_active, U_total_active, P_total_active*1000, U_total_active*1000);

% ==================================================
% === Function must be placed AFTER main script ====
% ==================================================

function [P_i, u_P, V_mean, I_mean] = compute_power(vfile, ifile, range, probe)
    % --- Load Voltage ---
    V_table = readtable(vfile);
    V_col = find(contains(V_table.Properties.VariableNames, 'DCV'), 1);
    V_raw = V_table{:, V_col};
    V_vals = V_raw(~isnan(V_raw));

    if strcmp(probe, 'PP6')
        V_vals = V_vals;  % already in V
    else
        V_vals = V_vals * 1e-6;  % µV → V
    end
    V_mean = mean(V_vals);

    % --- Load Current ---
    I_table = readtable(ifile);
    I_col = find(contains(I_table.Properties.VariableNames, 'DCI'), 1);
    I_raw = I_table{:, I_col};
    I_vals = I_raw(~isnan(I_raw)) * 1e-6;  % µA → A
    I_mean = mean(I_vals);

    % --- Power ---
    P_i = V_mean * I_mean;

    % --- Uncertainty (GUM) ---
    uA_V = std(V_vals, 1);
    uA_I = std(I_vals, 1);
    uB_V = 0.0005 * V_mean + 50e-6;

    switch range
        case '500uA'
            uB_I = 0.0015 * I_mean + 0.2e-6;
        case '5mA'
            uB_I = 0.0015 * I_mean + 20e-6;
        otherwise
            error('Unknown current range for %s', probe);
    end

    u_V = sqrt(uA_V^2 + uB_V^2);
    u_I = sqrt(uA_I^2 + uB_I^2);
    u_P = P_i * sqrt((u_V / V_mean)^2 + (u_I / I_mean)^2);
end
