% =====================================================
% MCU Power Estimation across Clock Frequencies (GUM)
% Scenarios: Camera Off vs Camera Idle
% =====================================================

% Clock divisors and corresponding folder names
clock_divisors = {'0', '1', '3', '7', '15', '31'};
frequencies_MHz = 400 ./ (1 + str2double(clock_divisors));  % Derived freq in MHz

% Files for PP4 (fixed)
pp4_vfile = 'PP4_V_5V.xlsx';
pp4_ifile = 'PP4_I_500uA.xlsx';

% Files for PP3 (vary with freq, in subfolders)
pp3_voltage_file = fullfile('PP3', 'PP3_V_5V.xlsx');

% Current range lookup
pp3_range = '50mA';
pp4_range = '500uA';

% Initialize result tables
num_freqs = numel(clock_divisors);
results = table(frequencies_MHz', zeros(num_freqs,1), zeros(num_freqs,1), ...
                              zeros(num_freqs,1), zeros(num_freqs,1), ...
                              'VariableNames', ...
                              {'FreqMHz','P_Off','U_Off','P_Idle','U_Idle'});

% === Precompute PP4 (constant across frequencies) ===
[P_pp4, U_pp4, ~, ~] = compute_power(pp4_vfile, pp4_ifile, pp4_range, 'PP4');

% === Load PP3 Voltage once (shared) ===
[V_pp3_mean, uV_pp3] = load_voltage(pp3_voltage_file, 'PP3');

fprintf('===== MCU Power Estimation Across Frequencies =====\n');

for i = 1:num_freqs
    folder = fullfile('PP3', clock_divisors{i});
    
    % Off current file
    ifile_off = fullfile(folder, 'PP3_I_50mA_Off.xlsx');
    [P_pp3_off, U_pp3_off] = compute_with_shared_voltage(V_pp3_mean, uV_pp3, ifile_off, pp3_range);
    
    % Idle current file
    ifile_idle = fullfile(folder, 'PP3_I_50mA_Idle.xlsx');
    [P_pp3_idle, U_pp3_idle] = compute_with_shared_voltage(V_pp3_mean, uV_pp3, ifile_idle, pp3_range);
    
    % Combine PP3 + PP4 power, RSS uncertainty
    results.P_Off(i)  = P_pp3_off  + P_pp4;
    results.U_Off(i)  = sqrt(U_pp3_off^2  + U_pp4^2);
    results.P_Idle(i) = P_pp3_idle + P_pp4;
    results.U_Idle(i) = sqrt(U_pp3_idle^2 + U_pp4^2);
    
    % Report
    fprintf('\nClock Divisor: %s (%d MHz)\n', clock_divisors{i}, round(frequencies_MHz(i)));
    fprintf('  Camera Off:  %.6f W ± %.6f W\n', results.P_Off(i),  results.U_Off(i));
    fprintf('  Camera Idle: %.6f W ± %.6f W\n', results.P_Idle(i), results.U_Idle(i));
end
%% ===== Power vs Frequency Plot =====

figure(10); clf;
hold on;

% Convert W → mW for better readability
P_off_mW  = results.P_Off * 1000;
U_off_mW  = results.U_Off * 1000;
P_idle_mW = results.P_Idle * 1000;
U_idle_mW = results.U_Idle * 1000;

h1 = errorbar(results.FreqMHz, P_off_mW, U_off_mW, ...
    'Color', [247, 142, 140]/255, 'LineStyle', 'none', ...
    'LineWidth', 6, 'CapSize', 16, ...
    'DisplayName', 'Camera Off');

% Camera Idle: thick line with light blue (#8BC2FD)
h2 = errorbar(results.FreqMHz, P_idle_mW, U_idle_mW, ...
    'Color', [11, 0, 95]/255, 'LineStyle', 'none', ...
    'LineWidth', 6, 'CapSize', 16, ...
    'DisplayName', 'Camera Idle');

% Aesthetics
set(gca, 'XDir', 'reverse');  % Highest freq on the left
ax = gca;            % Get current axis handle
ax.FontSize = 41;    % Set tick label font size
xlim([0 425]);  % sets the visible x-axis range from 0 to 450
xlabel('MCU Peripheral Clock Frequency (MHz)', 'FontSize', 45);
ylabel('Power Consumption (mW)', 'FontSize', 45);
% title('MCU Power vs Clock Frequency', 'FontSize', 84);
legend([h2, h1], {'Idle Scene', 'Camera Off'}, ...
    'Location', 'southwest', 'FontSize', 45);
grid on;


%% ===== PD6-Off: Total MCU power from PP3 + PP4 (print only) =====
pp4_vfile_PD6 = 'PP4_V_5V.xlsx';
pp4_ifile_PD6 = 'PP4_I_500uA_PD6_Off.xlsx';

pp3_vfile_PD6 = 'PP3_V_5V.xlsx';
pp3_ifile_PD6 = 'PP3_I_50mA_PD6_Off.xlsx';

% Rail powers with GUM uncertainty (robust loaders handle staggered rows)
[P_PP4_PD6, U_PP4_PD6] = compute_power_flex(pp4_vfile_PD6, pp4_ifile_PD6, '500uA', 'PP4');
[P_PP3_PD6, U_PP3_PD6] = compute_power_flex(pp3_vfile_PD6, pp3_ifile_PD6, '50mA',  'PP3');

% Total = sum of rails, uncertainties combined by RSS
P_PD6_tot = P_PP3_PD6 + P_PP4_PD6;
U_PD6_tot = hypot(U_PP3_PD6, U_PP4_PD6);

% Pretty print (in both W and mW)
fprintf('\n===== PD6 (Camera Off) — Total MCU Power =====\n');
fprintf('PP3 rail :  %.6f W  ± %.6f W\n', P_PP3_PD6, U_PP3_PD6);
fprintf('PP4 rail :  %.6f W  ± %.6f W\n', P_PP4_PD6, U_PP4_PD6);
fprintf('----------------------------------------------\n');
fprintf('TOTAL     :  %.6f W  ± %.6f W   (%.3f mW ± %.3f mW)\n', ...
        P_PD6_tot, U_PD6_tot, P_PD6_tot*1e3, U_PD6_tot*1e3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% New robust helpers (work with DCV or DCI, and skip blank/every-other rows)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P, u_P] = compute_power_flex(vfile, ifile, range, probe)
    [V_mean, uV] = load_voltage_flex(vfile, probe);
    [I_mean, uI] = load_current_flex(ifile, range);
    P   = V_mean * I_mean;
    u_P = P * sqrt((uV / V_mean)^2 + (uI / I_mean)^2);
end

function [V_mean, uV] = load_voltage_flex(vfile, probe)
    T = readtable(vfile);

    % Prefer DCV if present (your files store numbers there)
    vcol = find(contains(T.Properties.VariableNames, 'DCV'), 1);
    if isempty(vcol)
        % fall back to any numeric column
        vcol = find(varfun(@isnumeric, T, 'OutputFormat','uniform'), 1);
    end

    V = T{:, vcol};
    V = V(~isnan(V));          % drops the “every-other” empty rows

    % PP3/PP4 voltage logs are in µV → convert to V
    if strcmp(probe,'PP3') || strcmp(probe,'PP4')
        V_vals = V * 1e-6;
    else
        V_vals = V;
    end

    V_mean = mean(V_vals);

    % Type A
    uA = std(V_vals, 1);

    % Type B (same model you used before)
    uB = 0.0005 * V_mean + 50e-6;

    uV = sqrt(uA.^2 + uB.^2);
end

function [I_mean, uI] = load_current_flex(ifile, range)
    T = readtable(ifile);

    % Your current data is often stored in the DCV(V) column; use it if there.
    icol = find(contains(T.Properties.VariableNames, 'DCI'), 1);
    if isempty(icol)
        icol = find(contains(T.Properties.VariableNames, 'DCV'), 1);
    end
    if isempty(icol)
        % fall back to any numeric column
        icol = find(varfun(@isnumeric, T, 'OutputFormat','uniform'), 1);
    end

    Iraw = T{:, icol};
    Iraw = Iraw(~isnan(Iraw));   % removes the interleaved blank rows

    % Your current logs are in µA → convert to A
    I_vals = Iraw * 1e-6;

    I_mean = mean(I_vals);

    % Type A
    uA = std(I_vals, 1);

    % Type B by range (same form as your existing code)
    switch range
        case '500uA'
            uB = 0.0015 * I_mean + 0.2e-6;
        case '50mA'
            uB = 0.0015 * I_mean + 100e-6;
        otherwise
            error('Unknown range: %s', range);
    end

    uI = sqrt(uA.^2 + uB.^2);
end



%% ===== Helper Functions =====

function [P, u_P, V_mean, I_mean] = compute_power(vfile, ifile, range, probe)
    [V_mean, uV] = load_voltage(vfile, probe);
    [I_mean, uI] = load_current(ifile, range);
    
    P = V_mean * I_mean;
    u_P = P * sqrt((uV / V_mean)^2 + (uI / I_mean)^2);
end

function [P, u_P] = compute_with_shared_voltage(V_mean, uV, ifile, range)
    [I_mean, uI] = load_current(ifile, range);
    P = V_mean * I_mean;
    u_P = P * sqrt((uV / V_mean)^2 + (uI / I_mean)^2);
end

function [V_mean, uV] = load_voltage(vfile, probe)
    T = readtable(vfile);
    col = find(contains(T.Properties.VariableNames, 'DCV'), 1);
    V = T{:, col}; V = V(~isnan(V));

    if strcmp(probe, 'PP3') || strcmp(probe, 'PP4')
        V_vals = V * 1e-6;  % µV → V
    else
        V_vals = V;  % already in V
    end

    V_mean = mean(V_vals);
    uA = std(V_vals, 1);
    uB = 0.0005 * V_mean + 50e-6;
    uV = sqrt(uA^2 + uB^2);
end

function [I_mean, uI] = load_current(ifile, range)
    T = readtable(ifile);
    col = find(contains(T.Properties.VariableNames, 'DCI'), 1);
    I = T{:, col}; I = I(~isnan(I)) * 1e-6;  % µA → A

    I_mean = mean(I);
    uA = std(I, 1);

    switch range
        case '500uA'
            uB = 0.0015 * I_mean + 0.2e-6;
        case '50mA'
            uB = 0.0015 * I_mean + 100e-6;
        otherwise
            error('Unknown range: %s', range);
    end
    uI = sqrt(uA^2 + uB^2);
end


