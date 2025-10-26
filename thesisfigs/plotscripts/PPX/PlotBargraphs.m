% ===============================
% Power Measurement Plot Script (With Combined Uncertainty + Debugging Vectors + PP3 & PP4)
% ===============================
% Assumes Excel files are in the same folder as this script

%% ===== File Mappings =====
voltage_files = {
    fullfile('PP3', 'PP3_V_5V.xlsx'), ...
    'PP4_V_5V.xlsx', ...
    'PP6_V_5V.xlsx', ...
    'PP8_V_5V.xlsx', ...
    'PP9_V_5V.xlsx'
};

current_idle_files = {
    'PP6_I_500uA.xlsx', ...
    'PP8_I_5mA.xlsx', ...
    'PP9_I_5mA.xlsx'
};

current_active_files = {
    'PP6_I_500uA.xlsx', ...
    'PP8_I_5mA_Active.xlsx', ...
    'PP9_I_5mA_Active.xlsx'
};

probes_voltage = {'PP3', 'PP4', 'PP6', 'PP8', 'PP9'};
probes_current = {'PP6', 'PP8', 'PP9'};
scene_labels = {'Scenario 2 Idle Scene', 'Scenario 3 Active Scene'};

a_voltage = repmat(0.05 / 100, 1, 5);
b_voltage = repmat(500e-6, 1, 5);
a_current = [0.15, 0.15, 0.15] / 100;
b_current = [0.2e-6, 1e-6, 1e-6];

%% ===== Voltage Analysis =====
n_voltage = length(voltage_files);
voltage_means = zeros(1, n_voltage);
voltage_u_combined = zeros(1, n_voltage);
voltage_uA = zeros(1, n_voltage);
voltage_uB = zeros(1, n_voltage);
voltage_std = zeros(1, n_voltage);
voltage_raw = cell(1, n_voltage);

for i = 1:n_voltage
    data = readtable(voltage_files{i});
    col = find(contains(data.Properties.VariableNames, 'DCV'), 1);
    values = data{:, col};
    values = values(~isnan(values));

    if mean(values) > 1000
        values = values * 1e-6;
        fprintf('Corrected voltage scale for %s\n', voltage_files{i});
    end

    voltage_raw{i} = values;
    n = length(values);
    mu = mean(values);
    std_dev = std(values);
    uA = std_dev / sqrt(n);
    uB = sqrt((a_voltage(i) * mu)^2 + b_voltage(i)^2);
    u_combined = sqrt(uA^2 + uB^2);

    voltage_means(i) = mu;
    voltage_std(i) = std_dev;
    voltage_uA(i) = uA;
    voltage_uB(i) = uB;
    voltage_u_combined(i) = u_combined;
end

%% === Voltage Plot with Rounded-Up Uncertainties ===
figure(1); clf;
bar_handle_voltage = bar(voltage_means);
bar_handle_voltage.FaceColor = [0.6 0.6 0.6];  % solid gray bars
bar_handle_voltage.EdgeColor = 'none';

hold on;
for i = 1:n_voltage
    u_rounded = roundUp(voltage_u_combined(i), 3);  % round uncertainty to 4 decimals
    label_str = sprintf('%.3f ± %.3f', voltage_means(i), u_rounded);
    text(i, voltage_means(i) + u_rounded * 1.1, label_str, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 26);
end
hold off;

title('Voltage Readings by Probing Point', 'FontSize', 30);
ylabel('Voltage (V)', 'FontSize', 30);
xticks(1:n_voltage);
xticklabels(probes_voltage);
set(gca, 'FontSize', 28);
grid off;

%% ===== Current Analysis (Idle vs Active) =====
n_current = length(current_idle_files);
current_means = zeros(2, n_current);
current_u_combined = zeros(2, n_current);
current_uA = zeros(2, n_current);
current_uB = zeros(2, n_current);
current_std = zeros(2, n_current);
current_raw = cell(2, n_current);

% --- Idle ---
for i = 1:n_current
    data = readtable(current_idle_files{i});
    col = find(contains(data.Properties.VariableNames, 'DCI'), 1);
    values = data{:, col};
    values = values(~isnan(values));

    current_raw{1, i} = values;
    n = length(values);
    mu = mean(values);
    std_dev = std(values);
    uA = std_dev / sqrt(n);
    uB = sqrt((a_current(i) * mu)^2 + b_current(i)^2);
    u_combined = sqrt(uA^2 + uB^2);

    current_means(1, i) = mu;
    current_std(1, i) = std_dev;
    current_uA(1, i) = uA;
    current_uB(1, i) = uB;
    current_u_combined(1, i) = u_combined;
end

% --- Active ---
for i = 1:n_current
    data = readtable(current_active_files{i});
    col = find(contains(data.Properties.VariableNames, 'DCI'), 1);
    values = data{:, col};
    values = values(~isnan(values));

    current_raw{2, i} = values;
    n = length(values);
    mu = mean(values);
    std_dev = std(values);
    uA = std_dev / sqrt(n);
    uB = sqrt((a_current(i) * mu)^2 + b_current(i)^2);
    u_combined = sqrt(uA^2 + uB^2);

    current_means(2, i) = mu;
    current_std(2, i) = std_dev;
    current_uA(2, i) = uA;
    current_uB(2, i) = uB;
    current_u_combined(2, i) = u_combined;
end

%% === Current Plot with Rounded-Up Uncertainties ===
figure(2); clf;
bar_handle = bar(current_means');
hold on;

% Set custom bar colors: black and gray
bar_handle(1).FaceColor = [0 0 0];       % solid black
bar_handle(2).FaceColor = [0.6 0.6 0.6]; % medium gray
bar_handle(1).EdgeColor = 'none';
bar_handle(2).EdgeColor = 'none';

% Add text labels with mean ± rounded-up uncertainty above each bar
for i = 1:n_current
    for j = 1:2
        x = bar_handle(j).XData(i) + bar_handle(j).XOffset;
        y = current_means(j, i);
        err_raw = current_u_combined(j, i);
        err = roundUp(err_raw, 1);  % round uncertainty up to 1 decimals
        label_str = sprintf('%.1f ± %.1f', y, err);
        text(x, y + err * 1.05, label_str, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontSize', 24);
    end
end

hold off;
title('Current Readings for Camera: Idle vs Active Scene', 'FontSize', 30);
ylabel('Current [\muA]', 'FontSize', 30);
xticklabels(probes_current);
legend(scene_labels, 'Location', 'NorthWest');
set(gca, 'FontSize', 28);

ax = gca;
ax.YRuler.Exponent = 0;   % keep numbers, no 10^n
ytickformat('%.0f')       % integer labels, no scientific notation
grid off;

%% ===== Camera Power Computation (Extended with Per-Rail Contributions) =====
% Computes total power and per-rail contribution + uncertainties for camera

% Define camera supply indices
camera_voltage_indices = [3, 4, 5];  % PP6, PP8, PP9 in voltage_means
camera_current_indices = 1:3;        % Corresponding rows in current_means

% Preallocate
P_camera = zeros(2, 1);               % Total camera power in W
u_P_camera = zeros(2, 1);             % Combined uncertainty in W

P_individual_mW = zeros(2, 3);        % Individual power contributions in mW
u_individual_mW = zeros(2, 3);        % Their uncertainties

% Loop over Idle (1) and Active (2) scenes
for scene = 1:2
    P_sum = 0;
    u_sum_sq = 0;

    for i = 1:3
        % Extract voltage and current with uncertainties
        V = voltage_means(camera_voltage_indices(i));
        uV = voltage_u_combined(camera_voltage_indices(i));
        I = current_means(scene, camera_current_indices(i)) * 1e-6;
        uI = current_u_combined(scene, camera_current_indices(i)) * 1e-6;

        % Power and uncertainty
        P = V * I;  % in W
        uP = P * sqrt((uV / V)^2 + (uI / I)^2);

        % Save per-rail result in mW
        P_individual_mW(scene, i) = P * 1e3;
        u_individual_mW(scene, i) = uP * 1e3;

        % Accumulate for total power
        P_sum = P_sum + P;
        u_sum_sq = u_sum_sq + uP^2;
    end

    % Save total power and uncertainty (RSS)
    P_camera(scene) = P_sum;
    u_P_camera(scene) = sqrt(u_sum_sq);
end


%% === Print Summary (in mW) ===
fprintf('\n=== Total Camera Power (mW ± uncertainty) ===\n');
fprintf('Idle Scene  : %.3f ± %.3f mW\n', ...
    P_camera(1) * 1e3, roundUp(u_P_camera(1) * 1e3, 3));
fprintf('Active Scene: %.3f ± %.3f mW\n', ...
    P_camera(2) * 1e3, roundUp(u_P_camera(2) * 1e3, 3));

fprintf('\n=== Per-Rail Camera Power (mW ± uncertainty) ===\n');
camera_probes = [6, 8, 9];
for i = 1:3
    fprintf('[PP%d]  Idle   : %.3f ± %.3f mW\n', ...
        camera_probes(i), ...
        P_individual_mW(1, i), ...
        roundUp(u_individual_mW(1, i), 3));
    fprintf('        Active : %.3f ± %.3f mW\n', ...
        P_individual_mW(2, i), ...
        roundUp(u_individual_mW(2, i), 3));
end


%% === Helper Functions ===
function rounded = roundUp(x, decimals)
    factor = 10^decimals;
    rounded = ceil(x * factor) / factor;
end


