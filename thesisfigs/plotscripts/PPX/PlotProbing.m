% Define filename
filename = 'PP8_I_5mA_Active.xlsx';

% Extract metadata from filename
tokens = regexp(filename, 'PP(\d+)_([A-Z])_(\d+mA)', 'tokens');
tokens = tokens{1};
probe_point = tokens{1};
measurement_type = tokens{2};
range_setting = tokens{3};

% Read with sanitized variable names
opts = detectImportOptions(filename);
disp(opts.VariableNames);  % Optional: check actual column names in console

% Find the column that used to be 'DCV(V)'
dcv_col = find(contains(opts.VariableNames, 'DCV'));

% Set that column to double
opts = setvartype(opts, opts.VariableNames{dcv_col}, 'double');
data = readtable(filename, opts);

% Extract and clean data
values = data{:, dcv_col};
values = values(~isnan(values));

% Compute stats
mean_val = mean(values);
std_val = std(values);

% Plot
figure;
bar(mean_val);
hold on;
errorbar(1, mean_val, std_val, 'k', 'LineWidth', 1.5, 'CapSize', 10);
hold off;

% Labels
title(sprintf('PP%s %s @ %s — Mean ± Std Dev', probe_point, measurement_type, range_setting));
ylabel('Value from DCV column');
xticklabels({sprintf('PP%s %s %s', probe_point, measurement_type, range_setting)});
