% Define data for CSI-2 interface
csi_events_per_sec = [0, 500e6]; % events per second
csi_power_mw = [24.1, 46.2];     % power in mW

% Define data for CPI interface
cpi_events_per_sec = [0, 50e6, 250e6]; 
cpi_power_mw = [5.7, 43.7, 45.0];

% Create figure
figure(1);
clf;
hold on;
grid off;

% Plot CSI-2 interface data: dashed line, filled circle markers
plot(csi_events_per_sec, csi_power_mw, 'ko', ...
    'LineWidth', 1.5, 'MarkerSize', 16, ...
    'MarkerFaceColor', 'k', ...   % filled black
    'DisplayName', 'CSI-2');

% Plot CPI interface data: dotted line, unfilled circle markers
plot(cpi_events_per_sec, cpi_power_mw, 'ko', ...
    'LineWidth', 1.5, 'MarkerSize', 16, ...
    'MarkerFaceColor', 'none', ... % unfilled
    'DisplayName', 'CPI');

% Annotate CSI-2 data points
text(csi_events_per_sec(1), csi_power_mw(1), ...
    sprintf('  (%.0f Mevents/s, %.1f mW)', csi_events_per_sec(1)/1e6, csi_power_mw(1)), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ...
    'Color', 'black', 'FontSize', 30);

text(csi_events_per_sec(2), csi_power_mw(2), ...
    sprintf('(%.0f Mevents/s, %.1f mW)  ', csi_events_per_sec(2)/1e6, csi_power_mw(2)), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
    'Color', 'black', 'FontSize', 30);

% Annotate CPI data points
text(cpi_events_per_sec(1), cpi_power_mw(1), ...
    sprintf('  (%.0f Mevents/s, %.1f mW)', cpi_events_per_sec(1)/1e6, cpi_power_mw(1)), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ...
    'Color', 'black', 'FontSize', 30);

text(cpi_events_per_sec(2)+ 0*10e6, cpi_power_mw(2) + 0, ...
    sprintf(' (%.0f Mevents/s, %.1f mW)', cpi_events_per_sec(2)/1e6, cpi_power_mw(2)), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'Color', 'black', 'FontSize', 30);

text(cpi_events_per_sec(3), cpi_power_mw(3) - 0, ...
    sprintf('  (%.0f Mevents/s, %.1f mW) ', cpi_events_per_sec(3)/1e6, cpi_power_mw(3)), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'Color', 'black', 'FontSize', 30);

% Format X-axis to show in scientific notation
ax = gca;
ax.XAxis.Exponent = 6;
ax.FontSize = 32;

% Axis labels
xlabel('Events per second', 'FontSize', 32);
ylabel('Power (mW)', 'FontSize', 32);

% Title and legend
title('Power Consumption vs Event Rate', 'FontSize', 35);
legend('Location', 'south', 'FontSize', 35);

