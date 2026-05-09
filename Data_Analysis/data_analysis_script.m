% =========================================================
%  SolarDuel — Tracker vs Fixed Panel Efficiency Analysis
%  Academic figure style: white background, (a)(b)(c)(d) labels
% =========================================================

clc; clear; close all;

%% ── 1. Load Data ─────────────────────────────────────────

tracker = readtable('tracker_panel_data.csv');
fixed   = readtable('fixed_panel_data.csv');

t_min_tracker = tracker.Time_s / 60;
t_min_fixed   = fixed.Time_s   / 60;

PANEL_MAX_W = 0.9;

%% ── 2. Derived Metrics ───────────────────────────────────

tracker_eff = (tracker.Power_mW / 1000) / PANEL_MAX_W * 100;
fixed_eff   = (fixed.Power_mW   / 1000) / PANEL_MAX_W * 100;

[fixed_t_unique, ~, idx] = unique(fixed.Time_s);
fixed_p_unique = accumarray(idx, fixed.Power_mW, [], @mean);
fixed_power_interp = interp1(fixed_t_unique, fixed_p_unique, ...
                             tracker.Time_s, 'linear', 'extrap');

valid = fixed_power_interp > 5;
instant_gain = NaN(height(tracker), 1);
instant_gain(valid) = ((tracker.Power_mW(valid) - fixed_power_interp(valid)) ./ ...
                        fixed_power_interp(valid)) * 100;

energy_gain = (max(tracker.Energy_Wh) - max(fixed.Energy_Wh)) / ...
               max(fixed.Energy_Wh) * 100;

%% ── 3. Summary Statistics ────────────────────────────────

fprintf('============================================================\n');
fprintf('          SolarDuel — Efficiency Analysis Summary\n');
fprintf('============================================================\n\n');
fprintf('%-30s %12s %12s\n', 'Metric', 'Tracker', 'Fixed');
fprintf('%s\n', repmat('-',1,56));
fprintf('%-30s %11.2f%% %11.2f%%\n', 'Peak Efficiency',    max(tracker_eff),      max(fixed_eff));
fprintf('%-30s %11.2f%% %11.2f%%\n', 'Average Efficiency', mean(tracker_eff),     mean(fixed_eff));
fprintf('%-30s %11.1f mW %10.1f mW\n','Peak Power',        max(tracker.Power_mW), max(fixed.Power_mW));
fprintf('%-30s %11.1f mW %10.1f mW\n','Average Power',     mean(tracker.Power_mW),mean(fixed.Power_mW));
fprintf('%-30s %11.4f Wh %10.4f Wh\n','Total Energy',      max(tracker.Energy_Wh),max(fixed.Energy_Wh));
fprintf('\n%-30s %+11.1f%%\n', 'Tracker Energy Gain',   energy_gain);
fprintf('%-30s %11.2f%%\n',   'Avg Instantaneous Gain',mean(instant_gain,'omitnan'));
fprintf('============================================================\n\n');

%% ── 4. Figure ────────────────────────────────────────────

fig = figure('Position', [100 100 1100 780], 'Color', 'white');

c_tracker = [0.85 0.33 0.10];   % burnt orange — tracker
c_fixed   = [0.00 0.45 0.70];   % steel blue   — fixed
c_gain    = [0.47 0.67 0.19];   % muted green  — gain
c_neg     = [0.80 0.20 0.20];   % red          — negative gain
c_avg     = [0.50 0.50 0.50];   % grey         — average line

label_font = 11;
axis_font  = 9;
lw_data    = 1.2;
lw_avg     = 1.0;

% ── (a) Instantaneous Power ───────────────────────────────
ax1 = subplot(2,2,1); hold on; box on;
fill([t_min_tracker; flipud(t_min_tracker)], ...
     [tracker.Power_mW; zeros(height(tracker),1)], ...
     c_tracker, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
fill([t_min_fixed; flipud(t_min_fixed)], ...
     [fixed.Power_mW; zeros(height(fixed),1)], ...
     c_fixed, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
plot(t_min_tracker, tracker.Power_mW, 'Color', c_tracker, 'LineWidth', lw_data);
plot(t_min_fixed,   fixed.Power_mW,   'Color', c_fixed,   'LineWidth', lw_data);
legend('Tracker', 'Fixed', 'Location', 'northeast', 'FontSize', axis_font);
xlabel('Time (min)'); ylabel('Power (mW)');
text(0.02, 0.96, '(a)', 'Units','normalized','FontSize',label_font, ...
     'FontWeight','bold','VerticalAlignment','top');
styleAx(ax1, axis_font);

% ── (b) Panel Efficiency ──────────────────────────────────
ax2 = subplot(2,2,2); hold on; box on;
plot(t_min_tracker, tracker_eff, 'Color', c_tracker, 'LineWidth', lw_data);
plot(t_min_fixed,   fixed_eff,   'Color', c_fixed,   'LineWidth', lw_data);
yline(100, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
legend('Tracker', 'Fixed', 'Location', 'northeast', 'FontSize', axis_font);
xlabel('Time (min)'); ylabel('Efficiency (%)');
text(0.02, 0.96, '(b)', 'Units','normalized','FontSize',label_font, ...
     'FontWeight','bold','VerticalAlignment','top');
styleAx(ax2, axis_font);

% ── (c) Cumulative Energy ─────────────────────────────────
ax3 = subplot(2,2,3); hold on; box on;
area(t_min_tracker, tracker.Energy_Wh, 'FaceColor', c_tracker, ...
     'FaceAlpha', 0.15, 'EdgeColor', c_tracker, 'LineWidth', lw_data);
area(t_min_fixed,   fixed.Energy_Wh,   'FaceColor', c_fixed, ...
     'FaceAlpha', 0.15, 'EdgeColor', c_fixed,   'LineWidth', lw_data);
legend('Tracker', 'Fixed', 'Location', 'northeast', 'FontSize', axis_font);
xlabel('Time (min)'); ylabel('Energy (Wh)');
text(0.02, 0.96, '(c)', 'Units','normalized','FontSize',label_font, ...
     'FontWeight','bold','VerticalAlignment','top');
styleAx(ax3, axis_font);

% ── (d) Tracker Gain over Fixed ───────────────────────────
ax4 = subplot(2,2,4); hold on; box on;
pos_gain = instant_gain; pos_gain(pos_gain <  0 | isnan(pos_gain)) = 0;
neg_gain = instant_gain; neg_gain(neg_gain >= 0 | isnan(neg_gain)) = 0;
area(t_min_tracker, pos_gain, 'FaceColor', c_gain, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
area(t_min_tracker, neg_gain, 'FaceColor', c_neg,  'FaceAlpha', 0.25, 'EdgeColor', 'none');
plot(t_min_tracker, instant_gain, 'Color', c_gain, 'LineWidth', lw_data);
yline(0, '-', 'Color', [0 0 0], 'LineWidth', 0.8);
avg_gain = mean(instant_gain, 'omitnan');
yline(avg_gain, '--', sprintf('Mean: %.1f%%', avg_gain), ...
      'Color', c_avg, 'LineWidth', lw_avg, ...
      'LabelHorizontalAlignment', 'left', 'FontSize', axis_font - 1);
xlabel('Time (min)'); ylabel('Gain (%)');
ylim([-50 400]);
text(0.02, 0.96, '(d)', 'Units','normalized','FontSize',label_font, ...
     'FontWeight','bold','VerticalAlignment','top');
styleAx(ax4, axis_font);

% Tight layout
set(fig, 'Units', 'normalized');
set(findall(fig,'-property','FontName'), 'FontName', 'Times New Roman');

saveas(fig, 'efficiency_analysis.png');
fprintf('Figure saved as efficiency_analysis.png\n');

%% ── Helper ───────────────────────────────────────────────
function styleAx(ax, fs)
    ax.Color      = 'white';
    ax.XColor     = 'black';
    ax.YColor     = 'black';
    ax.GridColor  = [0.85 0.85 0.85];
    ax.GridAlpha  = 1.0;
    ax.Box        = 'on';
    ax.FontSize   = fs;
    ax.FontName   = 'Times New Roman';
    grid(ax, 'on');
end