% plot_levitation.m
%
% Plots one closed-loop levitation recording with two y axes:
%   left  y axis: relative position of the mass (um)
%   right y axis: applied voltage (V)
% against time (s, counted from the moment the PD controller was started).
%
% The recordings are the CSV files in the "levitation-2026-11-18" folder
% (made by firmware/tools/log_to_csv.py). In those files the position is
% stored as "height_pct" (% of travel between the two stops, as worked out
% by the firmware) and the voltage as "hv_V" (voltage commanded on the four
% high voltage outputs, 0 to 240 V).
%
% NOTE: BOTH QUANTITIES ARE RESCALED BEFORE PLOTTING (see "Settings"):
%   position -> 0 to POS_SCALE_UM  (1.12 um)
%   voltage  -> 0 to VOLT_SCALE_V  (5.2 V)
% This is a straight-line rescaling of the recorded numbers. So the plotted
% values are only real micrometers / volts if these two numbers are the
% correct conversion for your device.
%
% To use: open this file in MATLAB and press Run.

clear; close all;

% ===========================================================================
% Settings
% ===========================================================================

% which recording to plot (file names are relative to this script's folder)
here = fileparts(mfilename('fullpath'));
% data_file = fullfile(here, 'levitation-2026-11-18', 'hold_20s_sp50.csv');
data_file = fullfile(here, 'levitation-2026-11-18', 'setpoint_steps_30_to_75.csv');
%data_file = fullfile(here, 'levitation-2026-11-18', 'calibration_then_steps.csv');

% what the position and the voltage are rescaled to
POS_SCALE_UM = 1.12;   % top of the position axis (um)
VOLT_SCALE_V = 5.2;    % top of the voltage axis (V)

% how they are rescaled:
%   'minmax'    the smallest value in the plot becomes 0 and the largest
%               becomes POS_SCALE_UM (or VOLT_SCALE_V)
%   'fullscale' 0% of travel becomes 0 and 100% of travel (the other stop)
%               becomes POS_SCALE_UM; 0 V stays 0 and the firmware's voltage
%               limit HV_FULL_SCALE_V becomes VOLT_SCALE_V
SCALE_MODE = 'minmax';
HV_FULL_SCALE_V = 240; % (only used by 'fullscale')

% how much of the recording to show around the PD controller run
TIME_BEFORE_S = 1.0;   % seconds shown before the controller starts
TIME_AFTER_S = 2.0;    % seconds shown after it stops

% save the figure as a PNG file next to this script?
SAVE_PNG = true;
PNG_FILE = fullfile(here, 'levitation_plot.png');

POSITION_COLOR = [0.16 0.47 0.84];  % blue
VOLTAGE_COLOR = [0.92 0.41 0.20];   % orange

% ===========================================================================
% Read the CSV file
% ===========================================================================
% The file starts with comment lines ('#'), then one line of column names
%   t_us,c1,c2,c3,c4,hv_V,height_pct,target_pct,mode
% then the numbers. Skip everything up to and including the column names.

fid = fopen(data_file, 'r');
if fid < 0
    error('Cannot open %s', data_file);
end
text_line = fgetl(fid);
while ischar(text_line) && ~strncmp(text_line, 't_us', 4)
    text_line = fgetl(fid);
end
if ~ischar(text_line)
    fclose(fid);
    error('No line of column names (starting with t_us) found in %s', data_file);
end
raw = textscan(fid, '%f%f%f%f%f%f%f%f%f', 'Delimiter', ',', 'CollectOutput', true);
fclose(fid);
data = raw{1};

t_us = data(:, 1);        % time of the sensor reading (microseconds)
hv = data(:, 6);          % commanded voltage (V)
height = data(:, 7);      % height of the mass (% of travel)
ctrl_mode = data(:, 9);   % 0 = idle, 1 = fixed voltage, 2 = calibrating, 3 = PD controller

% the firmware prints a height of exactly -1 while it is not calibrated yet:
% leave those points out of the plot
height(height == -1 & ctrl_mode ~= 3) = NaN;

% ===========================================================================
% Pick the part to plot
% ===========================================================================

first = find(ctrl_mode == 3, 1, 'first');   % controller started
last = find(ctrl_mode == 3, 1, 'last');    % controller stopped
if isempty(first)
    error('This recording has no PD controller data (mode 3).');
end
t = (t_us - t_us(first))/1e6;          % seconds since the controller started
t_stop = t(last);
keep = t >= -TIME_BEFORE_S & t <= t_stop + TIME_AFTER_S;
t = t(keep);
hv = hv(keep);
height = height(keep);

% ===========================================================================
% Rescale
% ===========================================================================

switch SCALE_MODE
    case 'minmax'
        h_min = min(height(~isnan(height)));
        h_max = max(height(~isnan(height)));
        position = 0.5*(height - h_min)/(h_max - h_min)*POS_SCALE_UM;
        voltage = (hv - min(hv))/(max(hv) - min(hv))*VOLT_SCALE_V;
    case 'fullscale'
        position = height/100*POS_SCALE_UM;
        voltage = hv/HV_FULL_SCALE_V*VOLT_SCALE_V;
    otherwise
        error('SCALE_MODE must be ''minmax'' or ''fullscale''.');
end

% ===========================================================================
% Plot
% ===========================================================================

fig = figure('Color', 'w', 'Position', [100 100 900 480]);

yyaxis left
plot(t, position, '-', 'Color', POSITION_COLOR, 'LineWidth', 1.5);
ylim([0 POS_SCALE_UM]);
ylabel('Position (\mum)');
set(gca, 'YColor', POSITION_COLOR);

yyaxis right
plot(t, voltage, '-', 'Color', VOLTAGE_COLOR, 'LineWidth', 1.0);
ylim([0 VOLT_SCALE_V+1]);
ylabel('Applied voltage (V)');
set(gca, 'YColor', VOLTAGE_COLOR);

xlim([-TIME_BEFORE_S, t_stop + TIME_AFTER_S]);
xlabel('Time (s)');
% title('Closed-loop levitation');
legend({'Position', 'Applied voltage'}, 'Location', 'northeast');
grid on;
box on;
set(gca, 'FontSize', 18, 'FontName','Times New Roman');

if SAVE_PNG
    print(fig, PNG_FILE, '-dpng', '-r300');
    fprintf('Saved %s\n', PNG_FILE);
end
