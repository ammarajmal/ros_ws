clear 
close all
clc
%% reading csv files

% Loading the data
% % 2 Hz - camera 1
camera_file = 'camera_1_30s_2023-04-20_06-13-09_2Hz.csv';
ldv_file = 'protocol_optoNCDT ILD1420_2023-04-20_06-13-08.015_CAM1_2Hz_30s.csv';


% % 5 Hz - camera 1
% camera_file = 'camera_1_30s_2023-04-20_06-17-00_5Hz.csv';
% ldv_file = 'camera_1_30s_optoNCDT ILD1420_2023-04-20_06-17-03.572_5Hz.csv';

% % 5 Hz - camera 2
% camera_file = 'camera_2_30s_2023-04-20_06-18-40_5Hz.csv';
% ldv_file = 'protocol_optoNCDT ILD1420_2023-04-20_06-18-43.332_CAM2_5Hz_30s.csv';


cameraData = readtable(camera_file, 'VariableNamingRule', 'preserve');
ldvData = readtable(ldv_file, VariableNamingRule='preserve');
%%
camera_name = extractBefore(camera_file, '_30s'); % extract text before '_30s'
camera_number = extractAfter(camera_name, 'camera_'); % extract text after 'camera_' 
last_underscore = find(camera_file == '_', 1, 'last'); % find the position of the last underscore
hz_value = extractBetween(camera_file, last_underscore+1, strfind(camera_file, 'Hz')-1); % extract the value between the last underscore and 'Hz'
hz_str = num2str(str2double(hz_value));


% 5 Hz Signal - camera 1
% cameraData = readtable('camera_1_30s_2023-04-20_06-17-00_5Hz.csv', 'VariableNamingRule', 'preserve');
% ldvData = readtable('camera_2_30s_optoNCDT ILD1420_2023-04-20_06-17-03.572_5Hz - Copy.csv');



% % 5 Hz Signal - camera 2
% cameraData = readtable('camera_2_30s_2023-04-20_06-18-40_5Hz.csv', 'VariableNamingRule', 'preserve');
% ldvData = readtable('camera_2_30s_protocol_optoNCDT ILD1420_2023-04-20_06-18-43.332_5Hz.csv');






%% loading data values

% laoding the camera data
xDisplacementsCamera = cameraData.("field.transforms0.transform.translation.x");
yDisplacementsCamera = cameraData.("field.transforms0.transform.translation.y");
zDisplacementsCamera = cameraData.("field.transforms0.transform.translation.x");

fs_cam = 100;
ts_cam = 1/fs_cam;
time_cam = 0:ts_cam:ts_cam*(length(yDisplacementsCamera)-1);

yDisplacementsCamera = yDisplacementsCamera - mean(yDisplacementsCamera);


ldvDisplacements = ldvData.("Var4");

% ldvTime = ldvData.("Time");

%% adjusting values

% Align the signal around zero
ldvDisplacements = ldvDisplacements - mean(ldvDisplacements);

% ldvTimeSec = posixtime(datetime(ldvTime, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS'));
% % sampling frequency of ldv data
% fs = round(1 / (ldvTimeSec(2,1) - ldvTimeSec(1,1)));
fs = 1000;
downsample_freq = 100;
downsample_factor = fs/downsample_freq;

% downsampling the signal
ldvDisplacements_down = downsample(ldvDisplacements, downsample_factor);
% ldvTime_down = downsample(ldvTimeSec, downsample_factor);

% sampling frequency of ldv data after downsampling (confirmation)
% fs_down = round(1 / (ldvTime_down (2,1) - ldvTime_down (1,1)));
fs_down = 100;

% Calculate the duration of one cycle of the signal based on its frequency
ts = 1/fs_down;

% Generating time axis for the signal
time = 0:ts:ts*(length(ldvDisplacements_down)-1);




% Compute the DFT of the downsampled signal (ldv)
N_ldv = length(ldvDisplacements_down);
dft_ldv = fft(ldvDisplacements_down);

% Compute the frequency axis for the DFT
f_ldv = (0:N_ldv-1)*(fs_down/N_ldv) - fs_down/2;

% Shift the DFT to center it on zero frequency
dft_shifted_ldv = fftshift(dft_ldv);


% Compute the DFT of the downsampled signal (camera)
N_cam = length(yDisplacementsCamera);
dft_cam = fft(yDisplacementsCamera);

% Compute the frequency axis for the DFT
f_cam = (0:N_cam-1)*(fs_cam/N_cam) - fs_cam/2;

% Shift the DFT to center it on zero frequency
dft_shifted_cam = fftshift(dft_cam);

%% Plotting
figure;
sgtitle(strcat('Displacement Measurement - Camera', camera_number, ' (', hz_str,'Hz)'));

subplot(2,3,1);
plot(time(1:fs_down), ldvDisplacements_down(1:fs_down));
% yticks([-4 -3 -2 -1 0 1 2 3 4]);
xlabel('Time (s)');
ylabel('Displacements (mm)');
title('LDV - Time', FontWeight='normal');
grid on



subplot(2,3,2);
plot(time_cam(1:100), yDisplacementsCamera(1:100)*1000/2, color='red');
% yticks([-4 -3 -2 -1 0 1 2 3 4]);
xlabel('Time (s)');
ylabel('Displacements (mm)');
title('Camera - Time', FontWeight='normal');
grid on



% Align the signals using the default method (resampling)
[ldvDisplacements_down_aligned, yDisplacementsCamera_aligned] = alignsignals(ldvDisplacements_down(1:100), yDisplacementsCamera(1:100)*1000/2);
ldvDisplacements_down_aligned = ldvDisplacements_down_aligned(1:100);

subplot(2,3,3);
plot(time_cam(1:100), yDisplacementsCamera_aligned, LineWidth=1, Color='blue');
hold on
plot(time(1:fs_down), ldvDisplacements_down_aligned, LineWidth=1, Color='red');
hold off
% legend('ldv disp', 'cam disp')
xlabel('Time (s)');
ylabel('Displacements (mm)');
title('Comparison - Time', FontWeight='normal');
grid on

% Plot the magnitude of the DFT for positive frequencies on a semilogarithmic scale
subplot(2,3,4);

% Define the frequency range to plot
freq_range = (0:10);

% Find the indices of the frequencies to plot
idx = find(f_ldv(N_ldv/2+1:end) <= max(freq_range));

% Plot the magnitude of the DFT for the selected frequencies
% semilogy(f_ldv(N_ldv/2+1:end), abs(dft_shifted_ldv(N_ldv/2+1:end)));
% hold on;
semilogy(f_ldv(N_ldv/2+1+idx), abs(dft_shifted_ldv(N_ldv/2+1+idx)));
% hold off;
xlabel('Frequency (Hz)')
ylabel('Magnitude (log scale)')
title('LDV - Frequency', FontWeight='normal')
grid on


% Plot the magnitude of the DFT for positive frequencies on a semilogarithmic scale
subplot(2,3,5);

% Define the frequency range to plot
freq_range = (0:10);

% Find the indices of the frequencies to plot
idx = find(f_cam(fix(N_cam/2+1:end)) <= max(freq_range));

% Plot the magnitude of the DFT for the selected frequencies
% semilogy(f_ldv(N_ldv/2+1:end), abs(dft_shifted_ldv(N_ldv/2+1:end)));
% hold on;
semilogy(f_cam(fix(N_cam/2+1+idx)), abs(dft_shifted_cam(fix(N_cam/2+1+idx))), Color='red');
% hold off;
xlabel('Frequency (Hz)')
ylabel('Magnitude (log scale)')
title('Camera - Frequency', FontWeight='normal')
grid on



% Plot the magnitude of the DFT for positive frequencies on a semilogarithmic scale
subplot(2,3,6);

% Define the frequency range to plot
freq_range = (0:10);

% Find the indices of the frequencies to plot
idx = find(f_cam(fix(N_cam/2+1:end)) <= max(freq_range));

% Plot the magnitude of the DFT for the selected frequencies
semilogy(f_ldv(fix(N_ldv/2+1+idx)), abs(dft_shifted_ldv(fix(N_ldv/2+1+idx))), Color='blue', LineWidth=1);
hold on;
semilogy(f_cam(fix(N_cam/2+1+idx)), abs(dft_shifted_cam(fix(N_cam/2+1+idx))), Color='red', LineWidth=1);
hold off;
% legend('ldv disp', 'cam disp', 'Location','northeast')
xlabel('Frequency (Hz)')
ylabel('Magnitude (log scale)')
title('Comparison - Frequency', FontWeight='normal')
grid on



% Save the plot as a PNG file
saveas(gcf, strcat(camera_file, '.png'));



