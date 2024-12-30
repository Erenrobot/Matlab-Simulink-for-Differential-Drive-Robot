% MATLAB PID Tuning Script with Correct Scaling

% Clear workspace and command window
clear; clc; close all;

%% 1. Load Identified Transfer Function
try
    load('identified_model.mat'); % Loads 'sys_tf'
    disp('Identified Transfer Function Loaded Successfully:');
    disp(sys_tf);
catch ME
    error('Failed to load identified_model.mat. Ensure the file exists and contains the variable "sys_tf".\nError: %s', ME.message);
end

%% 2. [Removed] Convert Encoder Counts to Revolutions
% Explanation:
% The angular speed is already in revolutions per second (rev/s).
% Therefore, scaling the transfer function by 1/counts_per_rev is unnecessary and incorrect.

%% 3. Inspect Transfer Function Poles
poles = pole(sys_tf);
disp('Poles of sys_tf:');
disp(poles);

% Calculate time constants
tau = 1 ./ abs(poles);
disp('Time Constants (tau = 1/|Re(pole)|):');
disp(tau);

% Expected tau based on desired settling time < 0.1 seconds
% For ts ≈ 4*tau, tau should be < 0.025 seconds

if any(tau > 0.025)
    warning('Some time constants exceed the desired tau (< 0.025 s). Consider reviewing the transfer function scaling or model order.');
end

%% 4. Launch PID Tuner and Export Controller
disp('Opening PID Tuner GUI...');
pidTuner(sys_tf, 'PID');

% Instructions:
% - Tune the PID controller in the GUI to achieve the desired step response.
% - Once satisfied, click "Export" and name the controller 'pid_controller'.

% Pause the script until the user exports the controller
disp('Please tune the PID controller in the PID Tuner GUI.');
disp('Once you have exported the controller as "pid_controller", press Enter to continue.');
pause;

%% 5. Validate 'pid_controller' Exists
if ~exist('pid_controller', 'var')
    error('PID controller not found. Ensure you have exported the PID controller from the PID Tuner GUI as "pid_controller".');
end

%% 6. Simulate Closed-Loop Step Response
sys_cl = feedback(pid_controller * sys_tf, 1);
disp('Closed-Loop Transfer Function:');
disp(sys_cl);

% Define a realistic simulation time based on desired settling time (<0.1 s)
t = 0:0.001:0.5; % 0.5 seconds with 1 ms steps

% Plot step response
figure;
step(sys_cl, t);
title('Closed-Loop Step Response with Tuned PID Controller');
xlabel('Time (seconds)');
ylabel('Position (revolutions)');
grid on;

% Compute and display step response characteristics
step_info = stepinfo(sys_cl, 'SettlingTimeThreshold', 0.02); % 2% threshold
disp('Step Response Characteristics:');
disp(step_info);

% Check if settling time meets the requirement
if step_info.SettlingTime <= 0.1
    disp('Settling time meets the desired specification (< 0.1 seconds).');
else
    warning('Settling time exceeds the desired specification. Consider further tuning.');
end

%% 7. Save PID Controller Parameters
save('pid_controller.mat', 'pid_controller');
disp('PID controller saved to pid_controller.mat');

% Extract and display PID gains
Kp = pid_controller.Kp;
Ki = pid_controller.Ki;
Kd = pid_controller.Kd;
N = pid_controller.N;   % Derivative filter coefficient
Tf = pid_controller.Tf; % Time constant for derivative filter

fprintf('\nPID Controller Gains:\n');
fprintf('Kp: %.4f\n', Kp);
fprintf('Ki: %.4f\n', Ki);
fprintf('Kd: %.4f\n', Kd);
fprintf('N (Derivative Filter Coefficient): %.4f\n', N);
fprintf('Tf (Derivative Filter Time Constant): %.4f\n', Tf);

%% 8. Export PID Parameters for Arduino Implementation (Optional)
fprintf('\nPID Controller Parameters for Arduino Implementation:\n');
fprintf('double Kp = %.4f;\n', Kp);
fprintf('double Ki = %.4f;\n', Ki);
fprintf('double Kd = %.4f;\n', Kd);
fprintf('// N and Tf can be used if implementing a filtered derivative term\n');
fprintf('double N = %.4f;\n', N);
fprintf('double Tf = %.4f;\n', Tf);
