% MATLAB Script for System Identification using Arduino Mega and Sine Wave PWM

% Clear workspace and command window
clear; clc; close all;

%% Parameters
% Serial Communication
serialPort = 'COM3'; % Change to your Arduino COM port (e.g., 'COM4' on Windows, '/dev/ttyUSB0' on Linux)
baudRate = 115200;
timeout = 10; % seconds

% Sine Wave PWM Parameters
sineFreq = 1.0;       % Hz (should match Arduino sineFreq)
sampleRate = 100.0;    % Hz (should match Arduino sampleRate)
sampleInterval = 1000.0 / sampleRate; % ms between samples
numSteps = 1000;       % Number of PWM samples (Total time = 10 seconds)

% Data Storage
inputData = zeros(numSteps, 1);     % PWM Duty Cycle (0-255)
outputData = zeros(numSteps, 1);    % Angular Speed (rev/s)
timeData = (0:numSteps-1)' * (sampleInterval / 1000); % in seconds

%% Initialize Serial Connection Using serialport
try
    s = serialport(serialPort, baudRate, "Timeout", timeout);
    configureTerminator(s, "LF"); % Set line terminator to Line Feed
    flush(s); % Clear any existing data in the buffer
    disp(['Connected to ', serialPort]);
    pause(2); % Allow Arduino to reset after connection
catch ME
    error(['Failed to connect to ', serialPort, ': ', ME.message]);
end

%% Start Sine Wave PWM on Arduino
write(s, 'S', 'char'); % Send 'S' to start sine wave PWM
disp('Sine Wave PWM start command sent. Waiting for acknowledgment...');

% Wait for acknowledgment
try
    ack = readline(s);
    if contains(ack, "SINE_PWM_START_ACK")
        disp('Received Sine PWM start acknowledgment from Arduino.');
    elseif contains(ack, "SINE_PWM_START_CONFIRMED")
        % If using additional debug messages
        disp('Received Sine PWM start confirmation from Arduino.');
    else
        warning('Unexpected acknowledgment received: %s', ack);
    end
catch ME
    error('Failed to receive Sine PWM start acknowledgment: %s', ME.message);
end

%% Data Collection Loop
disp('Collecting data...');
tic;
for i = 1:numSteps
    try
        % Read the line from serial
        dataLine = readline(s); % Read one line terminated by 'LF'
        
        % Display received data for debugging (optional)
        disp(['Received: ', dataLine]);
        
        % Split the received data by comma
        dataParts = split(dataLine, ',');
        
        if numel(dataParts) ~= 3
            warning('Received malformed data: %s', dataLine);
            continue; % Skip to next iteration
        end
        
        % Convert PWM value, encoder count, and angular speed to numbers
        pwmValue = str2double(dataParts(1));
        encoderCount = str2double(dataParts(2));
        angularSpeed = str2double(dataParts(3));
        
        if isnan(pwmValue) || isnan(encoderCount) || isnan(angularSpeed)
            warning('Received NaN values. PWM Value: %s, Encoder Count: %s, Angular Speed: %s', dataParts(1), dataParts(2), dataParts(3));
            pwmValue = 0; % Default value or handle as needed
            encoderCount = 0;
            angularSpeed = 0.0;
        end
        
        % Store the input and output data
        inputData(i) = pwmValue;       % PWM duty cycle (0-255)
        outputData(i) = angularSpeed;  % Angular speed in rev/s
        
        % Optional: Display progress every 100 samples
        if mod(i, 100) == 0
            fprintf('Collected %d/%d samples...\n', i, numSteps);
        end
    catch ME
        warning('Error reading data at step %d: %s', i, ME.message);
        continue; % Skip to next iteration
    end
end
toc;

%% Stop Sine Wave PWM on Arduino
write(s, 'P', 'char'); % Send 'P' to stop sine wave PWM
disp('Sine Wave PWM stop command sent. Waiting for acknowledgment...');

% Wait for acknowledgment
try
    ack = readline(s);
    if contains(ack, "SINE_PWM_STOP_ACK")
        disp('Received Sine PWM stop acknowledgment from Arduino.');
    elseif contains(ack, "SINE_PWM_STOP_CONFIRMED")
        % If using additional debug messages
        disp('Received Sine PWM stop confirmation from Arduino.');
    else
        warning('Unexpected acknowledgment received: %s', ack);
    end
catch ME
    warning('Failed to receive Sine PWM stop acknowledgment: %s', ME.message);
end

%% Close Serial Connection
clear s;
disp('Serial connection closed.');

%% Plot Input and Output Data
figure;
subplot(2,1,1);
stairs(timeData, inputData, 'b');
title('Sine Wave PWM Input Signal');
xlabel('Time (s)');
ylabel('PWM Duty Cycle (0-255)');
ylim([-10 270]); % Adjust y-axis limits as needed
grid on;

subplot(2,1,2);
plot(timeData, outputData, 'r');
title('Angular Speed Output Data');
xlabel('Time (s)');
ylabel('Angular Speed (rev/s)');
grid on;

%% Save Data to Workspace and Files
data.input = inputData;
data.output = outputData;
data.time = timeData;

save('system_data.mat', 'data');
disp('Data saved to system_data.mat');

% Save to CSV files using writematrix
writematrix([timeData, inputData], 'sine_pwm_input.csv');
writematrix([timeData, outputData], 'angular_speed_output.csv');
disp('Data saved to sine_pwm_input.csv and angular_speed_output.csv');

%% System Identification

% Create iddata object
inputSignal = data.input;
outputSignal = data.output;
Ts = sampleInterval / 1000; % Sample time in seconds

data_id = iddata(outputSignal, inputSignal, Ts);

% Plot the data
figure;
plot(data_id);
title('Input and Output Data for System Identification');
legend('Angular Speed Output', 'Sine PWM Input');
grid on;

% Determine Model Order Using System Identification Tools
maxOrder = 4; % Example maximum order, adjust as needed
sys_tf = tfest(data_id, maxOrder, maxOrder);

% Display the estimated transfer function
disp('Estimated Transfer Function:');
disp(sys_tf);

% Validate the Identified Model
figure;
compare(data_id, sys_tf);
title('Model Validation: Identified Model vs. Actual Output');
grid on;

% Residual Analysis for Model Validation
residuals = resid(data_id, sys_tf);
figure;
plot(residuals);
title('Residuals of the Identified Model');
grid on;

% Optional: Export Model for Further Analysis
save('identified_model.mat', 'sys_tf');
disp('Identified model saved to identified_model.mat');
