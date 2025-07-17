clear all % reset all veriables
close all % clear plots
clc       % clear terminal

% Initialization
initial_value = 0.1;       % Starting value
reduction_amount = 0.002;  % Fixed decrement per iteration
Q_iterations = 50;         % Number of iterations

% Pre-allocate for efficiency
parameter = zeros(Q_iterations, 1);  
parameter(1) = initial_value;
enorm_Best = zeros(Q_iterations, 1);
parameter_current = initial_value; % Initialize as scalar

for numberit = 1:Q_iterations
    if numberit > 1
        parameter(numberit) = parameter(numberit-1) - reduction_amount;
    end

    parameter_current = parameter(numberit);  % Update global scalar
    save('myData.mat', 'Q_iterations', 'parameter_current', 'enorm_Best', 'parameter', "numberit", "reduction_amount" ); % resave all veriables from run
    run('Tuning_script.m')  % Run script with updated value
    save('myData.mat', 'Q_iterations', 'parameter_current', 'enorm_Best', 'parameter', "numberit", "reduction_amount" ); % resave all veriables from run
end

%run('Drone4into12V2')
    % Plot the results
figure;
plot(1:Q_iterations, enorm_Best, 'bo-', 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Error norm');
title('Lowest Error norm per iteration');
grid on;

 % Plot the results
figure;
plot(parameter, enorm_Best, 'bo-', 'LineWidth', 2);
xlabel('Q value');
ylabel('Best Error norm');
title('Lowest Error norm per Value');
grid on;