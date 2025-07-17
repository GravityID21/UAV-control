clear all % reset all veriables
close all % clear plots
clc       % clear terminal

% Initialization
initial_value = 100;   % Starting value
reduction_amount = 1;  % Fixed decrement per iteration
R_iterations = 100;       % Number of iterations

% Pre-allocate for efficiency
parameter = zeros(R_iterations, 1);  
parameter(1) = initial_value;
enorm_Best = zeros(R_iterations, 1);
parameter_current_R = initial_value; % Initialize as scalar

for numberit = 1:R_iterations
    if numberit > 1
        parameter(numberit) = parameter(numberit-1) - reduction_amount;
    end
    parameter_current_R = parameter(numberit);  % Update global scalar
    save('myData.mat', 'R_iterations', 'parameter_current_R', 'enorm_Best', 'parameter', "numberit", "reduction_amount" ); % resave all veriables from run
    run('Tuning_script')  % Run script with updated value
    save('myData.mat', 'R_iterations', 'parameter_current_R', 'enorm_Best', 'parameter', "numberit", "reduction_amount" ); % resave all veriables from run
end

run('Drone4into12V2')
    % Plot the results
figure;
plot(1:R_iterations, enorm_Best, 'bo-', 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Error norm');
title('Lowest Error norm per iteration');
grid on;

 % Plot the results
figure;
plot(parameter, enorm_Best, 'bo-', 'LineWidth', 2);
xlabel('R value');
ylabel('Best Error norm');
title('Lowest Error norm per Value');
grid on;