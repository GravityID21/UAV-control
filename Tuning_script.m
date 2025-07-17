clear all % reset all veriables
close all  %closes all plots
clc

% Load parameters from the params.m file
run('CrazyflieSimulationParameters.m');

% Define simulation parameters
T = 7;                         % Total simulation time
Ts = 1/100;                    % Time step size
t = (0:Ts:T)';                 % column vector to work with sim
NumberofSteps = length(t);     % NUmber of time steps
iterations = 100;              % Number of ILC iterations
NumberofInputs = 4;            % Number of inputs
NumberofOutputs = 4;           % Number of outputs

% Initial state and reference trajectory
X_d = 1 - cos(2 * t);
Y_d = sin(2 * t);
Z_d = 2 * t;
Psi_d = zeros(size(t));
ref = [X_d', Y_d', Z_d',Psi_d']; % Extend to 4 states
ref = reshape(ref, NumberofSteps, 4);

%Method
tune_Q = 1;
p2p = 0;
NOILC = 1;
Gradeint = 0;

% Initialize inputs and error storage
ILCinput = zeros(NumberofSteps*4, 1);
% defining which part of ILC is what
uxILC = ILCinput(1:4:end,1);
uyILC = ILCinput(2:4:end,1);
uzILC = ILCinput(3:4:end,1);
upsiILC = ILCinput(4:4:end,1);
enormPrev = 400;  % this needs to be greater then first error and diff greater then tol
enorm = zeros(iterations, 1);

if (p2p)
%% **Select waypoints for P2P-ILC**
waypoint_times = [0, 2, 4, 6];  
NumberofPoints = length(waypoint_times);

waypoint_indices = zeros(size(waypoint_times));
for i = 1:length(waypoint_indices)
    waypoint_indices(i) = waypoint_times(i)*100 + 1;
end

% Transformation Matrix T 4,701
Transform = zeros(NumberofOutputs, NumberofSteps);
for i = 1:NumberofPoints
    for j = 1:NumberofPoints 
    Transform(i, waypoint_indices(j)) = 1;  
    end 
end

%% might need to flip this
% Transformation Matrix Tg 
Transfrom_G = zeros(NumberofInputs*NumberofSteps, NumberofSteps*NumberofOutputs);
for i = 1:NumberofPoints
    Transform(i, waypoint_indices(i)) = 1;
end

else
Transform = ones(4, NumberofSteps);
Transfrom_G = ones(NumberofSteps*4, 4*NumberofSteps);
end

%%%%%%%%%%%%%% ILC loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% **Iterative Learning Control (ILC)**
for it = 1:iterations 
    
    mdl = 'CrazyflieSimulationPID_20ablock2';
    load_system('CrazyflieSimulationPID_20ablock2');
    
blockpath = 'CrazyflieSimulationPID_20ablock2/Subsystem';
linsys = linearize(mdl,blockpath);

[A,B,C,D] = ssdata(linsys);  % Directly extract state-space matrices

sys_d = c2d(ss(A, B, C, D), Ts, 'zoh'); % Zero-order hold discretization

% Extract discrete matrices
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

%%%%%%%%%%% Compute Markov parameters %%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:NumberofSteps
    for j = 1:i
        G_block = C * A^(i-j) * B;        
        G((i-1)*NumberofOutputs+1:i*NumberofOutputs, (j-1)*NumberofInputs+1:j*NumberofInputs) = G_block;
    end
end

% ILC matrices

load('myData.mat');  % Load all saved variables if tuning
%numberit = 1;
%parameter_current = 0.031; % User defined if not tuning 
Q = parameter_current * eye(NumberofOutputs * NumberofSteps);
lambda = 1;
parameter_current_R = 1; % User defined if not tuning
R = parameter_current_R* eye(NumberofInputs * NumberofSteps);

% Extract the reference only at selected waypoints
ref = Transform' .* ref; 
% Extract the reduced Markov matrix for waypoints
G = Transfrom_G' .* G;

%%%%%%%%% Run Simulink Simulation Properly %%%%%%%%%%%%%%%%%%%%%%
    simout = sim('CrazyflieSimulationPID_20ablock2'); 

% Compute the learning matrix in reduced space
L = inv(G' * G + lambda * eye(NumberofInputs * NumberofSteps)) * G'; 

%%% not live version %%%%%
u_flat = ILCinput;   %%%%%
                              
%%% live output version %%%
x = simout.x;
y = simout.y;
z = simout.z;
psi = simout.psi;
output = [x, y, z, psi];

%%%%%%%%%%% ILC rerange %%%%%%%%%%%%%%%%
    u_flat = reshape(output, [], 1); %comment out if non-live test
    out_flat = G * u_flat;
    out = reshape(out_flat, [], NumberofOutputs);    %out = [xOut, yOut, zOut, psiOut];
    
    e = ref - out;
    e_flat = reshape(e', [], 1);       %e_flat = [Xe, Ye, Ze, PSIe];

    if NOILC

    delta_u_flat = inv(R + G' * Q * G) * G' * Q * e_flat;
    ILCinput = ILCinput + reshape(delta_u_flat, [], 1);

    %%%%%%%%%%% Gradient Version %%%%%%%%%%%%%%
    elseif Gradient
%% **Iterative Learning Control (ILC) using Gradient Descent**
    % Gradient Descent P2P ILC parameters
    % Compute optimal γ for fastest convergence
    gamma = 1 / max(svd(G*G'));  % γ = 1 / |G^2|
   
    % Compute gradient update
    delta_u_flat = gamma * G' * e_flat;
    
    % Update input
    ILCinput = ILCinput + reshape(delta_u_flat, [], 1);

    elseif Kalman
 %% **Iterative Learning Control (ILC) using Kalman Descent**

    % Compute Kalman Gain
    K_k = P * G' / (G * P * G' + R);  

    % Update control input using Kalman Learning
    delta_u_flat = K_k * G' * e_flat;
    ILCinput = ILCinput + reshape(delta_u_flat, [], 1);  

    % Update error covariance estimate
    P = (eye(N*4) - K_k * G) * P + Q;

   end
    uxILC = ILCinput(1:4:end,1);      
    uyILC = ILCinput(2:4:end,1);    
    uzILC = ILCinput(3:4:end,1);    
    upsiILC = ILCinput(4:4:end,1);

    enorm(it) = norm(e, 'fro');
    fprintf('Iteration %d: Error Norm = %.4f\n', it, enorm(it));

%%%%%%%%%%%%%%%%%%%%%%% Auto Check convergence %%%%%%%%%%%%%%%%%%
    threshold = 0.00000003;  % Define a small threshold
    if abs(enorm(it) - enormPrev) < threshold || enorm(it) - enormPrev > 0
    %if abs(enorm(it) - enormPrev) < threshold
        disp(['Stopping ILC at iteration ', num2str(it), ' (Converged)']);
        break;  % Stops script execution        
    end
    enormPrev = enorm(it); % move old error for converg check
    outPrev = out;         % move old
    outputPrev = output;   % move old
end

%Retrive best values
out = outPrev;         
output = outputPrev;  
% Replace last value of enorm with previously stored value
enorm(it) = enormPrev; 
% Store best value in enorm_Best at the final iteration
enorm_Best(numberit) = enormPrev;

figure;
plot(1:it, enorm(1:it), 'b-o'); % Plot up to the current iteration
xlabel('Iteration');
ylabel('Error Norm');
title('Convergence of Error Norm');

 % Plot Reference vs. ILC Input for x, y, z, phi
 figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ref(:, i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, out(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['ILC input' num2str(i) ' Tracking']);
end

 % Plot Reference vs. Output for x, y, z, phi
 figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ref(:, i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, output(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['System Ouput' num2str(i) ' Tracking']);
end