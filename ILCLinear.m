clear all % reset all veriables
close all  %closes all plots
clc

m  = 0.52;                 % Mass (kg)
Ix = 6.228e-3;             % Moment of inertia around x-axis (kg·m²)
Iy = 6.225e-3;             % Moment of inertia around y-axis (kg·m²)
Iz = 1.121e-2;             % Moment of inertia around z-axis (kg·m²)
g  = 9.81;                 % Acceleration due to gravity (m/s²)

E =  -(1/m);
F = (1/ Ix);
H = (1/ Iy);
L = (1/ Iz);

% Quadcopter State-Space Matrices (Linearized Dynamics) 
A = [0 0 0 0 0 0 1 0 0 0 0 0
     0 0 0 0 0 0 0 1 0 0 0 0,
     0 0 0 0 0 0 0 0 1 0 0 0
     0 0 0 0 0 0 0 0 0 1 0 0
     0 0 0 0 0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 0 0 0 0 1
     0 0 0 0 -g 0 0 0 0 0 0 0
     0 0 0 g 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0];

B = [0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0
     0 0 0 0
     E 0 0 0 
     0 F 0 0 
     0 0 H 0 
     0 0 0 L];

C = [1 0 0 0 0 0 0 0 0 0 0 0;  % x
     0 1 0 0 0 0 0 0 0 0 0 0;  % y
     0 0 1 0 0 0 0 0 0 0 0 0;  % z
     0 0 0 0 0 0 0 0 0 0 0 1];  % psi
     
D = zeros(4, 4);

% Discretize the state-space model
Ts = 0.01; % Sampling time
sys_d = c2d(ss(A, B, C, D), Ts, 'zoh'); % Zero-order hold discretization

% Extract discrete matrices
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% Define simulation parameters
T = 7;                                         % Total simulation time
t = 0:Ts:T;
t_sim = t';                                    %colum vector to work with sim
NumberofSteps = length(t);
iterations = 75;                              % Number of ILC iterations
NumberofInputs = 4;                                         % Number of inputs
NumberofOutputs = 4;                                        % Number of outputs

%Method
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
ILCoutput = zeros(NumberofOutputs, NumberofSteps);
enorm = zeros(iterations, 1);

% Initial state and reference trajectory
x0 = zeros(size(A, 1), 1);                     % Initial state
x_d = 1 - cos(2 * t);
y_d = sin(2 * t);
z_d = 2 * t;
psi_d = zeros(size(t));
ref = [x_d', y_d', z_d', psi_d']; % Extend to 12 states

% Compute Markov parameters
for i = 1:NumberofSteps
    for j = 1:i
        G_block = C * A^(i-j) * B;        % 4x4 or 12x4 depending on C
        G((i-1)*NumberofOutputs+1:i*NumberofOutputs, (j-1)*NumberofInputs+1:j*NumberofInputs) = G_block;
    end
end

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
    for j = 1:NumberofPoints %
    Transform(i, waypoint_indices(j)) = 1; % 
    end %
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

% Extract the reference only at selected waypoints
re = Transform' .* ref; 
% Extract the reduced Markov matrix for waypoints
Ge = Transfrom_G' .* G;

% ILC matrices
Q = 99999999999999 * eye(NumberofOutputs * NumberofSteps);
lambda = 1;
R = eye(NumberofInputs * NumberofSteps);

% Compute the learning matrix in reduced space
L = inv(G' * G + lambda * eye(NumberofInputs * NumberofSteps)) * G';

if NOILC
   %% **Iterative Learning Control (ILC)**
   for k = 1:iterations 
    simout = sim('linearmod');

    %%% live output version %%%
    x = simout.X;
    y = simout.Y;
    z = simout.Z;
    psi = simout.Psi;
    output = [x, y, z, psi];

    u_flat = reshape(ILCinput, [], 1);
    u_flat = reshape(output, [], 1);
    Old_ILCinput = reshape(ILCinput, NumberofInputs, NumberofSteps);
    out_flat = G * u_flat;
    out = reshape(out_flat, NumberofOutputs, NumberofSteps)'; 
    
    % Extract output only at waypoint
    ye = Transform' .* out;  
    ee = (re - ye);  
    ee_flat = reshape(ee', [], 1);  % Ensure column vector (size = 2804 × 1)
    delta_u_flat = L*inv(R + G' * Q * G) * G' * Q * ee_flat;
    
    ILCinput = Old_ILCinput + reshape(delta_u_flat, NumberofInputs, NumberofSteps);
    ILCinput = reshape(ILCinput, [], 1);
    enorm(k) = norm(ee, 'fro'); 
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));

    uxILC = ILCinput(1:4:end,1);      
    uyILC = ILCinput(2:4:end,1);    
    uzILC = ILCinput(3:4:end,1);    
    upsiILC = ILCinput(4:4:end,1);
end

% Gradient Descent P2P ILC parameters
beta = 0.1;  % Learning rate

elseif Gradient
   %% **Iterative Learning Control (ILC) using Gradient Descent**
   for k = 1:iterations
    u_flat = reshape(ILCinput, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, NumberofOutputs, NumberofSteps)'; 
    
    % Extract output only at waypoints
    ye = Transform * y;  
    ee = re - ye;  
    ee_flat = reshape(ee', [], 1);
    
    % Compute gradient update
    delta_u_flat = beta * Ge' * ee_flat;
    
    % Update input
    ILCinput = ILCinput + reshape(delta_u_flat, NumberofInputs, NumberofSteps);
    enorm(k) = norm(ee, 'fro'); 
    
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));
   end

end

%% **Plot Error Norm Convergence**
figure;
plot(1:iterations, enorm, 'b-o');
xlabel('Iteration'); ylabel('Error Norm');
title('Error Norm Convergence');

% Plot Reference vs. Output for x, y, z, phi
figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ref(:, i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, out(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['Output ' num2str(i) ' Tracking']);
end