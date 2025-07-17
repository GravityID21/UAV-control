clear;
clc;

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
     0 0 0 0 0 0 0 0 0 0 0 1;  % psi
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0];

D = zeros(12, 4);


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
N = length(t);
iterations = 400;                              % Number of ILC iterations
m = 4;                                         % Number of inputs
n = 12;                                        % Number of outputs

% Initial state and reference trajectory
x0 = zeros(size(A, 1), 1);                     % Initial state
x_d = 1 - cos(2 * t);
y_d = sin(2 * t);
z_d = 2 * t;
psi_d = zeros(size(t));
ref = [x_d', y_d', z_d', psi_d', zeros(N, 8)]; % Extend to 12 states

% Compute Markov parameters
for i = 1:N
    for j = 1:i
        G_block = C * A^(i-j) * B;        % 4x4 or 12x4 depending on C
        G((i-1)*n+1:i*n, (j-1)*m+1:j*m) = G_block;
    end
end

%% **Select waypoints for P2P-ILC**
waypoint_times = [0, 1, 2, 3, 4, 5, 6, 7];  
M = length(waypoint_times);


waypoint_indices = zeros(size(waypoint_times));
for i = 1:length(waypoint_indices)
    waypoint_indices(i) = waypoint_times(i)*100 + 1;
end

% Transformation Matrix T 12,701
T = zeros(n, N);
for i = 1:M
    for j = 1:M %
    T(i, waypoint_indices(j)) = 1; % 
    end %
end

% Extract the reference only at selected waypoints
re = T' .* ref; 

%% might need to flip this
% Transformation Matrix Tg 
Tg = zeros(m*N, N*n);
for i = 1:M
    T(i, waypoint_indices(i)) = 1;
end

% Extract the reduced Markov matrix for waypoints
Ge = Tg' .* G;

% Initialize inputs and error storage
u = zeros(m, N);
y = zeros(n, N);
enorm = zeros(iterations, 1);

% P2P ILC matrices
% ILC matrices
Q = 2 * eye(n * N);
lambda = 1;
R = eye(m * N);


%% might need to flip this
% Transformation Matrix Tq 
Tq = zeros(n*N, n*N);
for i = 1:M
    T(i, waypoint_indices(i)) = 1;
end
Qe = Tq.*Q.*Tq;


% Compute the learning matrix in reduced space
L = inv(G' * G + lambda * eye(m * N)) * G';

%Method
NONE = 1;
NOILC = 0;
Gradeint = 0;

if NONE
   %% **Iterative Learning Control (ILC)**
   for k = 1:iterations
    u_flat = reshape(u, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, n, N)'; 
    
    % Extract output only at waypoint
    ye = T' .* y;  
    ee = (re - ye);  
    ee_flat = reshape(ee', [], 1);  % Ensure column vector (size = 2804 × 1)
    delta_u_flat = inv(R + G' * Q * G) * G' * Q * ee_flat;
    
    u = u + reshape(delta_u_flat, m, N);
    enorm(k) = norm(ee, 'fro'); 
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));
end

% Gradient Descent P2P ILC parameters
beta = 0.1;  % Learning rate


elseif NOILC
   %% **Iterative Learning Control (ILC) using Norm Optimal Method**7695
    for k = 1:iterations
    u_flat = reshape(u, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, n, N)'; 
    
    ye = T' .* y;  
    ee = (re - ye);  
    ee_flat = reshape(ee', [], 1);  
    
    Delta_u = (G' * Q * G + R) \ (G' * Q * ee_flat);
    
    u = u + reshape(Delta_u, m, []);
    enorm(k) = norm(ee, 'fro'); 
    
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));
   end

elseif Gradient
   %% **Iterative Learning Control (ILC) using Gradient Descent**
   for k = 1:iterations
    u_flat = reshape(u, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, n, N)'; 
      
    ye = T * y;  
    ee = re - ye;  
    ee_flat = reshape(ee', [], 1);
    
    delta_u_flat = beta * Ge' * ee_flat;
    
    u = u + reshape(delta_u_flat, m, N);
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
    plot(t, y(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['Output ' num2str(i) ' Tracking']);
end

%% **Plot Waypoint Tracking**
figure;
waypoint_labels = {'x', 'y', 'z', '\psi'};
for i = 1:n
    subplot(4, 1, i);
    plot(waypoint_times, re(:, i), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
    plot(waypoint_times, ye(:, i), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' waypoint_labels{i}]);
    legend('Reference', 'Output');
    title(['P2P ILC Tracking - ' waypoint_labels{i}]);
end
