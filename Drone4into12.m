clear;
clc;

m  = 1.82245;             % Mass (kg) of Quadcoptor
Ix = 0.00749;             % Moment of inertia around x-axis (kg·m²)
Iy = 0.00976;             % Moment of inertia around y-axis (kg·m²)
Iz = 0.05130;             % Moment of inertia around z-axis (kg·m²)
g  = 9.81;                % Acceleration due to gravity (m/s²)

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
%disp(A)
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
%disp(B)
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
%disp(C)
D = zeros(12, 4);
%disp(D)

% Discretize the state-space model
Ts = 0.01; % Sampling time
sys_d = c2d(ss(A, B, C, D), Ts, 'zoh'); 

% Extract discrete matrices
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% Define simulation parameters
T = 7;                                         % Total simulation time
t = 0:Ts:T;
N = length(t);
iterations = 50;                               % Number of ILC iterations
m = 4;                                         % Number of inputs
n = 12;                                        % Number of outputs

% Initial state and reference trajectory
%x0 = zeros(size(A, 1), 1);                     % Initial state
x_d = 1 - cos(2 * t);
y_d = sin(2 * t);
z_d = 2 * t;
psi_d = zeros(size(t));
ref = [x_d', y_d', z_d', psi_d', zeros(N, 8)]; % Extend to 12 states
%disp(size(ref))

% Compute Markov parameters
for i = 1:N
    for j = 1:i
        G_block = C * A^(i-j) * B;        % 4x4 or 12x4 depending on C
        G((i-1)*n+1:i*n, (j-1)*m+1:j*m) = G_block;
    end
end

%disp(size(G))
% Initialize inputs and error storage
u = zeros(m, N);
y = zeros(n, N);
enorm = zeros(iterations, 1);

% ILC matrices
Q = 0.8 * eye(n * N);
lambda = 1e-4;
R = eye(m * N);
L = inv(G' * G + lambda * eye(m * N)) * G';

% Define weighting matrices Q and R (diagonal and positive definite)
q = 1;                 % scalar weight for Q
r_weight = 0.1;        % scalar weight for R
Qn = diag(q * ones(N*n,1));
Rn = diag(r_weight * ones(N*n,1));

 % Choose a gradient descent step size
    alpha = 0.001;  % Adjust this value based on convergence
% Additional parameters for Gradient Descent ILC
    omega1 = 500;    % Tuning parameter > 0
    omega2 = 500;   % Tuning parameter > 0

% Define method flags (only one should be nonzero)
NONE     = 0;    % NOILC dervied in report 
NOILC    = 0;    % diffrernt Norm Optimal ILC update based %%%unused%%%%
Gradient = 0;    % Gradient Descent ILC update %%%%%unsed%%%%

if NONE
    % Conventional ILC Update
    disp('Conventional ILC.');
 for k = 1:iterations
    u_flat = reshape(u, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, n, N)';
    e = ref - y;
    e_flat = reshape(e', [], 1);
    delta_u_flat = inv(R + G' * Q * G) * G' * Q * e_flat;
    u = u + reshape(delta_u_flat, m, N);
    enorm(k) = norm(e, 'fro');
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));
    end

elseif NOILC
   %% **Iterative Learning Control (ILC) using Norm Optimal Method**7695
    disp('NO ILC.');
    for k = 1:iterations
    u_flat = reshape(u, [], 1);
    y_flat = G * u_flat;
    y = reshape(y_flat, n, N)';
    e = ref - y;
    e_flat = reshape(e', [], 1);
    
   % adjoint operator: G* = R^{-1} * G' * Q
    G_star = Rn \ (G' * Qn);
    
    % term (I + G * G*). I is chosen based on the dimension of the output.
    I = eye(size(G,1));
    causalTerm = I + G * G_star;
    
    % Solve for (I + G*G*)^{-1} e_k
    causal_factor = causalTerm \ e; 
    Delta_u = G_star * causal_factor;

    % Causal feedforward update law:
    % u_{k+1} = u_k + G* (I + G G*)^{-1} e_
    u = u + reshape(Delta_u, m, []);
    enorm(k) = norm(e, 'fro'); 
    
    fprintf('Iteration %d: Error Norm = %.4f\n', k, enorm(k));
   end

elseif Gradient
    % Gradient Descent ILC Update using Parameter Optimal Learning Gain
    for k = 1:iterations
        u_flat = reshape(u, [], 1);
        y_flat = G * u_flat;
        y = reshape(y_flat, n, N)';
        e = ref - y;
        e_flat = reshape(e', [], 1);
        
        % optimal learning gain beta according to:
        % beta = ||G^T e||^2 / (||G G^T e||^2 + omega1 + omega2 ||e||^2)
        beta_opt = (norm(G' * e_flat)^2) / (norm(G * G' * e_flat)^2 + omega1 + omega2 * (norm(e)^2));
        
        % Update control input using gradient descent:
        % u_{k+1} = u_k + beta_opt * G^T * e_k
        u_flat = u_flat + beta_opt * (G' * e_flat);
        
        u = reshape(u_flat, size(u));
        
        enorm(k) = norm(e, 2);
        fprintf('Iteration %d: Error Norm = %.4f, Beta = %.4f\n', k, enorm(k), beta_opt);
    end
  end

 disp(size(e))
 % Plot Error Norm Convergence
 figure;
 plot(1:iterations, enorm, 'b-o'); xlabel('Iteration'); ylabel('Error Norm');
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
