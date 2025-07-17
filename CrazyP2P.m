clear all % reset all veriables
close all  %closes all plots
clc

% Load parameters from the params.m file
run('CrazyflieSimulationParameters.m');

% Define simulation parameters
T = 7;            % Total simulation time
Ts = 1/100;       % Time step size
t = (0:Ts:T)';    % Column vector to work with sim
N = length(t);    % Number of time steps
%n = 4;           % Number of inputs:   leave this commented out here
%m = 4;           % Number of outputs:  leave this commented out here


K = 2;            % number of ILC iterations 
%initial
enormPrev = 400;  % This needs to be greater then first error for auto convergance check
ILCinput = zeros(4*N,1);
%Defining which part of ILC column is each input
uxILC = ILCinput(1:4:end,1);
uyILC = ILCinput(2:4:end,1);
uzILC = ILCinput(3:4:end,1);
upsiILC = ILCinput(4:4:end,1);

%Method
NOILC = 1;
Gradient = 0;
Kalman = 0;

    % Initialize Kalman filter covariance
    P = eye(N*4);  % Initial estimate covariance matrix

if(0) %enable p2p tracking by makeing this 1
    %% **Select waypoints for P2P-ILC**
    waypoint_times = [0, 1, 2, 3, 4, 5, 6, 7];  
    M = length(waypoint_times);

    waypoint_indices = zeros(size(waypoint_times));
    for i = 1:length(waypoint_indices)
        waypoint_indices(i) = waypoint_times(i)*100 + 1;
    end

    % Transformation Matrix Trans for refernce transfrom
    Trans = zeros(4, N);
    for i = 1:M
        for j = 1:M 
        Trans(i, waypoint_indices(j)) = 1; 
        end 
    end

    % Transformation Matrix Tg for G matrix transfrom
    Tg = zeros(4*N, N*4);
    for i = 1:M
        for j = 1:M 
        Tg(i, waypoint_indices(j)) = 1;  
        end 
    end

    Trans = Trans(1:4, :);
else
    Trans = ones(4, N);
    Tg = ones(N*4, 4*N);
end
%%%%%%%%%%%%%% ILC loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for it = 1:k
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

% Load parameters from the params.m file
%run('CrazyStateModel.m');
%A = Ad;
%B = Bd;

n = 4; % input number
m = 4; % output number

% Initial state and reference trajectory
X_d = 1 - cos(2 * t);
Y_d = sin(2 * t);
Z_d = 2 * t;
Psi_d = zeros(size(t));
ref = [X_d', Y_d', Z_d',Psi_d' ]; 
ref = reshape(ref, N, 4);

% Extract the reference only at selected waypoints
ref = Trans' .* ref; 

%%%%%%%%%%% Compute Markov parameters %%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:N
    for j = 1:i
        G_block = C * A^(i-j) * B;        % 4x4 or 12x4 depending on C
        G((i-1)*n+1:i*n, (j-1)*m+1:j*m) = G_block;
    end
end

% Extract the reference only at selected waypoints
ref = Trans' .* ref;
% Extract the reduced Markov matrix for waypoints
G = Tg' .* G;

%%%%%%%%% Run Simulink Simulation Properly %%%%%%%%%%%%%%%%%%%%%%
    simout = sim('CrazyflieSimulationPID_20ablock2'); 

% ILC matrices
Q = 0.44 * eye(n * N);
%Q = 2;
lambda = 1e-4;
R = 1*eye(m * N);
%L = inv(G' * G + lambda * eye(m * N)) * G'; 

%%% not live version %%%%%
u_flat = ILCinput;    
                              
%%% live outputs of system %%%
x = simout.x;
y = simout.y;
z = simout.z;
psi = simout.psi;
output = [x, y, z, psi];

%u_flat = reshape(output, [], 1); % uncomment to use outputs in update calculation
%%%%%%%%%%% ILC rerange %%%%%%%%%%%%%%%%
    out_flat = G * u_flat;
    out = reshape(out_flat, [], 4);    %out = [xOut, yOut, zOut, psiOut]; 
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
    threshold = 0.0003;  % Define a small threshold
    
    %if abs(enorm(it) - enormPrev) < threshold || enorm(it) - enormPrev > 0
    if abs(enorm(it) - enormPrev) < threshold
        disp(['Stopping ILC at iteration ', num2str(it), ' (Converged)']);
        break;  % Stops script execution        
    end
    enormPrev = enorm(it); % move old error for converg check
end

% Plot Error Norm Convergence
 figure;
 plot(1:it, enorm, 'b-o'); xlabel('Iteration'); ylabel('Error Norm');
 title('Convergence of Error Norm ');

 % Plot Reference vs. ILC Input for x, y, z, phi
 figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ref(:, i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, out(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['ILC Input ' num2str(i) ' Tracking']);
end

 % Plot Reference vs. Output for x, y, z, phi
 figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ref(:, i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t, output(:, i), 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(['Output ' num2str(i)]);
    legend('Reference', 'Output');
    title(['System Output ' num2str(i) ' Tracking']);
end
