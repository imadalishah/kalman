clear all, close all, clc
 
%% Model Characteristics
A = [1   -0.5
    1      0];
C = [1 0];
% B and D matrices have been ignored in the model.
N  = 100;               % No of points
X  = zeros(2,N);        % State variables
y  = zeros(1,N);        % Output
Q = 0.1 * eye(2);       % Process Noise Covariance Matrix
R = 0.05;               % Observation Noise Covariance Matrix
 
%% Initial Values
X(:,1) = [1 0]';        % Initial state
y(1) = C * X(:,1) ;     % Initial observation
 
%% Simulate states and observations
for i = 2 : N
    X(:,i) = A * X(:,i-1)   % States
    y(:,i) = C * X(:,i)     % Real Observations
end
 
%% Kalman Filter Implementation
xh(:,1) = 0.01*randn(2,1);    % Initial state
Px = eye(2);                  % Initial state covariance matrix
for i = 1 : size(y,2)
    xh_(:,i) = A * xh(:,i);   % Current state estimate
    Px_ = A*Px*A' + Q;        % Current state covariance matrix
    K = Px_ * C' * inv(C*Px_*C' + R);   % Kalman filter coefficient
    yh_(:,i) = C * xh_(:,i) + R;        % Estimated observation
    inov(:,i) = y(:,i) - yh_(:,i);      % Measurement residual error
    xh(:,i+1) = xh_(:,i) + K * inov(:,i); % Estimate of the current state
    Px = Px_ - K*C*Px_;     % state covariance matrix
end
   
%% Plot the estimation results   
figure, plot(y,'b')
hold on, plot(yh_,'r')
grid on
title('Kalman Filter Implementation')
legend('Real observation','Estimated Xk')
