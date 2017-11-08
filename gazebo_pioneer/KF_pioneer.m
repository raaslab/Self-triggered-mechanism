function [x_hat_tplus1, y_hat_tplus1, Sigma_hat_tplus1] = KF_pioneer (...
    x_hat_t, y_hat_t, Sigma_hat_t, Xr, Yr, ID, t)

global posesub_tar tarxtrue tarytrue tftree

% ID is a vector of the same length as Xr and Yr
% and gives the identity of the robots in Xr and Yr
% e.g., ID can be [1; 2; 3] when Xr and Yr denote
% the position of the first, second and third robots.
% ID will be [1; 2; 3; 4; 5] when Xr and Yr denotes
% the position of all five robots, etc.


% Assume object is moving on a circle
% Its true position at any time t is given by:
% [cos(t/Omega) + xc; sin(t/Omega) + yc]
% where Omega = angular rate, user-defined parameter
% and [xc,yc] = center of the circle, user-defined parameter

% Omega = 100;
% xc = 4 - cos(0);
% yc = 20 - sin(0);
% 
% x_true = cos((t-1)/Omega) + xc;
% y_true = sin((t-1)/Omega) + yc;

%%%subscribe target pose
posesub_tar = rossubscriber('/robot_6/p3dx/base_pose_ground_truth');

% % waitForTransform(tftree, '/global_origin', '/robot_6/base_link');
% robot6ToGlobal = getTransform(tftree, '/global_origin', '/robot_6/base_link'); 
% targetTranslation=robot6ToGlobal.Transform.Translation;
% %robot6Rotation=robot6ToGlobal.Rotation;

target_pose = receive(posesub_tar,3);

target_pose = target_pose.Pose.Pose;

tarxtrue=target_pose.Position.X;
tarytrue=target_pose.Position.Y;


% tf
% tarxtrue=targetTranslation.X;
% tarytrue=targetTranslation.Y;


% Simulate measurements for each robot
% We assume distance measurements
% z = true distance between robot and target + noise
% noise is zero-mean Gaussian with variance sigma_z^2
% where sigma_z is a user-defined parameter 

sigma_z = 0.2;  % std in noise

% Generate noise first
% To ensure consistency, we use time t as the seed
% so all the noise values at a given time instance are the same
rng(t);
noise = sigma_z*randn(1000,1);    % assuming no more than 1000 robots


for i = 1 : length(Xr)  % for all the robots
    Z(i) = norm([Xr(i)-tarxtrue; Yr(i)-tarytrue]) + noise(ID(i));
    %plotCircle([Xr(i); Yr(i)],Z(i),0,[0 1 0]);
end


% Apply Kalman Filter propagate
% TODO keep track of velocity
% Here we apply propagate step assuming the robot remains stationary
% User-defined parameters:
%   Q: 2x2 state noise covariance

Q = 0.2*eye(2);

X_hat_t = [x_hat_t; y_hat_t];   % make matrices

X_hat_tplus1_minus = X_hat_t;
Sigma_hat_tplus1_minus = Sigma_hat_t + Q;

%h = [h; covarianceEllipse(X_hat_tplus1_minus,Sigma_hat_tplus1_minus,[0 0 1]),11.82];

% Apply Kalman Filter update
% Linearized measurement model
% z_hat = distance between X_hat_tplus1_minus and Xr
% Z = [z_hat*(X_hat - Xr); d_hat*(Y_hat - Yr)]

for i = 1 : length(Xr)
    z_hat(i) = norm(X_hat_tplus1_minus - [Xr(i); Yr(i)]);
    H(i,:) = -1/z_hat(i) * ([Xr(i); Yr(i)]-X_hat_tplus1_minus)';
end

% residual
res = (Z-z_hat)';

% measurement covariance
R = sigma_z^2*eye(length(Xr));

% residual covariance
S = H*Sigma_hat_tplus1_minus*H' + R;

% Kalman Gain
K = Sigma_hat_tplus1_minus * H' * inv(S);

% state update
X_hat_tplus1 = X_hat_tplus1_minus + K*res;

% covariance update (Joseph Form for numerical stability)
Sigma_hat_tplus1 = (eye(2)-K*H)*Sigma_hat_tplus1_minus*(eye(2)-K*H)' + K*R*K';

x_hat_tplus1 = X_hat_tplus1(1);
y_hat_tplus1 = X_hat_tplus1(2);


end
