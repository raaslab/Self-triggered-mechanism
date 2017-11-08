function [x_hat_tplus1, y_hat_tplus1, Sigma_hat_tplus1] = KF_ICI_gazebo (...
    x_hat_t, y_hat_t, Sigma_hat_t, Xr, Yr, ID, t)

global posesub_tar tarxtrue tarytrue tftree

x_hat_tplus1=x_hat_t;
y_hat_tplus1=y_hat_t;
X_hat_tplus1=zeros(2*length(Xr),1);
Sigma_hat_tplus1=Sigma_hat_t;

%
% Omega = 100;
% xc = 4 - cos(0);
% yc = 20 - sin(0);
% 
% x_true = cos((t-1)/Omega) + xc;
% y_true = sin((t-1)/Omega) + yc;

%subscribe target pose
posesub_tar = rossubscriber('/robot_6/p3dx/base_pose_ground_truth');

target_pose = receive(posesub_tar,3);

target_pose = target_pose.Pose.Pose;

tarxtrue=target_pose.Position.X;
tarytrue=target_pose.Position.Y;

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
 

Q = 0.2*eye(2);
% measurement covariance
R = sigma_z^2;
for i = 1 : length(ID)  %for all the cooperative robots, each has a real range-mesurement
    z = norm([Xr(ID(i))-tarxtrue; Yr(ID(i))-tarytrue]) + noise(ID(i));
   
    X_hat_t= [x_hat_t(ID(i)); y_hat_t(ID(i))]; 
    X_hat_tplus1_minus = X_hat_t;
    Sigma_hat_tplus1_minus = Sigma_hat_t(2*ID(i)-1:2*ID(i),:) + Q;
    z_hat = norm(X_hat_tplus1_minus - [Xr(ID(i)); Yr(ID(i))]); % predicted range for each one
    H = -1/z_hat * ([Xr(ID(i)); Yr(ID(i))]-X_hat_tplus1_minus)'; %observabition matrix 
   
    res = (z-z_hat)';

    % residual covariance
    S = H*Sigma_hat_tplus1_minus*H' + R;

    % Kalman Gain
    K = Sigma_hat_tplus1_minus * H' * inv(S);

    % state update
    X_hat_tplus1(2*ID(i)-1:2*ID(i),1) = X_hat_tplus1_minus + K*res;
   
    % covariance update (Joseph Form for numerical stability)
    Sigma_hat_tplus1(2*ID(i)-1:2*ID(i),:) =...
        (eye(2)-K*H)*Sigma_hat_tplus1_minus*(eye(2)-K*H)' + K*R*K';
       
end

    if length(ID)==1
        x_hat_tplus1(ID(1)) = X_hat_tplus1(2*ID(1)-1,1);
        y_hat_tplus1(ID(1)) = X_hat_tplus1(2*ID(1),1);
    else
        
        for i = 1: length(ID)-1
             [X_hat_tplus1(2*ID(i+1)-1:2*ID(i+1)),Sigma_hat_tplus1(2*ID(i+1)-1:2*ID(i+1),:),~] ...
                 = ICI(X_hat_tplus1(2*ID(i)-1:2*ID(i)),Sigma_hat_tplus1(2*ID(i)-1:2*ID(i),:),...
                                            X_hat_tplus1(2*ID(i+1)-1:2*ID(i+1)),...
                                            Sigma_hat_tplus1(2*ID(i+1)-1:2*ID(i+1),:));
        end
        
        for i = 1: length(ID)
               x_hat_tplus1(ID(i)) = X_hat_tplus1(2*ID(length(ID))-1);
               y_hat_tplus1(ID(i)) = X_hat_tplus1(2*ID(length(ID)));  
               Sigma_hat_tplus1(2*ID(i)-1:2*ID(i),:)=...
                   Sigma_hat_tplus1(2*ID(length(ID))-1:2*ID(length(ID)),:); 
        end
    end
end