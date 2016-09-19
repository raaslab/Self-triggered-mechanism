clear all; clc;

figure(1); clf; 
axis equal; box on; hold on;
xlim([-15 25]);
ylim([5 35]);

% Initial position and covariance estimate
x_hat = 3.5;
y_hat = 19.5;
Sigma_hat = 20*eye(2);

% Randomly generate robot positions around target
num_robots = 2;
Xr = 10*(rand(1,num_robots)-0.5)+4;
Yr = 10*(rand(1,num_robots)-0.5)+20;

h(1) = covarianceEllipse([x_hat;y_hat],Sigma_hat,[1 0 0],11.82);
h(2) = plot(x_hat,y_hat,'rs','MarkerSize',8);
plot(Xr,Yr,'gs','MarkerSize',8);

for t = 0 : 300
    [x_hat, y_hat, Sigma_hat, x_true, y_true] = KF (x_hat, y_hat, Sigma_hat, Xr, Yr, t);
    
    h(1) = covarianceEllipse([x_hat;y_hat],Sigma_hat,[1 0 0],11.82);
    h(2) = plot(x_hat,y_hat,'rs','MarkerSize',8);
    h(3) = plot(x_true,y_true,'bp','MarkerSize',8);
    plot(Xr,Yr,'gs','MarkerSize',8);
    legend('Target Covariance','Target Estimate Mean','True Target','Robot');
    pause(0.1);
    cla;
end