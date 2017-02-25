clear all;      % clears all variables in your workspace
N=6;
maxstep=2000;

initheta=round(sort(360*rand(N,1)));
cla;

perKFcenconver=periconvexKFcentrafun(N,initheta,maxstep);
perKFdisconver=periconvexKFdistrifun(N,initheta,maxstep);

selfKFcenconver=selfconvexKFcentrafun(N,initheta,maxstep);
%selfKFdisconver=selfconvexKFdistrifun(N,initheta,maxstep);

selfKFdisconver2=selfKFdistri2fun(N,initheta,maxstep);
%selfKFdiscond=selfKFdiscondfun(N,initheta,maxstep);

%figure; hold on;
plot(sum(perKFcenconver)); 

figure; hold on;
plot(sum(perKFdisconver));

figure; hold on;
plot(sum(selfKFcenconver));

%figure; hold on;
%plot(sum(selfKFdisconver));

figure; hold on;
plot(sum(selfKFdisconver2));
