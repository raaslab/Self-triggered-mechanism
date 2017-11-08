function[newtheta]=orderlythetafun(itheta)
%itheta=[-10;100;-160;-60];
newtheta=itheta;
for i = 1 : length(itheta)-1  % for all the robots
    if itheta(i)>itheta(i+1)
        newtheta(i+1)=itheta(i+1)+2*pi;
        itheta(i+1)=newtheta(i+1);
    end
end
end