function [angle_new] = regulateangular(angle_old)

if angle_old<-pi
   angle_new=2*pi+angle_old;
elseif angle_old>pi;
   angle_new=angle_old-2*pi;   
else
   angle_new=angle_old; 
end
