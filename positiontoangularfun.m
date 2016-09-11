function[angular]=positiontoangularfun(y,x)
if(atan2(y,x)<0)
    angular=atan2d(y,x)+360;
else
    angular=atan2d(y,x);
end
end