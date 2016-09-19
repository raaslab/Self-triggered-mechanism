%function[angular]=positiontoangularfun(y,x,baseangular,pasty,pastx)
function[angular]=positiontoangularfun(y,x)
%angular=baseangular+(atan2d(y,x)-atan2d(pasty,pastx));

angular=atan2d(y,x);
% if(atan2(y,x)<0)
%     angular=atan2d(y,x)+360;
% else
%     angular=atan2d(y,x);
% end
end