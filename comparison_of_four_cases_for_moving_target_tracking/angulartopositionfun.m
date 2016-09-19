function[pox,poy]=angulartopositionfun(itheta,tarx,tary)

%itheta=45;

P=[3;7];
Q=[8;17]; 
R=[5;26]; 
S=[1;22];
Tar=[tarx;tary]; % jointed positions and the target
%plot(Tar(1,1),Tar(2,1),'*'), hold on


% y=Tar(2,1)+tan(itheta*2*pi/360)*(x-Tar(1,1));
rotation=[cos(itheta*2*pi/360) -sin(itheta*2*pi/360);sin(itheta*2*pi/360) cos(itheta*2*pi/360)]*[1;0];
%syms x y;

% corsspoint with the first line
%[xc1,yc1]=solve(2*x+1-y==0,Tar(2,1)+tan(itheta*2*pi/360)*(x-Tar(1,1))-y==0);
xc1=(Tar(1,1)*tan((pi*itheta)/180) - Tar(2,1) + 1)/(tan((pi*itheta)/180) - 2);
yc1=(tan((pi*itheta)/180) - 2*Tar(2,1) + 2*Tar(1,1)*tan((pi*itheta)/180))/(tan((pi*itheta)/180) - 2);
%corsspoint with the second line
 %[xc2,yc2]=solve(-3*x+41-y==0,Tar(2,1)+tan(itheta*2*pi/360)*(x-Tar(1,1))-y==0);
 xc2=(Tar(1,1)*tan((pi*itheta)/180) - Tar(2,1) + 41)/(tan((pi*itheta)/180) + 3);
 yc2=(3*Tar(2,1) + 41*tan((pi*itheta)/180) - 3*Tar(1,1)*tan((pi*itheta)/180))/(tan((pi*itheta)/180) + 3);
 
%corsspoint with the third line
%[xc3,yc3]=solve(x+21-y==0,Tar(2,1)+tan(itheta*2*pi/360)*(x-Tar(1,1))-y==0);
xc3=(Tar(1,1)*tan((pi*itheta)/180) - Tar(2,1) + 21)/(tan((pi*itheta)/180) - 1);
yc3=(21*tan((pi*itheta)/180) - Tar(2,1) + Tar(1,1)*tan((pi*itheta)/180))/(tan((pi*itheta)/180) - 1);

%corsspoint with the 4th line
%[xc4,yc4]=solve(-7.5*x+29.5-y==0,Tar(2,1)+tan(itheta*2*pi/360)*(x-Tar(1,1))-y==0);
xc4=(2*Tar(1,1)*tan((pi*itheta)/180) - 2*Tar(2,1) + 59)/(2*tan((pi*itheta)/180) + 15);
yc4=(15*Tar(2,1) + 59*tan((pi*itheta)/180) - 15*Tar(1,1)*tan((pi*itheta)/180))/(2*tan((pi*itheta)/180) + 15);

if (xc1-Tar(1,1))*rotation(1,1)+(yc1-Tar(2,1))*rotation(2,1)>0 && xc1>=P(1,1) && xc1<=Q(1,1)
    pox=xc1;
    poy=yc1;
elseif (xc2-Tar(1,1))*rotation(1,1)+(yc2-Tar(2,1))*rotation(2,1)>0 && xc2<=Q(1,1) && xc2>=R(1,1)
    pox=xc2;
    poy=yc2;
elseif (xc3-Tar(1,1))*rotation(1,1)+(yc3-Tar(2,1))*rotation(2,1)>0 && xc3<=R(1,1) && xc3>=S(1,1)
    pox=xc3;
    poy=yc3;
elseif (xc4-Tar(1,1))*rotation(1,1)+(yc4-Tar(2,1))*rotation(2,1)>0 && xc4>=S(1,1) && xc4<=P(1,1)
    pox=xc4;
    poy=yc4;
else
    pox=0;
    poy=0;
end

%display(pox);
%display(poy);
%plot(pox, poy, '*')
%hold off
end
