function[condob]=condobserva(tarx,tary,p1x,p1y,p2x,p2y)

%calculate the observability matrix
Obmatrix=[(tarx-p1x)/sqrt((tarx-p1x)^2+(tary-p1y)^2) (tary-p1y)/sqrt((tarx-p1x)^2+(tary-p1y)^2); ...
    (tarx-p2x)/sqrt((tarx-p2x)^2+(tary-p2y)^2) (tary-p2y)/sqrt((tarx-p2x)^2+(tary-p2y)^2)];

%calculate the condition number to measure the degree of the matrix with
%respect to singular
condob=cond(Obmatrix);
end