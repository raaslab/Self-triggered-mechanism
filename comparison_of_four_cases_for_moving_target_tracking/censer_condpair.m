function[pair_x,pair_y,pair_id]=censer_condpair(N,pix,piy,px,py,tarx,tary)

pair_x=px(1,1);
pair_y=py(1,1);
pair_id=1;

for i=2:N
     if condobserva(tarx,tary,pix,piy,px(i,1),py(i,1))<condobserva(tarx,tary,pix,piy,pair_x,pair_y)
         pair_x=px(i,1);
         pair_y=py(i,1);
         pair_id=i;
     else         
     end

end