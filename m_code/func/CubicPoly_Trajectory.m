function [p,t] = CubicPoly_Trajectory(aglo,aglf,vo,vf,to,tf,dt)  
td=tf-to;
a0=aglo;
a1=vo;
a2=( 3*(aglf-aglo)-(2*vo+vf)*td )/td^2;
a3= ( -2*(aglf-aglo)+(vo+vf)*td )/td^3;
t=to:dt:tf;
p=zeros(1,length(t));
count=0;
for tt=t
    count=count+1;
    p(count)=a0+a1*(tt-to)+a2*(tt-to)^2+a3*(tt-to)^3;
end
end

