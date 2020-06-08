function [c] = sphere3itrsect(centers,radius)
%centers:3x3;[x1,y1,z1;x2,y2,z2;x3,y3,z3]
%radius:1x3:[r1;r2;r3]
c1=centers(1,:);c2=centers(2,:);c3=centers(3,:);
r1=radius(1);r2=radius(2);r3=radius(3);
u=(c2-c1)/norm(c2-c1);v=(c3-c1)/norm(c3-c1);n=cross(u,v);n=n/norm(n);
a=(norm(u)^2-r2^2+r1^2)/(2*norm(c2-c1));
b=(norm(v)^2-r3^2+r1^2)/(2*norm(c3-c1));
t1=cross(u,n);t2=cross(v,n);
x=(t2(2)*b*v(1)+(a*u(2)-b*v(2))*t2(1)-t2(2)*a*u(1))/(t2(2)*t1(1)-t1(2)*t2(1));
cl=sqrt((norm(c2-c1)-a)^2+ x^2);
h=sqrt(r2^2-cl^2);
c=(c1+a*u+x*t1+h*n)';
end

