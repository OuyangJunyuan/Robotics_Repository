function [omegatheta] = zyz2exp(zyz)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
z1=[cos(zyz(1)),-sin(zyz(1)),0;sin(zyz(1)),cos(zyz(1)),0;0,0,1];
y=[cos(zyz(2)),0,sin(zyz(2));0,1,0;-sin(zyz(2)),0,cos(zyz(2))];
z2=[cos(zyz(3)),-sin(zyz(3)),0;sin(zyz(3)),cos(zyz(3)),0;0,0,1];
R=z1*y*z2;


theta=acos((trace(R)-1 )/2); 
omegatheta=[0.5/sin(theta)*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];theta];
end

