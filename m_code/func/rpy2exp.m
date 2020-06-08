function [exp] = rpy2exp(rpy)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
r=[1,0,0;0,cos(rpy(1)),-sin(rpy(1));0,sin(rpy(1)),cos(rpy(1))];
p=[cos(rpy(2)),0,sin(rpy(2));0,1,0;-sin(rpy(2)),0,cos(rpy(2))];
y=[cos(rpy(3)),-sin(rpy(3)),0;sin(rpy(3)),cos(rpy(3)),0;0,0,1];
R=y*p*r;

theta=acos((trace(R)-1 )/2); 
exp=[0.5/sin(theta)*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];theta];
end

