function [rpy] = exp2rpy(expcood)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
wh=[0,-expcood(3),expcood(2);expcood(3),0,-expcood(1);-expcood(2),expcood(1),0];
R=eye(3,3)+ wh*sin(expcood(4))+ wh^2 *(1-cos(expcood(4)));
pitch=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
if abs(pitch-pi/2)>1e-4
    yaw=atan2(R(2,1)/cos(pitch),R(1,1)/cos(pitch));
    roll=atan2(R(3,2)/cos(pitch),R(3,3)/cos(pitch));
else %singularity case
    delta=asin(R(1,2));
    yaw=0;
    roll=delta;
end
rpy=[roll,pitch,yaw];
end

