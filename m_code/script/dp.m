clear;
clc;
% �Ľ�����x��z theta d a alpha offset
ML(1) =  Link([ 0,0,0,0,0],'modified');
ML(2) =  Link([ 0,0,0,-pi/2,0],'modified');
ML(1) =  Link([ 0,0,0,0,0],'modified');
ML(2) =  Link([ 0,0,1,-pi/2,0],'modified');
ML(3) =  Link([ 0,0,1,0,0],'modified');
ML(4) =  Link([ 0,0,1,pi/2,0],'modified');
ML(5) =  Link([ -pi/2,0,0,-pi/2,0],'modified'); % �˴�ȡ-90�����ҵ�ĳ�ʼ����ϵһ�¡�
ML(6) =  Link([ 0,0,0,-pi/2,0],'modified');
robot = SerialLink(ML,'name','Fanuc M20ia');
robot.teach(); % ���������϶��ĹؽڽǶ�



