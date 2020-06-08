clear all;
l1 = 1; l2 =1 ; r1 = 0.5 ; r2 = 0.5 ; m1 =12 ; m2 =12 ; 
ix1 = 0.0125 ; ix2 = 0.0125 ;iy1 = 1.0025 ; iy2 =1.0025 ; iz1 =1.01 ; iz2=1.01 ;
%problem 1
pset = [5,15,40]; tset = [0, 1, 2];
a = 80; dt = 0.01;
[x1out,t]=LFPB_ViaPoint(pset,tset,a,dt);

T=0.0001;
t = t(1):T:t(end); N = length(t); 
Kv = [0.05,0;0,0.05] ; Kp = [7.5,0;0,4.5]; torquemax=20;

xd=[deg2rad(x1out);deg2rad(x1out)];
xnow = zeros(2,N); vnow = zeros(2,N);  torquenow = zeros(2,N); ex = zeros(2,N);
anow=[0;0]; exlast = [0:0] ;
count=0;kk=1;
for k = 1:N
    count = count +1;
    if(count == dt/T)
        count=0;
        kk = kk+1;
    end
    %获取偏差信号
    ex(:,k) = xd(:,kk) - xnow(:,k);
    ev = ( ex(:,k) - exlast )/T;
    
    %获取控制量
    x1 = xnow(1,k); x2 = xnow(2,k); v1 = vnow(1,k) ; v2 = vnow(2,k) ;
    m11 = m1*r1^2 + iz1 + iz2 + (l1*sin(x2))^2*m2 + (r2+l1*cos(x2))^2*m2; 
    m12 = (r2+l1*cos(x2))*m2*r2 + iz2;  m21 = m12; m22 = r2^2*m2 + iz2;

    c11 = -m2*l1*r2*sin(x2)*v2; c12 = -(m2*l1*r2*sin(x2))*(v1+v2);
    c21 = m2*l1*r2*sin(x2)*v1;
    c22 = 0;

    Mnow = [m11,m12;m21,m22];
    Cnow = [c11,c12;c21,c22];
    torquenow(:,k) = Mnow*(Kv*ev+Kp*ex(:,k)) + Cnow*vnow(:,k);
    
    %模拟得到输出
    %饱和特性
    if abs(torquenow(1,k))>torquemax
        torquenow(1,k) = sign(torquenow(1,k))*torquemax;
    end
    if abs(torquenow(2,k))>torquemax
        torquenow(2,k) = sign(torquenow(2,k))*torquemax;
    end
    anow = inv(Mnow)*(torquenow(:,k)- Cnow*vnow(:,k) );
    vnow(:,k+1) = vnow(:,k) + anow*T;
    xnow(:,k+1) = xnow(:,k) + (vnow(:,k)+vnow(:,k+1))/2*T;
    exlast = ex(:,k);
end
subplot(2,2,1);plot(t,torquenow(:,1:N)); title('torque');legend('1','2');
subplot(2,2,2);plot(t,xnow(:,1:N)); title('angle');legend('1','2');
subplot(2,2,3);plot(t,vnow(:,1:N)); title('v');legend('1','2');
subplot(2,2,4);plot(t,ex(:,1:N)); title('e');legend('1','2');
