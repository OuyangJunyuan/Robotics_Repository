clear all;


N=120;
CON=25;
Xexpect=CON*ones(1,N);
X=zeros(1,N);
Xkf=zeros(1,N);
Z=zeros(1,N);
P=zeros(1,N);

%赋初值
X(1)=25.1;
P(1)=0.01;
Z(1)=24.9;
Xkf(1)=Z(1);

%噪声
Q=0.01;
R=0.25;
W=sqrt(Q)*randn(1,N);   %方差决定噪声大小
V=sqrt(R)*randn(1,N);   %方差决定噪声大小

%系统矩阵
F=1;
G=1;
H=1;
I=eye(1);   %本系统状态为一维


%模拟房间温度及测量过程，并滤波
for k=2:N
    %第一步，随时间推移，房间真实温度波动变化
    %k时刻的真实温度，温度计是不知道的，但是它又是真实存在的，因此我们要过滤杂波，尽量收敛到真实温度
    X(k)=F*X(k-1)+G*W(k-1);
    
    %第二步，随着时间推移，获取实时数据
    %温度计对k时刻房间温度的测量，Kalman滤波是站在温度计角度进行的。
    %他不知道此刻真实温度，只能站在本次测量值和上一次估计值来做处理。其目的是最大限度降低测量噪声R的影响。尽可能的逼近X（k);
    Z(k)=H*X(k)+V(k);
    
    %第三步，卡尔曼滤波。
    %有了k时刻的观测和k-1时刻的状态就可以滤波了。
    X_pre=F*Xkf(k-1);      %状态预测
    P_pre=F*P(k-1)*F'+Q;    %协方差预测
    Kg=P_pre*inv(H*P_pre*H'+R); %计算卡尔曼增益
    e=Z(k)-H*X_pre;     %新息
    Xkf(k)=X_pre+Kg*e;  %状态更新
    P(k)=(I-Kg*H)*P_pre;    %协方差更新
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%计算误差
Err_Messure=zeros(1,N); %测量值与真实值之间的偏差
Err_Kalman=zeros(1,N);  %估计值与真实值之间的偏差
for k=1:N
    Err_Messure=abs(Z(k)-X(k));
    Err_Kalman=abs(Xkf(k)-X(k));
end



t=1:N;
figure; %画图显示
%依次输出理论值，叠加过程噪声的真实值
%温度计测量值，卡尔曼估计值
plot(t,Xexpect,'-b',t,X,'-r',t,Z,'-ko',t,Xkf,'-g*');
%plot(t,X,'-r',t,Xkf,'-g*');
legend('期望值','真实值','观测值','卡尔曼滤波值');
%legend('真实值','卡尔曼滤波值');
xlabel('采样时间/s');
ylabel('温度值/c');

%误差分析图
figure %画图显示
plot(t,Err_Messure,'-b',t,Err_Kalman,'-k*');
legend('测量偏差','卡尔曼滤波偏差');
xlabel('采样时间/s');
ylabel('温度值/c');
