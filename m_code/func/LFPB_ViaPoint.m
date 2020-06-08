function [po,to]=LFPB_ViaPoint(pi,ti,ai,dt)
n=length(pi); %长度
to=ti(1):dt:ti(n); %时间点

%在起始点左侧虚拟半段，参数后缀：时间段开始点参数为1，时间段结束点参数为2，中间直线速度为v1，结束点右边直线速度为v2
h = pi(2)-pi(1); td = ti(2)-ti(1); 
ab2 = sign(h) * abs(ai);
tb2 = td - sqrt( td^2 - 2*h/ab2 ); tb2=tb2*2; %乘2是为了和后面的抛物线时间段长度一致，左右关于过点对称
v2 = h/(td - 0.25*tb2);

c=zeros(3,3); %时间段内轨迹参数，并初始化起点的抛物线参数。
c(3,1) = ab2/2; c(3,2) = v2 - 2*c(3,1)*(ti(1)+tb2/2); c(3,3) = pi(1) - ( c(3,1)*ti(1)^2 + c(3,2)*ti(1) );

%最后一个点计算
he = pi(end)-pi(end-1); tde = ti(end)-ti(end-1);
abe = -sign(he) * abs(ai);
tbe = tde - sqrt( tde^2 + 2*he/abe ); tbe=tbe*2;
ve =  he/(tde - 0.25*tbe);

%迭代标记
k=1;
count=2;%计数
temp = 0; %
po=zeros(1,length(to));
%计算时间段结束点参数2
for t=to(2:end) %从第二个点开始，第一个点为pi(1)
        if t>ti(k) %当前时间进入 ti(k)~ti(k+1)
            k = k+1;%更新计算点为时间段结束点，计算参数2：ab2、ab2
        
        if k == n %计算点是最后一个点
           tb1 = tb2; v1 = v2; %更新时间段开始参数为上一个时间段结束点参数。
           ab2 = abe; tb2 = tbe; %时间段结束点参数为之前计算的终止点参数。
        elseif k == n-1 %计算点是倒数第二个点，比较特殊，因为倒数第二个点右侧的速度是计算好的，必须是终止点直线速度。
           tb1 = tb2; v1 = v2;%更新时间段开始参数为上一个时间段结束点参数。
           v2 = ve; %倒数第二点为结束点的时间段结束点的直线时间是和终止点一样的。
           ab2 = sign(v2 - v1)*abs(ai); 
           tb2 = (v2-v1)/ab2;
        else
           tb1 = tb2; v1 = v2; %更新时间段开始参数为上一个时间段结束点参数。
           v2 =  (pi(k+1)-pi(k) )/( ti(k+1)-ti(k) ); %时间段结束点右侧直线速度由两个点之间的连线斜率计算。
           ab2 = sign(v2 - v1)*abs(ai); 
           tb2 = (v2-v1)/ab2;
        end
        
        if (tb1+tb2)/2> ti(k)-ti(k-1)  %直线段不存在
            t1 = ti(k) - tb2/2;
            t2 = t1;
            warning('加速度过小，直线加速度段消失，优先满足时间段结束点抛物线时间，结果可能有误');
        else
            t1 = ti(k-1) + tb1/2 ; t2 = ti(k) - tb2/2 ; %更新直线段起始和结束点
        end
        %分段轨迹参数计算
        c(1,1) = c(3,1) ; c(1,2) = c(3,2) ; c(1,3) = c(3,3) ; %本时间段起始抛物线为上个时间段结束点抛物线参数，故只需要赋值。
        temp = c(1,1)*t1^2 + c(1,2)*t1 + c(1,3); %计算直线段起始点

        c(2,1) = v1 ; c(2,2) = temp -  t1 *c(2,1); %计算直线参数
        temp = c(2,1)*t2 + c(2,2); %计算直线参数结束点

        c(3,1) = ab2/2 ; c(3,2) = v1 - 2*c(3,1)*t2 ; c(3,3) = temp - ( c(3,1)*t2^2 + c(3,2)*t2 );%计算时间段结束点处抛物线参数
   end
    if t<t1
        po(count) = c(1,1)*t^2 + c(1,2)*t + c(1,3);
    elseif t<t2 
        po(count) = c(2,1) * t + c(2,2);
    else
        po(count) = c(3,1)*t^2 + c(3,2)*t + c(3,3);
    end
    count=count+1;
end
po(1)=pi(1);
end

%配合这个使用，matlab自带的sign输入等于0的时候输出等于0，使得后续计算出错。
function output = sign0(input)
    if input >=0
        output = 1;
    else
        output = -1;
    end
end

