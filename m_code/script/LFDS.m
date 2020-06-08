%%
clear all;
of=[5,2,3,1,0,0];
lim=[10,10,30];
to=0;
dt = 0.01;

[pout,tout]=LFDS_(of,lim,to,dt);
plot(tout,pout);
%%
function [pout,tout]=LFDS_(of,lim,to,dt)
po = of(1); pf = of(2); vo = of(3) ; vf = of(4) ; %ao = of(5) ; af =of(6);
vmax = lim(1) ; amax = lim(2) ; jmax = lim(3) ;

sigma = sign(pf - po);  
po = sigma*po;
pf = sigma*pf;
vo = sigma*vo;
vf = sigma*vf;
vmax = ((sigma+1)/2)*vmax + ((sigma-1)/2)*(-vmax);
amax = ((sigma+1)/2)*amax + ((sigma-1)/2)*(-amax);
jmax = ((sigma+1)/2)*jmax + ((sigma-1)/2)*(-jmax);

alima = 0; alimd = 0; vlim = 0;
Ta = 0 ; Td = 0; Tj1 = 0 ; Tj2 = 0;
if (vmax-vo)*jmax < amax^2 %判断规划过程 加速度是否达到 amax
    Tj1 = sqrt( (vmax-vo)/jmax ) ;
    Ta = 2*Tj1;
    alima = jmax * Tj1;
else %能达到amax
    Tj1 = amax/jmax ;
    Ta = Tj1 + (vmax - vo)/amax;
    alima = amax;
end
    
if (vmax-vf)*jmax <amax^2 % 判断规划过程 加速度是否达到 -amax
    Tj2 = sqrt( (vmax - vf)/jmax ) ;
    Td = 2*Tj2 ;
    alimd = -jmax*Tj1;
else    %能达到 -amax
    Tj2 = amax/jmax ;
    Td = Tj2 + (vmax - vf)/amax ;
    alimd = -amax;
end

Tv = (pf - po)/vmax - Ta/2*(1+ vo/vmax) - Td/2*(1+ vf/vmax);


if Tv < 0 %不存在匀速时间段， 即最大速度达不到 vmax
    Tv=0 ;
    while 1
        Tj1 = amax/jmax ; Tj2 = Tj1 ;
        delta = amax^4/jmax^2 + 2*(vo^2 + vf^2) + amax*(4*(pf-po)-2*amax/jmax*(vo+vf));
        Ta = ( amax^2/jmax - 2*vo + sqrt(delta) )/(2*amax);
        Td = ( amax^2/jmax - 2*vf + sqrt(delta) )/(2*amax);

        if Ta <0 || Td <0 %没有加速段（第一个)或没有减速段（第二个s）
            if Ta < 0  %没有加速段（第一个)
                Ta = 0; Td = 2*(pf -po)/(vo+vf);
                Tj1 = 0; Tj2 = (jmax*(pf-po)- sqrt(jmax*( jmax*(pf-po)^2 + (vf+vo)*(vf+vo)*(vf-vo) ) ) )/(jmax*(vf+vo)); 
                alima = 0;
                alimd = -jmax*Tj2;
                vlim = vo;
            else  %没有减速段（第二个s）
                Ta = 2*(pf -po)/(vo+vf); Td=0; 
                Tj1 = (jmax*(pf-po)- sqrt(jmax*( jmax*(pf-po)^2 - (vf+vo)*(vf+vo)*(vf-vo) )) )/(jmax*(vf+vo)); Tj2 = 0;
                alima = jmax*Tj1;
                alimd = 0;
                vlim = vo + alima*(Ta - Tj1);
            end
            break;
        else %第八步
            if Ta>=2*Tj1 && Td >=2*Tj2 %能达到最大加速度，即存在匀加速阶段。
                alima = amax;
                alimd = -amax;
                vlim = vo + alima*(Ta - Tj1);
                break;
            else
                amax = amax * 0.99;
            end
        end
    end
else
    vlim=vmax;
end
%%
T=Ta+Td+Tv;
tout=to:dt:T;
count=1;
pout=zeros(1,length(tout));
for t=tout
    if t <= Tj1
        pout(count) = po + vo*t + (jmax*t^3)/6;
    elseif  t<= Ta-Tj1
        pout(count) = po + vo*t + (alima/6)*( 3*t^2 - 3*Tj1*t + Tj1^2 );
    elseif  t<= Ta
        pout(count) = po + (vlim+vo)*Ta/2 - vlim*(Ta-t) + jmax/6*(Ta-t)^3; 
        
    elseif  t<= Ta+Tv
        pout(count) = po + (vlim+vo)*Ta/2 + vlim*(t-Ta);
        
    elseif  t<= T-Td + Tj2
        pout(count) = pf - (vlim+vf)*Td/2 + vlim*(t-T+Td) - (jmax*(t-T+Td)^3)/6;
    elseif  t<= T-Tj2
        pout(count) = pf - (vlim+vf)*Td/2 + vlim*(t-T+Td) + (alimd/6)*( (3*(t-T+Td)^2) -3*Tj2*(t-T+Td)+Tj2^2); %注意公式中是alimd ，是负的？？注意公式中lim，min，limd，lima的意思。
    elseif  t<= T
        pout(count) = pf - vf*(T-t) - jmax*power((T-t),3) /6;
    end
    
    count = count + 1;
end
    pout=sigma*pout;
end