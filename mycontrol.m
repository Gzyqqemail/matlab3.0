function u = mycontrol(i)
global Highway
global UavTeam

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pcur  =  UavTeam.Uav(i).CurrentPos;
Vcur  =  UavTeam.Uav(i).Velocity;
Pdes  =  UavTeam.Uav(i).Waypoint(:,UavTeam.Uav(i).CurrentTaskNum);
   rm = UavTeam.Uav(i).r;
   ra = UavTeam.Uav(i).ra;     
 vmax = UavTeam.Uav(i).vmax;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Area Definition
pt1  = Highway(1).ph1; 
pt2  = Highway(1).ph2;
 rt = Highway(1).rh;
pt3 = Highway(1).ph2 + [0 rt]';
 ksi  = Pcur + 1/UavTeam.gain*Vcur;%滤波位置
 rb = 0;
 rsr= 10000;%另两个虚拟管道的半径
 rrt= 10000;
 
 if UavTeam.Uav(i).State == 1  % Left Standby Area
    psr1 = [pt2(1) rt+rsr]';
    psr2 = [-rsr rt+rsr]';%这里是不是有问题，应该是[-rb rt+rsr]'，管道终点线在-rsr处？
    psr3 = [-rsr rt]';
       u = mytunnel(i,psr1,psr2,psr3,rsr);
    
 elseif  UavTeam.Uav(i).State == 2 % Right Standby Area
   psr1p = [pt2(1) -rt-rsr]';
   psr2p = [-rsr -rt-rsr]';
   psr3p = [-rsr -rt]';
       u = mytunnel(i,psr1p,psr2p,psr3p,rsr); 
       
 elseif  UavTeam.Uav(i).State == 3 % Left Ready Area
   prt1 = [-rrt rt+rrt]';%把准备区的长度也设为rrt了，长度等于一半的宽度
   prt2 = [-rrt 0]';%把管道终点线设为纵坐标为0了
   prt3 = [-2*rrt    0]';
       u = mytunnel(i,prt1,prt2,prt3,rrt); 
       
 elseif  UavTeam.Uav(i).State == 4 % Right Ready Area 
   prt1p = [-rrt -rt-rrt]';
   prt2p = [-rrt 0]';
   prt3p = [0    0]';
       u = mytunnel(i,prt1p,prt2p,prt3p,rrt); 
       
 else% Tunnel and Tunnel Extension
       u = mytunnel(i,pt1,pt2,pt3,rt); 
 end
end


 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function u = mytunnel(i,p1,p2,p3,rh)
global UavTeam

Pcur  =  UavTeam.Uav(i).CurrentPos;
Vcur  =  UavTeam.Uav(i).Velocity;
Pdes  =  UavTeam.Uav(i).Waypoint(:,UavTeam.Uav(i).CurrentTaskNum);
rs = UavTeam.Uav(i).rs;
ra = UavTeam.Uav(i).ra;
vmax = UavTeam.Uav(i).vmax;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
At12 = eye(2) - (p1-p2)*(p1-p2)'/(norm(p1-p2)*norm(p1-p2));
At23 = eye(2) - (p2-p3)*(p2-p3)'/(norm(p2-p3)*norm(p2-p3));

% Velocity to line
k1 = 1;
el = Pcur + 1/UavTeam.gain*Vcur - p2;
Vl = -mysat(k1*At23*el,vmax);

% Velocity away other multicoptee
Vm = [0;0];
for l = 1:UavTeam.AvailableNumMax
    if i~=l && norm(Pcur)~=0
        k2 = 1; e = 0.000001;
        ksimil = (Pcur - UavTeam.Uav(l).CurrentPos) + 1/UavTeam.gain*(Vcur - UavTeam.Uav(l).Velocity);
        temp = (1+e)*norm(ksimil) - (2*rs)*mys(norm(ksimil)/(2*rs),e);
        bil = k2*dmysigma(norm(ksimil),2*rs,rs+ra)/temp ...
            - k2*mysigma(norm(ksimil),2*rs,rs+ra)*((1+e)-dmys(norm(ksimil)/(2*rs),e))/(temp^2);
        Vm = Vm - bil*(ksimil/norm(ksimil));
    end
end

% Velocity within the tunnel
Vt = [0;0];
k3 = 1; e = 0.001;
ksihi = At12*((Pcur-p2) + 1/UavTeam.gain*Vcur);
temp1  = (rh-rs)/(norm(ksihi)+e);
temp2  = (rh-rs) - norm(ksihi)*mys(temp1,e);
%% wrong 负号
ci = -k3*dmysigma(rh-norm(ksihi),rs,rs+ra)/temp2 ...
    - k3*mysigma(rh-norm(ksihi),rs,rs+ra)*(-mys(temp1,e)+norm(ksihi)*dmys(temp1,e)*(rh-rs)/(norm(ksihi)+e)/(norm(ksihi)+e))/(temp2^2);
Vt = Vt - ci*(ksihi/norm(ksihi));

% Sum of all velocities
u = mysat(Vl+Vm+Vt,vmax);
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function u=mysat(x,a)
% ensure the direction to be the same
    if norm(x)>a
       u =a*x/norm(x);
    else
       u =x ;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u=mys(x,rs)

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = x;
elseif x1<=x  & x<=x2
    u = (1-rs)+sqrt(rs^2-(x-x2)^2);
else
    u = 1;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u=mysigma(x,d1,d2)

if x<=d1
    u = 1;
elseif d1<=x  & x<=d2
    A = -2/((d1-d2)^3); B = 3*(d1+d2)/((d1-d2)^3); C = -6*d1*d2/((d1-d2)^3); D = d2^2*(3*d1-d2)/((d1-d2)^3);
    u = A*x^3 + B*x^2 + C*x + D;
else
    u = 0;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u=dmys(x,rs)

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = 1;
elseif x1<=x  & x<=x2
    u = (x2-x)/sqrt(rs^2-(x-x2)^2)/2;
else
    u = 0;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u=dmysigma(x,d1,d2)

if x<=d1
    u = 0;
elseif d1<=x  & x<=d2
    A = -2/((d1-d2)^3); B = 3*(d1+d2)/((d1-d2)^3); C = -6*d1*d2/((d1-d2)^3);
    u = 3*A*x^2 + 2*B*x + C;
else
    u = 0;
end
end
  