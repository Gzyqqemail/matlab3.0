function out = MyMap(UavTeam,Obstacle,Highway)
global time
persistent gfigure;
if isempty(gfigure)
    gfigure = 1;
end
M = UavTeam.AvailableNumMax;
% 2D map
% for k = 1: M
%     for i = 1:UavTeam.Uav(k).AllTaskNum  
%     plot(UavTeam.Uav(k).Waypoint(1,i), UavTeam.Uav(k).Waypoint(2,i),'rp')
%     hold on
%     end
% end
% end

% Highway plot
line11 = [Highway(1).ph1(1),Highway(1).ph2(1)-4500];
line12 = [Highway(1).ph1(2)+Highway(1).rh,Highway(1).ph2(2)+Highway(1).rh];
plot(line11,line12);
hold on
line21 = [Highway(1).ph1(1)  Highway(1).ph2(1)-4500]';
line22 = [Highway(1).ph1(2)-Highway(1).rh  Highway(1).ph2(2)-Highway(1).rh]';
plot(line21,line22);
hold on
%计算每个无人机的位置和滤波位置，并画出无人机的三个区域
for k = 1: M
    o1 = [UavTeam.Uav(k).CurrentPos(1) UavTeam.Uav(k).CurrentPos(2)]';
    o2 = [UavTeam.Uav(k).CurrentPos(1)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(1) UavTeam.Uav(k).CurrentPos(2)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(2)]';
    mydrawcolorball(o1,o2,k);  
end
axis([-600 600 -600 600]) 
grid on
xlabel('x')
ylabel('y')
% 在图的特定位置显示仿真时间
xPosition = -600; % x 轴位置
yPosition = 500; % y 轴位置
text(xPosition, yPosition, ['仿真时间: ' num2str(time)], 'FontSize', 12, 'Color', 'red');
title(['第' num2str(gfigure) '张图'] );
gfigure = gfigure+1;
out = 0;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out=mydrawcolorball(o1,o2,k)
%     if (k==1) 
%         plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','y','MarkerFaceColor', 'y');
%         radiusplot(o1,o2,k)
% %         hold on
%     elseif (k==2)
%        plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','m','MarkerFaceColor', 'm');
%        radiusplot(o1,o2,k)
% %         hold on
%     elseif (k==3)
%        plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','c','MarkerFaceColor', 'c');
%        radiusplot(o1,o2,k)
% %         hold on
%     elseif (k==4)
%         plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','r','MarkerFaceColor','r');
%         radiusplot(o1,o2,k)
% %         hold on
%     elseif (k==5)
%         plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','g','MarkerFaceColor', 'g');
%         radiusplot(o1,o2,k)
% %         hold on       
%     elseif k==6
%         plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','b','MarkerFaceColor', 'b');
%         radiusplot(o1,o2,k)
% %         hold on  
%     else
%         plot(o1(1),o1(2),'.','MarkerSize',15,'MarkerEdgeColor','k','MarkerFaceColor', 'k');
        radiusplot(o1,o2,k)
%         hold on        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out=radiusplot(o1,o2,k)
global rm rs ra rd
%         mycircle(o1,rm)
mycircle_y(o1,rd)
mycircle_g(o2,ra)
mycircle_b(o2,rs)
plot(o1(1),o1(2),'k-o','MarkerFaceColor','k','MarkerSize',4)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = mycircle_b(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'b-') 
end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function out = mycircle_g(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'g-') 
 end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function out = mycircle_y(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'y-') 
 end