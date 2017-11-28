function plotting(task, res, Ax, Ay)

figure;
subplot(3,1,1); hold on;

% plot the road
fill([0;1;1;0]*task.s(end),[0;0;1;1]*task.road.lanewidth*2,0.8*[1 1 1],'EdgeColor',0.7*[1 1 1]);
plot([0,task.s(end)],task.road.lanewidth*[1 1],'w--','LineWidth',2);
plot(task.s,task.zone.ymin,'b',task.s,task.zone.ymax,'b');
%plot(res_time.xErel,res_time.cz,'k',res_time.xErel,res_time.ow,'k');
plot(task.s,task.E.yref,'k-.');
plot(task.E.x0,task.E.y0,'r*');
plot(task.L.x0,task.L.y0,'b*');
if task.adjacentveh
    plot(task.A.x0,task.A.y0,'g*');
end
if task.adjacentveh2
    plot(task.A2.x0,task.A2.y0,'g*');
end

% plot optimal path
h1=plot(task.s,res.yE,'k','LineWidth',2);
xlim([0,task.E.horizon]); ylim([0 2]*task.road.lanewidth);
xlabel('Relative longitudinal position (m)','FontSize',16);
ylabel('Lateral position (m)','FontSize',16);
xlim([0,task.s(end)]);

legend([h1],'Optimal path');

subplot(3,1,2); hold on;
[AX,H1,H2]=plotyy(task.s,res.vEx*3.6,task.s,res.vEy,'plot');
set(get(AX(1),'Ylabel'),'String','Longitudinal speed (km/h)','FontSize',16)
set(get(AX(2),'Ylabel'),'String','Lateral speed (m/s)','FontSize',16) 
xlabel('Relative position of the ego vehicle (m)','FontSize',16);
% legend('Longitudinal speed','Lateral speed');
set(H1,'color','b','LineWidth',2);
set(H2,'color','r','LineWidth',2);
hold(AX(1));
hold(AX(2));
hold all
%h1=plot(res_time.xErel,res_time.vEx*3.6,'color','b','LineWidth',2);
%h2=plot(AX(2),res_time.xErel,res_time.vEy,'color','r','LineWidth',2);
% xlim(AX,[0,task.s(end)]);
legend([H1, H2],'long. Speed, space', 'lat. speed, space');

subplot(3,1,3); hold on;
[AX,H1,H2]=plotyy(task.s,Ax,task.s,Ay);
xlim([0,task.s(end)]);
set(get(AX(1),'Ylabel'),'String','Longitudinal acceleration (m/s^2)','FontSize',16)
 set(get(AX(2),'Ylabel'),'String','Lateral acceleration (m/s^2)','FontSize',16) 
 xlabel('Relative position of the ego vehicle (m)','FontSize',16);
 set(H1,'color','b','LineWidth',2);
 set(H2,'color','r','LineWidth',2);
 legend('Longitudinal speed','Lateral speed');
 hold(AX(1));
 hold(AX(2));
 hold all
%  xlim(AX, [0,task.s(end)]);
 legend([H1, H2], 'long acc, space', 'long. acc., space');