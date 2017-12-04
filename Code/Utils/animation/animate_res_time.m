% clear all;
clc;
movingframe=true;
saveobj=false;
task=task_time;

% load task2;
res=res_time;

XLim=[0,150]; YLim=[-2, 12];
XLIM=[0,res.xE(end)];
if task.oncomingveh
    XLIM(2)=max(XLIM(2),task.O.x0);
end
if ~movingframe
    XLim=XLIM;
end

%% Plot results in the road frame
if exist('fig','var') && ishandle(fig)
    figure(fig);
else
    fig=figure('units','normalized','outerposition',[0 0 0.68 1]);
%     fig=figure('units','normalized','outerposition',[0 0 1 1]);
    set(gca,'Unit','normalized','Position',[0.05 0.05 0.94 0.94],'Layer','top','XTick',[],'YTick',[], ...
        'XLimMode','manual','YLimMode','manual','ZLimMode','manual');
    set(gca,'Units','pixels');
    p=get(gca,'Position'); 
    p(2)=p(4)/2;                        % move it near the center of screen
%     p(3)=p(3)-50; p(1)=p(1)+50;         % make YLabel visible
    p(4)=p(3)*diff(YLim)/diff(XLim);    % keep realistic aspect ratio
    set(gca,'Position',p);
end
hold off;

% plot the road
x=[0,1,1,0]*XLIM(2);
y=[0 0 2 2]*task.road.lanewidth;
fill(x,y,0.8*[1 1 1],'EdgeColor',0.7*[1 1 1]); hold on;
plot(XLIM,task.road.lanewidth*[1 1],'w--','LineWidth',2);
plot(res.xE,res.yE,'Color',0.5*[1 1 1],'LineStyle','-.');
xlim(XLim); ylim(YLim);
xlabel('Position of the ego vehicle (m)');
ylabel('Lateral position (m)');
if movingframe
    title('Moving frame');
else
    title('Inertial frame');
end
box off;
h=gca;

% leading vehicle
Lvehg=hgtransform('Parent',h);
Lveh=plotveh(task.L.x0,task.L.y0,0,task.L.width,task.L.length,'r'); set(Lveh,'Parent',Lvehg);
x=[XLIM(1),task.L.x0+[-task.L.lf, -task.L.lf, task.L.lr, task.L.lr], XLIM(2)];
y=[task.zone.ymin(1) task.zone.ymin(1) task.zone.ymin+task.zone.deltaymin task.zone.ymin+task.zone.deltaymin task.zone.ymin(1) task.zone.ymin(1)];
ymin=plot(x,y,'b','Parent',h);  % plot the limit on minimum vertical position
x=[XLIM(1),task.L.x0+[-task.L.lflane, -task.L.lflane, task.L.lrlane, task.L.lrlane], XLIM(2)];
y=[task.zone.ymax(1) task.zone.ymax(1) task.zone.ymax+task.zone.deltaymax task.zone.ymax+task.zone.deltaymax task.zone.ymax(1) task.zone.ymax(1)];
ymax=plot(x,y,'b','Parent',h); % plot the limit on minimum vertical position

% egovehicle
k=1;
Evehg=hgtransform('Parent',h);
Eveh=plotveh(res.xE(k),res.yE(k),0,task.E.width,task.E.length,[0 0.5 1]); set(Eveh,'Parent',Evehg);
if movingframe
    txspeed=text(res.xE(k)-task.E.length/2,res.yE(k)+task.E.width+2,sprintf('%1.0f km/h',res.vEx(k)*3.6),'Parent',Evehg,...
        'VerticalAlignment','top','FontSize',14); % shown text with current speed
end

if task.oncomingveh
    Ovehg=hgtransform('Parent',h);
    Oveh=plotveh(task.O.x0,task.O.y0,pi,task.O.width,task.O.length,'g'); set(Oveh,'Parent',Ovehg);
    y=[0; 2*task.road.lanewidth];
    if task.egofirst
        x=-(y-task.O.y0 + task.road.lanewidth)/task.road.lanewidth*task.O.lf+task.O.x0;
    else
        x=(y-task.O.y0 + task.road.lanewidth)/task.road.lanewidth*task.O.lf+task.O.x0;
    end
    plot(x,y,'b','Parent',Ovehg);
end
if task.adjacentveh
    Avehg=hgtransform('Parent',h);
    Aveh=plotveh(task.A.x0,task.A.y0,0,task.A.width,task.A.length,'g'); set(Aveh,'Parent',Avehg);
    y=[0; 2*task.road.lanewidth];
    if task.egofirst
        x=(y-task.A.y0 + task.road.lanewidth)/task.road.lanewidth*task.A.lr+task.A.x0;
    else
        x=-(y-task.A.y0 + task.road.lanewidth)/task.road.lanewidth*task.A.lr+task.A.x0;
    end
    Aramp=plot(x,y,'b','Parent',Avehg);
end

if saveobj
    writerObj=VideoWriter('task.mp4','MPEG-4');
    writerObj.FrameRate=20;
    writerObj.Quality=100;
    open(writerObj);
end

hold off;
if saveobj
    Nt=300;
else
    Nt=1000;
end
t=linspace(0,25,Nt);
dt=t(2:Nt)-t(1:Nt-1);
xEt=interp1(res.t,res.xE,t);
vEt=interp1(res.t,res.vEx,t);
yEt=interp1(res.t,res.yE,t);
tovr=interp1(xEt,t,res.xE([find(sum(res.C_ow,2)>=2,1,'first'), find(sum(res.C_ow,2)>=2,1,'last')]));

tovr=interp1(xEt,t,[res.xE(find(sum(res.C_ow,2)>=2,1,'first')),xEt(end)]);
for k=2:Nt
% for k = 2:2
    Evehg.Matrix=makehgtform('translate',xEt(k),yEt(k)-yEt(1),0);
    Lvehg.Matrix=makehgtform('translate',task.L.vx*t(k),0,0);
    if task.oncomingveh
        Ovehg.Matrix=makehgtform('translate',task.O.vx*t(k),0,0);
        if t(k) >= tovr(1) && t(k) <= tovr(2) 
            Ovehg.Children(1).Visible='on';
        else
            Ovehg.Children(1).Visible='off';
        end
    end
    if task.adjacentveh
        Avehg.Matrix=makehgtform('translate',task.A.vx*t(k),0,0);
        if t(k)>=tovr(1) && t(k)<=tovr(2) 
            Avehg.Children(1).Visible='on';
        else
            Avehg.Children(1).Visible='off';
        end
    end
    ymin.XData(2:5)=ymin.XData(2:5)+task.L.vx*dt(k-1);
    ymax.XData(2:5)=ymax.XData(2:5)+task.L.vx*dt(k-1);
    if movingframe
        h.XLim=h.XLim + task.L.vx*dt(k-1);
        set(txspeed,'String',sprintf('%1.0f km/h',vEt(k)*3.6));        
    end
    
    if saveobj
        frame = getframe(gca,[-50 -50 h.Position(3)+50 h.Position(4)+50]);
        writeVideo(writerObj,frame);
    else
        drawnow update;
        pause(1e-3)
    end
end
if saveobj
    close(writerObj);
end