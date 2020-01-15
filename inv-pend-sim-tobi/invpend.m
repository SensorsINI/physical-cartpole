% inverted pendulum simulation with mouse and PD control of motor drive
function invpend
% L=half pend length, tracklength=half cart track length, 
% umax=max motor force, Q=dimensionless motor drive
% joy=joystick (if installed), filename=log file name
global L umax Q tracklength stopme x xd a ad logfile control joy filename
resetonfail=false;  % if pendulum angle crosses 90 or -90 reset()
m=4; % mass of pend, kg
M=10; % mass of cart, kg
Mfric=10; % cart friction, N/m/s
Jfric=.1; % friction coefficient on angular velocity, Nm/rad/s
g=9.8; % gravity, m/s^2
L=4; % half length of pend, m
J=(m*(2*L)^2)/12; % moment of inertia around center of pend, kg*m
tracklength=3*2*L; % half of total length of cart track,m
umax=300; % max cart force, N
maxv=100; % max DC motor speed, m/s, in absense of friction, used for motor back EMF model
dt=0; % delta time,s, set by simulation and animation loop time
controlDisturbance=0.1; % disturbance, as factor of umax
sensorNoise=0.01; % noise, as factor of max values
fps=15; % desired animation rate, Hz
fpsavg=15;
control=false; % enable feedback control
kpa=30; kda=3; kpx=3; kdx=0;  % PD parameters for angle (a) and position (x) control
try
    joy=vrjoystick(1);
catch
    disp('no joystick available, use mouse around cart position to control motor drive')
    clear joy
end
% starting state
reset()
try 
    opengl hardware
catch
    fprintf('cannot switch to opengl on linux at runtime\n');
end
fg=figure(1);
cla
axis equal
grid on
set(gca,'ylim',[-2*L,2*L],'xlim',[-tracklength-2*L,tracklength+2*L])
set(fg, 'UserData', []);
if ~exist('joy','var')
    set(gcf,'windowbuttonmotionfcn',@(s,e) motor(get(gca,'CurrentPoint')))
end

set(gcf,'keypressfcn',@(s,e) keyfcn(e))
t=0 ; % time now,s
% helpers
jml2=J+m*L^2;
ml=m*L;
stopme=false;
logfile=openlogfile();
tic
thistoc=toc;
lastsimtoc=toc;
lastdrawtoc=toc;
Q=0; % must exist at all, set by mouse inside figure if no joystick
while ~stopme
    ueff=umax*(1-abs(xd)/maxv)*Q; % dumb model of EMF of motor, Q is drive -1:1 range
    ueff=ueff+controlDisturbance*randn()*umax; % noise on control
    if ueff<-umax
        ueff=-umax;
    elseif ueff>umax
        ueff=umax
    end
    if x<=-tracklength || x>=tracklength
        ueff=0;
    end
  
    % update xd, ad, then x,a
    ca=cos(a);
    sa=sin(a);
    A=jml2+(ml^2*(cos(a))^2/(M+m));
    B=M+m+(ml^2)/jml2;
    add=(-ml/(M+m)*ca*ueff+m*g*L*sa-ml*(1+1/(M+m))*ad^2*ca*sa-ad*Jfric)/A;
    xdd=(ueff-g*ml^2*sa/jml2+ml^2*L*ad^2*ca*sa/jml2-xd*Mfric)/B;
    ad=ad+add*dt;
    xd=xd+xdd*dt;
    a=a+ad*dt;
    x=x+xd*dt;
    
    % check endpoints
    if x<=-tracklength
        xd=0;
        x=-tracklength;
    elseif x>=tracklength
        xd=0;
        x=tracklength;
    end
    
    if resetonfail
        if a>pi/2 || a<-pi/2
           reset()
           pause(1)
           Q=0;
       end
    end
        
    
    % maybe draw
    thistoc=toc;
    if (thistoc-lastdrawtoc)>=1/fps ||  (thistoc-lastdrawtoc)<0
        mixfactor=.01;
        fpsavg=fpsavg*(1-mixfactor)+mixfactor/(thistoc-lastdrawtoc);
        cla
        title(sprintf('t=%8.3fs ueff=%.2f v=%.2f control=%d',t,ueff,xd,control))
        line([x,x+2*L*sin(a)],[0,2*L*cos(a)],'Color','b');
        rectangle('Position',[x-L/4,0,L/2,L/5])
        line([-tracklength,-tracklength],[-L,L],'Color','k');
        line([+tracklength,+tracklength],[-L,L],'Color','k');
        line([x,x+2*L*ueff/umax],[0,0],'Color','r');
        drawnow();
        lastdrawtoc=thistoc;
        if control  % only control on rendering cycle, like human
            Q=kpa*rem(a,2*pi)+kda*ad+kpx*x+kdx*xd;
            %         fprintf('a=%f kpa*a=%f ad=%f kda*ad=%f x=%f x*kpx=%f xd=%f xd*kdx=%f Q=%f\n',a,kpa*rem(a,2*pi),ad,kda*ad,x,kpx*x, xd,kdx*xd,Q);
        else
            if exist('joy','var')
                Q=axis(joy,1);
            end
        end
    end
    % log data
    fprintf(logfile,'%g,%g,%g,%g,%g,%g,%g,%d\n', t,x,xd,a,ad,ueff,Q,control);
    % next time
    dt=thistoc-lastsimtoc;
    t=t+dt;
    lastsimtoc=thistoc;
    %     fprintf('t=%g dt=%g fpsavg=%g\n', t,dt,fpsavg);

end
title 'stopped'
fclose(logfile);
fprintf('saved data log file %s\n',filename)
end

function motor(point)
global L Q tracklength x
if ~isempty(point)
    %     disp(point)
    xp=point(1,1);
    yp=point(1,2);
    if xp>tracklength+2*L || xp<-tracklength-2*L || yp<-2*L || yp>2*L
        Q=0;
    else
        Q=((xp-x)/(2*L));
    end
    %     fprintf('x=%f y=%f tracklength=%f u=%f\n',x,y,tracklength,u);
end
end

function reset()
global x xd a ad
fprintf('reset\n');
x=-.5; % pos, m
a=0; %deg2rad((rand()-.5)); % angle, rad
xd=0; % xdot, m/s
ad=0; % angle dot, rad/s
t=0;
tic;
end

function keyfcn(eventdata)
global stopme logfile control

if eventdata.Key=='0'
    reset();
elseif eventdata.Key=='x'
    stopme=true
elseif eventdata.Key=='c'
    control=~control
end
end

function f=openlogfile()
    global filename
    filename=sprintf('data/pend-%s.csv',datestr(now,'yyyy-mm-dd-HH-MM-SS'));
    f=fopen(filename,'w');
    fprintf(f,'t,x,xd,a,ad,u,Q,control\n');
end