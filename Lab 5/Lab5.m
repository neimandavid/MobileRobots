global robot, global isSim, global vMax, global tMove, global tStart, global wheelbase, global simle, global simre, global simlv, global simrv;
isSim = false;
if(~isSim)
    robot = raspbot();
end
simle = 0; simre = 0;
simlv = 0; simrv = 0;
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m

tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

x = 0; y = 0; th = 0; %State variables
xp = 0; yp = 0; thp = 0; %Predicted stuff
t = -0.1; %t starts at -0.1 to compensate for my first encoder reading
%I only ever use t to compute the trajectory
%I use dt for all the estimation so this should be okay
data = [x; y; th; t];
pdata = [x; y; th; t];
edata = [0; 0; 0; t];
cumerror = [0; 0; 0];
oldenc = getEncoders();
move(0, 0);
pause(0.1); %Allow another set of encoder readings to come in for ease of math

%Figure 8 parameters:
ks = 3;
vff = 0.2;
sf = 1;
tf = sf/vff*ks;
kth = 2*pi/sf;
kk = 15.1084;
%Notes: They wanted me to use a scaled time T off the computer clock
%T would be scaled using ks to t
%Instead, I scaled tf and s by ks, which should be equivalent
%Basically, anything using a value of t uses the value t/ks, t being real time
%whereas before it would use t = T/ks, T being real time

wff = 0;
vcomm = vff;
wcomm = 0;

%PID gains
kcoeff = 1;
kx = 3;
ky = 30;
kt = 3; %kth is already taken!!!
kdx = 0.1;
kix = 0.001;
kdy = 1;
kiy = .1;
kdt = 0.1;
kit = 0.01;

c = 0;
d = 0;
firstloop = true;

close all

while true  
   newenc = getEncoders(); %Ideally would have a clever wait here
   %Compute dt and relevant speeds/distances
   dt = newenc(3)-oldenc(3);
   if dt == 0
       'No new encoder data'
       c = c + 1
       d = d + 1
       pause(0.05);
       continue
   end
   if norm(newenc-oldenc) > 0.3 & d < 3 & firstloop == false
       'Bad encoder data?'
       newenc-oldenc
       c = c + 1
       d = d + 1
       pause(0.05)
       continue
   end
   d = 0;
   firstloop = false;
   %'Yay, new encoder data!'
   vl = (newenc(1)-oldenc(1))/dt;
   vr = (newenc(2)-oldenc(2))/dt;
   [Vmeas, wmeas] = IKcomp(vl, vr);
   %Update state estimate (using midpoint method; could use Runge-Kutta, etc.)
   th = th + wmeas*dt/2;
   x = x + Vmeas * cos(th)*dt;
   y = y + Vmeas * sin(th)*dt;
   th = th + wmeas*dt/2;
   %Update state prediction
   thp = thp + wff*dt/2;
   xp = xp + vff*cos(thp)*dt;
   yp = yp + vff*sin(thp)*dt;
   thp = thp + wff*dt/2;
   t = t + dt;
   temp = [x; y; th; t];
   data = [data, temp];
   tempp = [xp; yp; thp; t];
   pdata = [pdata, tempp];
   %Compute trajectory
   s = vff*t/ks;
   k = kk/ks*sin(kth*s);
   wff = k*vff;
   if t >= tf
       vff = 0;
       wff = 0;
   end
   %Add PID stuff here
   xdiff = xp-x;
   ydiff = yp-y;
   R = [cos(th), sin(th); -sin(th), cos(th)];
   temp = R*[xdiff; ydiff];
   xdiff = temp(1);
   ydiff = temp(2);
   thdiff = thp - th; %Normalize this to [-pi, pi]
   while thdiff < -pi
       thdiff = thdiff + 2*pi;
   end
   while thdiff > pi
       thdiff = thdiff - 2*pi;
   end
   tempe = [xdiff; ydiff; thdiff; t];
   cumerror = cumerror + [xdiff; ydiff; thdiff]*dt;
   cumerror(abs(cumerror) > 1) = cumerror(abs(cumerror) > 1)./abs(cumerror(abs(cumerror) > 1));
   edata = [edata, tempe];
   derror = [edata(1:3, end) - edata(1:3, end-1)]/dt;
   derror(abs(derror) > 1) = derror(abs(derror) > 1)./abs(derror(abs(derror) > 1));
   %xdiff, ydiff are now error in robot frame
   vpid = kcoeff*(kx*xdiff+kdx*derror(1)+kix*cumerror(1));
   wpid = kcoeff*(ky*ydiff+kdy*derror(2)+kiy*cumerror(2) + kt*thdiff+kdt*derror(3)+kit*cumerror(3));
   
   vcomm = vff + vpid;
   wcomm = wff + wpid;
   
   FKmove(vcomm, wcomm);
   %Plot stuff
   plot(data(1,:), data(2,:));
   hold on
   plot(pdata(1, :), pdata(2, :));
   hold off
   %quiver(x, y, Vmeas*cos(th), Vmeas*sin(th));
   axis([-0.6 0.6 -0.6 0.6]);
   title('Robot Trajectory');
   xlabel('x (m)');
   ylabel('y (m)');
   %Update encoder buffer
   oldenc = newenc;
   %Pause so Matlab doesn't hate me...
   pause(0.05);
   %Check for exit
   if t >= tf+1
       move(0, 0); %Stop
       break
   end
end

plot(edata(4, :), edata(1, :));
hold on
plot(edata(4, :), edata(2, :));
plot(edata(4, :), edata(3, :));
title('Errors');
xlabel('Time (s)');
ylabel('Error (m or rad)');
legend('x', 'y', 'theta');
hold off

figure;
hold on
plot(data(1, :), data(2, :));
plot(pdata(1, :), pdata(2, :));
title('Trajectories');
xlabel('X (m)');
ylabel('Y (m)');
legend('Actual', 'Predicted');
hold off

figure;
hold on
for x = 1:3
    plot(data(4, :), data(x, :));
    plot(pdata(4, :), pdata(x, :));
end
title('Stuff vs. Time');
xlabel('Time (s)');
ylabel('Parameter (m or rad)');
legend('x', 'x predicted', 'y', 'y predicted', 'theta', 'theta predicted');
hold off

'Finished moving'

function [V, w] = IKcomp(vl, vr)
    global wheelbase;
    V = (vl+vr)/2;
    w = (vr-vl)/wheelbase;
end

%Given a linear velocity V (m/s) and an angular velocity w (rad/s)
%compute and send the corresponding wheel velocities
function FKmove(V, w)
    global wheelbase;
    vl = V - w/2*wheelbase;
    vr = V + w/2*wheelbase;
    move(vl, vr);
end

function out = findClosest()
    ranges = getRanges();
    [r th] = min(ranges);
    out = [r, th];
end

%Sets data outside of [0.06, 2] to NaN, which is clearly garbage data
function ranges = getRanges()
    ranges = rawRanges();
    for x = 1:360
        if ranges(x) < 0.06 || ranges(x) > 2
            ranges(x) = nan; %Doesn't count as max or min
        end
    end
end

%Makes 0 be right, 90 forward, etc.
%Not sure about 1-indexing; also not sure about the 5 degrees
%But this should be close
function ranges = rawRanges()
    global robot, global isSim;
    if(~isSim)
        tempRanges = robot.laser.LatestMessage.Ranges;
        ranges = zeros(360);
        ranges(1:90) = tempRanges(271:360);
        ranges(91:360) = tempRanges(1:270);
    else
        ranges = zeros(360, 1);
        ranges(90) = 0.06 + 1.94*rand();
        %Whatever test data we want here
    end
end

%Copy/pasted from Lab 1 final version (then cleaned up):

%Wrapper for reading encoders
%Returns a length 2 vector
%Actual values if not simulating; simulated values otherwise
function e = getEncoders()
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove, global data, global origEnc
    if ~isSim
        tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
        e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y, tstamp];
    else
        e = [simle, simre, toc(tStart)];
    end
end

%Wrapper for setVelocity
%Sets simulated values and sends commands to the robot
function move(lv, rv)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vMax, global robot, global tStart, global tMove, global data, global origEnc

    %Cap the speeds
    if lv > vMax
        lv = vMax;
    end
    if lv < -vMax
        lv = -vMax;
    end
    if rv > vMax
        rv = vMax;
    end
    if rv < -vMax
        rv = -vMax;
    end
    
    if ~isSim
        robot.sendVelocity(lv, rv); %They flipped the order back, so I will too
    end
    dt = toc(tMove); %Get time since last move command
    if dt > 1 %Adjust for watchdog
        dt = 1;
    end
    ld = simlv*dt;
    rd = simrv*dt;
    simle = simle + ld; %Simulate encoder values forward
    simre = simre + rd;
    simlv = lv;
    simrv = rv;
    tMove = tic;
end