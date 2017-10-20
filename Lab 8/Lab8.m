global robot, global isSim, global vMax, global tMove, global tStart, global wheelbase, global simle, global simre, global simlv, global simrv, global simencnoise;
isSim = false;
if(~isSim)
    robot = raspbot();
end
simle = 0; simre = 0;
simlv = 0; simrv = 0;
rng('shuffle') %Get more randomness on simulated encoder noise (without this it's just psuedorandom)
simencnoise = .1; %Fraction of velocity reading to randomly add/subtract as noise
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m

tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

global minrange, global maxrange;
minrange = 0.06;
maxrange = 1;

global data;
data = [];

start = [0; 0; 0]; %Start position: [x, y, theta]
pose = start; %pose is where we actually are
goalpose = start; %goalpose is where we want to be

if ~isSim
    robot.startLaser;
    forksUp();
    pause(5);
    forksDown();
end

%Comment if we don't want the intermediate step
goToSail(0.127, 0, 0.2, pose, pose);
data = []; %Don't plot old trajectories
'First move'
%moveRelPos(.11, 0, 0, 0.2, pose, pose);
%goToSail(0.127, .05, 0.2, pose, pose);
'Second move'
forksUp();
pause(2);
forksDown();

robot.stopLaser;

%LIDAR code (partly from lab 2)
function [poseout, goalposeout] = goToSail(sailwidth, standoffdist, vMax, startpose, initialgoalpose)
    close all;
    global isSim;
    if isSim
        load('rangeData.mat');
    else
        rangeData = getNiceRanges(-2); %Positive makes the robot go left
    end

    close all
    plot(rangeData(3, :), rangeData(4, :), 'o') %x is forward, y is left
    sailpos = findSail(rangeData, sailwidth);
    if sailpos == [0; 0; 0]
        'No sail found'
        poseout = startpose;
        goalposeout = initialgoalpose;
    else
        [poseout, goalposeout] = moveRelPos(sailpos(1)+standoffdist*cos(sailpos(3)), sailpos(2)+standoffdist*sin(sailpos(3)), sailpos(3), vMax, startpose, initialgoalpose);
    end    
end

function cdata = findSail(ranges, sailwidth)
    figure(1)
    numpts = size(ranges, 2);
    
    d = Inf; %Shortest distance to a sail
    xc = 0; yc = 0; thc = 0; %Relative location of sail
    
    for x = 1:numpts
       startpt = rangeValue(x, ranges);
       oldpt = startpt;
       %Find a proposed line
       cumx = startpt(3);
       cumy = startpt(4);
       linepts = [startpt];
       for y = 1:numpts
           newpt = rangeValue(x+y, ranges);
           if mod(newpt(2)-startpt(2), 360) > 180 || norm(startpt(3:4) - newpt(3:4)) > sailwidth
               %If you've gone more than 180 degrees around from start, or
               %passed the width of a sail, stop
               break
           end
           cumx = cumx + newpt(3);
           cumy = cumy + newpt(4);
           linepts = [linepts, newpt];
           oldpt = newpt;
       end
       if y < 5
           %'Not enough points'
           continue %Skip stuff with less than 5 points
       end
       if norm(startpt(3:4)-oldpt(3:4)) < 0.5*sailwidth
           %'Sail too small'
           continue %Skip stuff whose width is off by more than 50%
       end
       
       %Get averages
       xbar = cumx/y;
       ybar = cumy/y;
       linepts = linepts - [0; 0; xbar; ybar];
       xp = linepts(3, :);
       yp = linepts(4, :);
       
       %Basically just copied from lab writeup...
       Ixx = xp * xp';
       Iyy = yp * yp';
       Ixy = - xp * yp';
       Inertia = [Ixx Ixy;Ixy Iyy] / y; % normalized
       lambda = eig(Inertia);
       lambda = sqrt(lambda)*1000.0;
       
       temp = cross([oldpt(3)-startpt(3), oldpt(4)-startpt(4), 0], [0, 0, 1]); %Points perpendicular to sail and away from origin
       thd = atan2(temp(2), temp(1)); %Alternate way to get angle, based
       %only on the endpoints
       
       thm = atan2(2*Ixy,Iyy-Ixx)/2.0; %Not sure what off-diagonal elts do, but I'm scared of sign errors...
       thm-thd
       th = thd;
       if lambda(1) > 1.3
           %'Not a line'
           continue %Skip stuff with really high variance
       end
       
       %Plot "sails"
       hold on
       xplot = [startpt(3), oldpt(3)];
       yplot = [startpt(4), oldpt(4)];
       plot(xplot, yplot, '-o');
       quiver(xbar, ybar, cos(th), sin(th));
       hold off
       
       if norm([xbar, ybar]) < d
           d = norm([xbar, ybar]);
           xc = xbar;
           yc = ybar;
           thc = th;
       end
       
    end
    cdata = [xc; yc; thc];
end

function out = rangeValue(index, ranges)
    out = ranges(:, mod(index-1, size(ranges, 2)) + 1);
end

function niceRanges = getNiceRanges(fudge)
    %r, theta, x, y
    ranges = getRanges(fudge);
    niceRanges = zeros(4, size(ranges(~isnan(ranges)), 1));
    count = 1;
    for x = 1:360
       if ~isnan(ranges(x))
          niceRanges(:, count) = [ranges(x); x; ranges(x)*cosd(x); ranges(x)*sind(x)];
          count = count + 1;
       end
    end
end

%Sets data outside of [0.06, 2] to NaN, which is clearly garbage data
function ranges = getRanges(fudge)
    global minrange, global maxrange;
    ranges = rawRanges(fudge);
    for x = 1:360
        if ranges(x) < minrange || ranges(x) > maxrange
            ranges(x) = nan; %Doesn't count as max or min
        end
    end
end

%Makes 0 be right, 90 forward, etc.
%Not sure about 1-indexing; also not sure about the 5 degrees
%But this should be close
function ranges = rawRanges(fudge)
    if nargin == 0
        fudge = 0;
    end
    if fudge < 0
        fudge = 360 + fudge;
    end
    global robot, global isSim;
    if(~isSim)
        tempRanges = robot.laser.LatestMessage.Ranges;
        ranges = zeros(1, 360);
        %I hope this is fudged correctly...
        %ranges(1:355) = tempRanges(6:360);
        %ranges(356:360) = tempRanges(1:5);
        ranges(1:fudge) = tempRanges(361-fudge:360);
        ranges(fudge+1:360) = tempRanges(1:360-fudge);
        %ranges = tempRanges;
    else
        ranges = zeros(1, 360);
        %ranges(90) = 0.06 + 1.94*rand();
        %Whatever test data we want here
    end
end

%Lab 7 code
function [poseout, goalposeout] = moveRelPos(xtarget, ytarget, thtarget, vMax, currentPose, curgoalpose)
    global data;
    global pdata;
    %currentPose and curgoalpose are in global coordinates (else currentPose = 0 all the time...)
    
    %Default argument values for convenience
    if nargin <= 3
        vMax = 0.2;
    end
    if nargin <= 4
        currentPose = [0, 0, 0];
    end
    if nargin <= 5
        curgoalpose = currentPose;
    end
    
    %Compute state
    %x, y, th, xp, yp, thp in global coordinates
    x = currentPose(1); y = currentPose(2); th = currentPose(3); dist = 0;%State variables
    xp = curgoalpose(1); yp = curgoalpose(2); thp = curgoalpose(3); %Predicted state
    
    %3D PID gains
    kcoeff = 1; %Global scaling factor for PID. 0 for no PID
    kx = 3;
    ky = 30;
    kt = 3;
    kdx = 0.1;
    kix = 0.001;
    kdy = 3;
    kiy = .3;
    kdt = 0.1;
    kit = 0.01;

    badenccount = 0;
    consecbadenccount = 0;
    loopcount = 0;

    %sgn is direction of driving (forward or backward). Using 1 (forward)
    curve = cubicSpiral.planTrajectory(xtarget, ytarget, thtarget, 1);
    
    %Convert poseArray into global coordinates (V and w same in both)
    temp = ones(size(curve.poseArray));
    temp(1:2, :) = curve.poseArray(1:2, :);
    temppose = pose(curgoalpose(1), curgoalpose(2), curgoalpose(3)); %Using their pose class to convert the intended trajectory to global coordinates
    temp = bToA(temppose)*temp;
    temp(3, :) = curve.poseArray(3, :) + curgoalpose(3); %theta's not in the homogeneous transform; do it separately
    curve.poseArray = temp;
    
    planVelocities(curve, vMax); %Generate the rest of the data for the curve
    tend = 0; %Time spent so far servoing at the end
    
    t = -0.1; %t starts at -0.1 to compensate for my first encoder reading
    if(size(data) == 0)
        data = [x; y; th; t]; %Where we are
        pdata = [xp; yp; thp; t]; %Where we think we are
    end
    edata = [0; 0; 0; t]; %Difference between last time step and this one (initial value doesn't matter; at each step we'll use the new value)
    cumerror = [0; 0; 0];
    oldenc = getEncoders();
    pause(0.1); %Allow another set of encoder readings to come in for ease of math

    while true  
       loopcount = loopcount + 1;
       newenc = getEncoders(); %Ideally would have a clever wait here
       %Compute dt and relevant speeds/distances
       dt = newenc(3)-oldenc(3);
       t = t + dt;
       if dt == 0
           'No new encoder data'
           badenccount = badenccount + 1
           consecbadenccount = consecbadenccount + 1
           pause(0.05);
           continue
       end
       if norm(newenc-oldenc) > 0.5 & consecbadenccount < 3 & loopcount > 5
           'Bad encoder data?'
           newenc-oldenc
           loopcount
           badenccount = badenccount + 1
           consecbadenccount = consecbadenccount + 1
           pause(0.05)
           continue
       end
       consecbadenccount = 0;
       %'Yay, new encoder data!'
       %These are measured vl and vr; commanded ones handled by FKmove()
       vl = (newenc(1)-oldenc(1))/dt;
       vr = (newenc(2)-oldenc(2))/dt;
       [Vmeas, wmeas] = IKcomp(vl, vr);
       %Update state estimate (using midpoint method; could use Runge-Kutta, etc.)
       th = th + wmeas*dt/2;
       x = x + Vmeas * cos(th)*dt;
       y = y + Vmeas * sin(th)*dt;
       th = th + wmeas*dt/2;
       dist = dist + abs(Vmeas)*dt;
       temp = [x; y; th; t];
       data = [data, temp];
       
       dmax = max(curve.distArray);
       if dist >= dmax
           %Set target position to end of trajectory, increment end timer,
           %and turn off feedforward commands
           xp = curve.poseArray(1, end);
           yp = curve.poseArray(2, end);
           thp = curve.poseArray(3, end);
           vff = 0;
           wff = 0;
           tend = tend + dt;
       else   
           %Helper variables for linear interpolation
           dprev = max(curve.distArray(curve.distArray <= dist));
           dnext = min(curve.distArray(curve.distArray > dist));
           iprev = find(curve.distArray == dprev, 1);
           inext = find(curve.distArray == dnext, 1);
           prop = (dist - dprev)/(dnext-dprev);
           
           %Actually do the linear interpolation for feedforward
           vprev = curve.VArray(iprev);
           vnext = curve.VArray(inext);
           vff = prop*vprev + (1-prop)*vnext;
           wprev = curve.wArray(iprev);
           wnext = curve.wArray(inext);
           wff = prop*wprev + (1-prop)*wnext;
           
           %Interpolate predicted location
           xp = prop*curve.poseArray(1, iprev) + (1-prop)*curve.poseArray(1, inext);
           yp = prop*curve.poseArray(2, iprev) + (1-prop)*curve.poseArray(2, inext);
           thp = prop*curve.poseArray(3, iprev) + (1-prop)*curve.poseArray(3, inext);
           tempp = [xp; yp; thp; t];
           pdata = [pdata, tempp];
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
       figure(2);
       plot(data(1,:), data(2,:));
       hold on
       plot(pdata(1, :), pdata(2, :));
       hold off
       %quiver(x, y, Vmeas*cos(th), Vmeas*sin(th));
       axis([-1 1 -1 1]);
       title('Robot Trajectory');
       xlabel('x (m)');
       ylabel('y (m)');
       %Update encoder buffer
       oldenc = newenc;
       %Pause so Matlab doesn't hate me...
       pause(0.05);
       %Check for exit
       if tend >= 1
           move(0, 0); %Stop
           break
       end
    end
    
    %Plot final trajectory on sail graph
    close(2);
    figure(1);
    hold on
    plot(data(1,:), data(2,:));
    plot(pdata(1, :), pdata(2, :));
    axis([-2, 2, -2, 2]);
    hold off
    %quiver(x, y, Vmeas*cos(th), Vmeas*sin(th));
    title('Robot Trajectory');
    xlabel('x (m)');
    ylabel('y (m)');

    goalposeout = curve.poseArray(:, end);
    poseout = [x; y; th];
end

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

%Copy/pasted from Lab 1 final version (then cleaned up):

%Wrapper for reading encoders
%Returns a length 2 vector
%Actual values if not simulating; simulated values otherwise
function e = getEncoders()
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove, global data, global origEnc
    if ~isSim
        tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
        e = [robot.encoders.LatestMessage.Vector.X; robot.encoders.LatestMessage.Vector.Y; tstamp];
    else
        e = [simle; simre; toc(tStart)];
    end
end

%Wrapper for setVelocity
%Sets simulated values and sends commands to the robot
function move(lv, rv)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vMax, global robot, global tStart, global tMove, global data, global origEnc, global simencnoise

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
    ld = simlv*dt * (rand()*2*simencnoise - simencnoise + 1);
    rd = simrv*dt * (rand()*2*simencnoise - simencnoise + 1);
    simle = simle + ld; %Simulate encoder values forward
    simre = simre + rd;
    simlv = lv;
    simrv = rv;
    tMove = tic;
end

%Wrapper for forks
function forksUp()
    global robot, global isSim;
    if ~isSim
        robot.forksUp()
    else
        'Raising forks'
    end
end

%Wrapper for forks
function forksDown()
    global robot, global isSim;
    if ~isSim
        robot.forksDown()
    else
        'Lowering forks'
    end
end