close all

global robot, global isSim, global vMax, global tMove, global tStart, global wheelbase, global simle, global simre, global simlv, global simrv, global simencnoise;
isSim = false;
if(~isSim)
    robot = raspbot();
end
simle = 0; simre = 0;
simlv = 0; simrv = 0;
rng('shuffle') %Get more randomness on simulated encoder noise (without this it's just psuedorandom)
simencnoise = 0; %Fraction of velocity reading to randomly add/subtract as noise
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m

tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

global minrange, global maxrange;
minrange = 0.06;
maxrange = 3;

global walls; %Matrix, each row x1, y1, x2, y2 of one wall
walls = [0, 0, 96*0.0254, 0; 0, 0, 0, 144*0.0254; 96*0.0254, 0, 96*0.0254, 144*0.0254; 0, 144*0.0254, 96*0.0254, 144*0.0254];

ftinm = 12*0.0254; %1 foot in meters
pallets = [ftinm, 6*ftinm, pi/2, true; 2*ftinm, 6*ftinm, pi/2, true; 3*ftinm, 6*ftinm, pi/2, true; 4*ftinm, 6*ftinm, pi/2, true; 5*ftinm, 6*ftinm, pi/2, true; 6*ftinm, 6*ftinm, pi/2, true; 7*ftinm, 6*ftinm, pi/2, true; 7*ftinm, 4*ftinm, 0, true; 7*ftinm, 3*ftinm, 0, true; 7*ftinm, 2*ftinm, 0, true; 1*ftinm, 8*ftinm, pi, true; 1*ftinm, 9*ftinm, pi, true; 1*ftinm, 10*ftinm, pi, true];
global drops;
drops = [1*ftinm, 1*ftinm, -pi/2, true; 2*ftinm, 1*ftinm, -pi/2, true; 3*ftinm, 1*ftinm, -pi/2, true; 4*ftinm, 1*ftinm, -pi/2, true; 5*ftinm, 1*ftinm, -pi/2, true; 6*ftinm, 1*ftinm, -pi/2, true; 7*ftinm, 1*ftinm, -pi/2, true];
global data;
data = [];

%start = [0; 0; 0]; %Start position: [x, y, theta]
start = [ftinm; ftinm; -pi/2];
pose = start; %pose is where we actually are
goalpose = start; %goalpose is where we want to be

if ~isSim
    robot.startLaser();
    forksUp();
    pause(5);
    forksDown();
end

topick = [1, 2, 3, 4, 10, 9, 8];%[1, 2, 3, 4, 10, 9, 8];

for x = 1:7
    %if topick(x) == 1
        [pose, goalpose] = backThenTurn(0.1, 8*pi/9, 0.2, pose, goalpose);
    %else
    %    [pose, goalpose] = backThenTurn(0.1, pi, 0.2, pose, goalpose);
    %end

    %forksUp();
    %pause(5);
    %forksDown();
    pose = getLidarPose(pose)
    goalpose = pose;
    sd1 = 0.3; %First standoff distance
    [pose, goalpose] = moveAbsPos([pallets(topick(x), 1)-sd1*cos(pallets(topick(x), 3)), pallets(topick(x), 2)-sd1*sin(pallets(topick(x), 3)),pallets(topick(x), 3)], 0.4, pose, goalpose, 1);
    %[pose, goalpose] = goToSail(0.127, -0.3, 0.2, pose, goalpose, [90, 270]);
    %pose = getLidarPose(pose)
    %goalpose = pose;
    [pose, goalpose] = goToSail(0.127,-0.13, 0.10, pose, goalpose);
    %pose = getLidarPose(pose)
    %goalpose = pose;
    [pose, goalpose] = moveRelPos(.07, 0, 0, 0.05, pose, pose, 0); %No PID
    forksUp();
    %pause(3);
    %pose = getLidarPose(pose)
    %goalpose = pose;
    if topick(x) == 8 || topick(x) == 9 || topick(x) == 10
        [pose, goalpose] = backThenTurn(0.07, -8*pi/9, 0.05, pose, goalpose);
    else
        [pose, goalpose] = backThenTurn(0.07, pi, 0.05, pose, goalpose);
    end
    pose = getLidarPose(pose)
    goalpose = pose;
    %New code for drops
    %Compute drop spot (off by distance from sensor to forks
    dropindex = getClosestDropIndex(pose)
    dropspot = drops(dropindex, 1:3)';
    dropdist = 0.06; %Sensor to fork distance
    dropspot(1) = dropspot(1) - dropdist*cos(dropspot(3));
    dropspot(2) = dropspot(2) - dropdist*sin(dropspot(3));
    %pose = getLidarPose(pose)
    %goalpose = pose;
    %forksDown();
    [pose, goalpose] = moveAbsPos(dropspot, 0.2, pose, goalpose, 0);
    dropspot = drops(dropindex, 1:3)';
    dropdist = 0.06; %Sensor to fork distance
    dropspot(1) = dropspot(1) - dropdist*cos(dropspot(3));
    dropspot(2) = dropspot(2) - dropdist*sin(dropspot(3));
    %forksUp();
    
    %Dropoff correction code
%     pose = getLidarPose(pose)
%     goalpose = pose;
%     if(norm(pose - dropspot) > .075)
%         [pose, goalpose] = moveAbsPos(dropspot, 0.1, pose, goalpose, 1);
%     end
    
    forksDown();
    pause(0.5);
    %pose = getLidarPose(pose)
    %goalpose = pose;
end


robot.stopLaser();
'Done'

function minindex = getClosestDropIndex(pose)
    global drops;
    mindist = 10000000;
    minindex = 7;
    for x = 1:7
        if norm(pose(1:2)-drops(x, 1:2)') < mindist && drops(x, 4)
            mindist = norm(pose(1:2)-drops(x, 1:2)');
            minindex = x;
        end
    end
    drops(minindex, 4) = false;
    return;
end

function [poseout, goalposeout] = moveAbsPos(targetPose, vMax, currentPose, goalPose, kcoeff)
    th = -currentPose(3);
    R = [cos(th), -sin(th); sin(th), cos(th)];
    dx = targetPose(1) - currentPose(1);
    dy = targetPose(2) - currentPose(2);
    temp = R*[dx; dy];
    dx = temp(1); dy = temp(2);
    dth = targetPose(3) - currentPose(3);
    [poseout, goalposeout] = moveRelPos(dx, dy, dth, vMax, currentPose, currentPose, kcoeff);
end

function [pose, rangesout] = getLidarPose(pose)
    t1 = tic;
    eps = 10^-9;
    dpose = 10^-2;
    cutoff = dpose;
    ranges = getNiceRanges(-5);
    oldpx = 0;
    oldpy = 0;
    oldpth = 0;
    rangesout = [];
    while true
        px = (computeError(pose + [eps; 0; 0], ranges) - computeError(pose, ranges))/eps;
        py = (computeError(pose + [0; eps; 0], ranges) - computeError(pose, ranges))/eps;
        pth = 5*(computeError(pose + [0; 0; eps], ranges) - computeError(pose, ranges))/eps;
        [~, rangesout] = computeError(pose, ranges);
        grad = [px; py; pth];
        
        %Don't bother checking for small error; small gradient is enough
        %Don't scale the cutoff value with the step size or we're slow
        if norm(grad) < cutoff || computeError(pose, ranges) < cutoff || toc(t1) > 5
            break
        end
        
        %Update pose and repeat
        pose = pose - 10*dpose*grad;
        
        if (px*oldpx < -cutoff^2 || py*oldpy < -cutoff^2 || pth*oldpth < -cutoff^2)&& dpose > cutoff/10
            dpose = dpose/1.1;
        end
        
        oldpx = px;
        oldpy = py;
        oldpth = pth;
    end
    while pose(3) < -pi
        pose(3) = pose(3) + 2*pi;
    end
    while pose(3) > pi
        pose(3) = pose(3) - 2*pi;
    end
    pose = real(pose);
end

function [err, newRanges] = computeError(pose, ranges)
    global walls;
    if size(walls, 1) == 0
        'No walls; returning 0 for no good reason'
        err = 0;
        return;
    end
    
    if size(walls, 1) == 1
        'Only 1 wall; returning 0 for no good reason'
        err = 0;
        return;
    end
    
    %Convert walls into robot frame
    relWalls = zeros(size(walls));
    %H*p takes p from robot frame to world frame
    
    %Inverse rotation map
    R = [cos(-pose(3)), sin(pose(3)); -sin(pose(3)), cos(-pose(3))];
    d = [pose(1); pose(2)];
    di = -R*d; %Inverse
    
    %Forward map; I want inverse
    %H = [cos(-pose(3)), sin(-pose(3)), pose(1); -sin(-pose(3)), cos(-pose(3)), pose(2); 0, 0, 1];
    
    %Inverse map
    H = [R(1, 1), R(1, 2), di(1); R(2, 1), R(2, 2), di(2); 0, 0, 1]; %Inv
        
    for x = 1:size(walls, 1)
        tempmat = [walls(x, 1), walls(x, 3); walls(x, 2), walls(x, 4); 1, 1];
        M = H*tempmat;%inv(H)*tempmat;
        relWalls(x, :) = [M(1, 1), M(2, 1), M(1, 2), M(2, 2)];
    end
    
    m = size(ranges, 2);
    newRanges = zeros(6, m);
    for x = 1:m
        p = [ranges(3, x), ranges(4, x)];
        %Compute distance to closest two walls
        n = size(relWalls, 1); %number of walls
        dists = zeros(1, n);
        for y = 1:n
         dists(y) = distToSegment(p, relWalls(y, :));
        end
        dists = sort(dists);
        cdist = dists(1); %distance to closest wall
        distdiff = dists(2)-dists(1); %difference between two closest distances
        newRanges(:, x) = [ranges(:, x); cdist; distdiff];
    end
    
    %Parameters for throwing out garbage data
    walltol = 0.3; %Max allowed distance to wall
    cornertol = 0.02; %Min allowed distance difference (small indicates near corner)
   
    %Filter garbage data
    tempNewRanges = newRanges(:, (newRanges(5, :) < walltol & newRanges(6, :) > cornertol) );
    while (size(tempNewRanges, 2) == 0)
        walltol = walltol*1.1;
        cornertol = cornertol/1.1;
        tempNewRanges = newRanges(:, (newRanges(5, :) < walltol & newRanges(6, :) > cornertol) );
    end
    newRanges = tempNewRanges(:, 1:1:size(tempNewRanges, 2));
    err = sum(newRanges(5, :))/size(newRanges, 2); %Average smallest error
end
    
function [poseout, goalposeout] = backThenTurn(inputd, theta, vMax, startpose, initgoalpose)
    global wheelbase; global robot;
    
    t = 0;
    x = 0;
    y = 0;
    th = 0;
    
    x = startpose(1);
    y = startpose(2);
    th = startpose(3);
    
    badenccount = 0;
    consecbadenccount = 0;
    loopcount = 0;
    
    oldenc = getEncoders();
    %pause(1);
    
    %Back
    while t < inputd/vMax
        %Send move command for backwards
       FKmove(-vMax, 0);
       
       newenc = getEncoders(); %Ideally would have a clever wait here
       %Compute dt and relevant speeds/distances
       dt = newenc(3)-oldenc(3);
       if dt == 0
           %'No new encoder data'
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05);
           continue
       end
       if norm(newenc-oldenc) > 0.5 && consecbadenccount < 3 && loopcount > 5
           %'Bad encoder data?'
           %newenc-oldenc
           %loopcount
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05)
           continue
       end
       t = t + dt;
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
       pause(0.05);
       oldenc = newenc;
    end
    %Turn
    t = 0;
    while t < theta/(vMax*2/wheelbase)
        %Send move command for turn
       FKmove(0, vMax*2/wheelbase);
       
       newenc = getEncoders(); %Ideally would have a clever wait here
       %Compute dt and relevant speeds/distances
       dt = newenc(3)-oldenc(3);
       if dt == 0
           %'No new encoder data'
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05);
           continue
       end
       if norm(newenc-oldenc) > 0.5 && consecbadenccount < 3 && loopcount > 5
           %'Bad encoder data?'
           %newenc-oldenc
           %loopcount
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05);
           continue
       end
       t = t + dt;
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
       pause(0.05);
       oldenc = newenc;
    end
    robot.stop();
    
    pause(0.2);
    newenc = getEncoders(); %Ideally would have a clever wait here
    %Compute dt and relevant speeds/distances
    dt = newenc(3)-oldenc(3);
    t = t + dt;
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
    pause(0.05);
    oldenc = newenc;
    
    goalposeout = initgoalpose;
    goalposeout(1) = goalposeout(1)-inputd*cos(goalposeout(3));
    goalposeout(2) = goalposeout(2)-inputd*sin(goalposeout(3));
    goalposeout(3) = goalposeout(3) + theta;
    poseout = [x; y; th];
    while poseout(3) < -pi
        poseout(3) = poseout(3) + 2*pi;
    end
    while poseout(3) > pi
        poseout(3) = poseout(3) - 2*pi;
    end
    %poseout = startpose;
    %poseout(1) = poseout(1) + x*cos(poseout(3)) - y*sin(poseout(3));
    %poseout(2) = poseout(2) + x*sin(poseout(3)) + y*cos(poseout(3));
    %poseout(3) = poseout(3) + th;
end

%LIDAR code (partly from lab 2)
function [poseout, goalposeout] = goToSail(sailwidth, standoffdist, vMax, startpose, initialgoalpose, badrange)
    if nargin < 6
        badrange = [180, 180]; %Allow all sails
    end
    initialgoalpose = startpose; %Don't do silly stuff; use the lidar
    
    close all;
    pause(1);
    rangeData = getNiceRanges(-5); %Positive makes the robot go left

    close all
    plot(rangeData(3, :), rangeData(4, :), 'o') %x is forward, y is left
    [sailpos, dist] = findSail2(rangeData, sailwidth, badrange);
    if sailpos == [0; 0; 0]
        'No sail found'
        poseout = startpose;
        goalposeout = initialgoalpose;
    elseif (dist > 0.35 && standoffdist > -0.3) || (dist < 0.15 && standoffdist < -0.15) %Retry if too far or close
        goToSail(sailwidth, -0.3, 0.2, startpose, initialgoalpose, badrange)
        [poseout, goalposeout] = goToSail(sailwidth, standoffdist, vMax, startpose, initialgoalpose, badrange);
    else
        [poseout, goalposeout] = moveRelPos(sailpos(1)+standoffdist*cos(sailpos(3)), sailpos(2)+standoffdist*sin(sailpos(3)), sailpos(3), vMax, startpose, initialgoalpose);
        hold on
        plot(sailpos(1), sailpos(2), 'x', 'MarkerSize', 100)
        hold off
    end    
end

function [cdata, d] = findSail2(ranges, sailwidth, badrange)
    if nargin < 3
        badrange = [180, 180];
    end
    debug = false;

    numpts = size(ranges, 2);
    d = Inf; %Shortest distance to a sail
    xc = 0; yc = 0; thc = 0; %Relative location of sail
    
    x = 1;
    
    while x < 2*numpts
       startpt = rangeValue(x, ranges);
       oldpt = startpt;
       %Find a proposed line
       cumx = startpt(3);
       cumy = startpt(4);
       linepts = [startpt];
       for y = 1:numpts
          newpt = rangeValue(x+y, ranges);
          if norm(newpt(3:4)-oldpt(3:4)) > 0.1
              break
              %Found a 5 cm gap
          end
          if mod(newpt(2)-startpt(2), 360) > 180 
               break
          end
          cumx = cumx + newpt(3);
          cumy = cumy + newpt(4);
          linepts = [linepts, newpt];
          oldpt = newpt;
       end
       %Check if sail
       if size(linepts, 2) < 3
           if debug
               'Not enough points'
           end
           x = x+y;
           continue;
       end
       if norm(startpt(3:4)-oldpt(3:4)) < .6*sailwidth
          if debug
              'Too small'
          end
          x = x+y;
          continue;
       end
       if norm(startpt(3:4)-oldpt(3:4)) > 1.4*sailwidth
          if debug
              'Too big'
          end
          x = x+y;
          continue;
       end
       if x == 1
           if debug
               'First object; ignoring'
           end
           x = x+y;
           continue;
       end
       %Add check to ignore sails behind us (for right after a drop)
       if badrange(1) < startpt(2) && startpt(2) < badrange(2)
           if debug
               'Sail starts within bad range'
           end
           startpt
           x = x+y;
           continue;
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
       th = thd;
       if lambda(1) > 10
           if debug
                'Not a line'
           end
           x = x+y;
           continue;
       end
       
       %It's a sail
       if debug
           'Found a sail'
       end
       
       %Plot "sails"
       hold on
       xplot = [startpt(3), oldpt(3)];
       yplot = [startpt(4), oldpt(4)];
       plot(xplot, yplot, '-o');
       quiver(xbar, ybar, cos(th), sin(th));
       hold off
       
       thdirect = atan2(ybar, xbar);
       newd = norm([xbar, ybar]);
       
       if newd < d
           d = newd;
           xc = xbar;
           yc = ybar;
           thc = th;
       end
       
       x = x+y;
    end
    hold on
    plot(ranges(3, :), ranges(4, :), 'x');
    hold off
    cdata = [xc; yc; thc];
end

function out = rangeValue(index, ranges)
    out = ranges(:, mod(index-1, size(ranges, 2)) + 1);
end

function niceRanges = getNiceRanges(fudge)
    %If no fudge given assume 0
    if nargin < 1
        fudge = 0;
    end

    %If simulating, return whatever we have stored
    global isSim;
    if isSim
        load('rangeData.mat');
        return;
    end
    
    %Else actually compute things
    global walls, global pose;
    %r, theta, x, y
    ranges = getRanges(fudge);
    niceRanges = zeros(4, size(ranges(~isnan(ranges)), 1));
    count = 1;
    for x = 1:360
       if ~isnan(ranges(x))
          niceRanges(:, count) = [ranges(x); x; ranges(x)*cosd(x); ranges(x)*sind(x)];
          count = count + 1;
       end
       save('rangeData.mat', 'niceRanges'); %Store our last set of range data
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
function [poseout, goalposeout] = moveRelPos(xtarget, ytarget, thtarget, vMax, currentPose, curgoalpose, kcoeff)
    relposclock = tic();
    if nargin < 7
        kcoeff = 1;
    end
    global data;
    global pdata;
    %currentPose and curgoalpose are in global coordinates (else currentPose = 0 all the time...)
    
    %Default argument values for convenience
    if nargin <= 3
        vMax = 0.2;
    end
    if nargin <= 4
        currentPose = [0; 0; 0];
    end
    if nargin <= 5
        curgoalpose = currentPose;
    end
    
    %Compute state
    %x, y, th, xp, yp, thp in global coordinates
    x = currentPose(1); y = currentPose(2); th = currentPose(3); dist = 0;%State variables
    xp = curgoalpose(1); yp = curgoalpose(2); thp = curgoalpose(3); %Predicted state
    
    %3D PID gains
    %kcoeff = 1; %Global scaling factor for PID. 0 for no PID. Now an
    %argument
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
       if(toc(relposclock) > 20) %20 second timeout
           'Timed out'
           poseout = getLidarPose([x; y; th]);
           goalposeout = poseout;
           return;
       end
       loopcount = loopcount + 1;
       newenc = getEncoders(); %Ideally would have a clever wait here
       %Compute dt and relevant speeds/distances
       dt = newenc(3)-oldenc(3);
       t = t + dt;
       if dt == 0
           %'No new encoder data'
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05);
           continue
       end
       if norm(newenc-oldenc) > 0.5 && consecbadenccount < 3 && loopcount > 5
           %'Bad encoder data?'
           %newenc-oldenc
           %loopcount
           badenccount = badenccount + 1;
           consecbadenccount = consecbadenccount + 1;
           pause(0.05);
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
           if isempty(dprev) || isempty(dnext)
               'Think we got a weird case...'
               continue
           end
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
%        figure(2);
%        plot(data(1,:), data(2,:));
%        hold on
%        plot(pdata(1, :), pdata(2, :));
%        hold off
%        %quiver(x, y, Vmeas*cos(th), Vmeas*sin(th));
%        axis([-1 1 -1 1]);
%        title('Robot Trajectory');
%        xlabel('x (m)');
%        ylabel('y (m)');
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
    %close(2);
%     figure(1);
%     hold on
%     plot(data(1,:), data(2,:));
%     plot(pdata(1, :), pdata(2, :));
%     axis([-2, 2, -2, 2]);
%     hold off
%     %quiver(x, y, Vmeas*cos(th), Vmeas*sin(th));
%     title('Robot Trajectory');
%     xlabel('x (m)');
%     ylabel('y (m)');

    goalposeout = curve.poseArray(:, end);
    poseout = [x; y; th];
    while poseout(3) < -pi
        poseout(3) = poseout(3) + 2*pi;
    end
    while poseout(3) > pi
        poseout(3) = poseout(3) - 2*pi;
    end
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

%p a point (x, y); s a vector [x1, y1, x2, y2]
function d = distToSegment(p, s)
    p1 = s(1:2);
    p2 = s(3:4);
    
    %Check if off first endpoint
    u = p2-p1;
    v = p-p1;
    if dot(u, v) < 0
       d = ((p(1)-p1(1))^2 + (p(2)-p1(2))^2)^0.5;
       return;
    end
    
    %Check if off second endpoint
    u = p1-p2;
    w = p-p2;
    if dot(u, w) < 0
       d = ((p(1)-p2(1))^2 + (p(2)-p2(2))^2)^0.5;
       return;
    end
    
    %u, v, w are sides of a triangle; use Heron's formula to get altitude
    a = norm(u); b = norm(v); c = norm(w); S = 0.5*(a+b+c);
    d = (S*(S-a)*(S-b)*(S-c))^0.5 / (0.5*a);
end