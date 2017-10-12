global robot, global isSim, global vMax, global tMove, global tStart, global wheelbase, global simle, global simre, global simlv, global simrv;
isSim = true;
if(~isSim)
    robot = raspbot();
end
simle = 0; simre = 0;
simlv = 0; simrv = 0;
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m

tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

start = [0; 0; pi/4]; %Start position: [x, y, theta]
pose = start;

close all

% t = 0;
% dt = 0.05;
% startEnc = getEncoders();
% data = [0; 0; 0];
% while t < 5
%     temp = getEncoders();
%     data = [data, temp-startEnc];
%     move(0.2, 0.2);
%     t = t + dt;
%     pause(dt);
% end
% t = 0;
% while t < 5
%     temp = getEncoders();
%     data = [data, temp-startEnc];
%     move(-0.2, -0.2);
%     t = t + dt;
%     pause(dt);
% end
% robot.stop;
% 'Stopped'
% plot(data(3, :), data(1, :));

pose = moveRelPos(0.3048, 0.3048, 0, 0.2, pose);
pose = moveRelPos(-0.6096, -0.6096, -pi/2, 0.2, pose);
pose = moveRelPos(-0.3048, 0.3048, pi/2, 0.2, pose);

function poseout = moveRelPos(xtarget, ytarget, thtarget, vMax, currentPose)
    if nargin == 4
        currentPose = [0, 0, 0];
    end
    temppose = pose(currentPose(1), currentPose(2), currentPose(3));
    x = currentPose(1); y = currentPose(2); th = currentPose(3); dist = 0;%State variables
    xp = 0; yp = 0; thp = currentPose(3); %Predicted state
    t = -0.1; %t starts at -0.1 to compensate for my first encoder reading
    data = [x; y; th; t]; %Where we are
    pdata = [x; y; th; t]; %Where we think we are
    edata = [0; 0; 0; t]; %Difference between last time step and this one
    cumerror = [0; 0; 0];
    oldenc = getEncoders();
    pause(0.1); %Allow another set of encoder readings to come in for ease of math

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
    
    %Convert poseArray into [0, 0, 0] coordinates
    temp = ones(size(curve.poseArray));
    temp(1:2, :) = curve.poseArray(1:2, :);
    temp = bToA(temppose)*temp;
    temp(3, :) = curve.poseArray(3, :) + currentPose(3);
    curve.poseArray = temp;
    
    planVelocities(curve, vMax);
    tend = 0;
    
    figure;

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
       if dist > dmax
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
    

%     figure;
%     plot(edata(4, :), edata(1, :));
%     hold on
%     plot(edata(4, :), edata(2, :));
%     plot(edata(4, :), edata(3, :));
%     title('Errors');
%     xlabel('Time (s)');
%     ylabel('Error (m or rad)');
%     legend('x', 'y', 'theta');
%     hold off

    %figure;
    reset(gca)
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
    plot(data(4, :), data(1, :));
    plot(pdata(4, :), pdata(1, :));
    legend('Actual', 'Predicted');
    hold off

%     figure;
%     hold on
%     for index = 1:3
%         plot(data(4, :), data(index, :));
%         plot(pdata(4, :), pdata(index, :));
%     end
%     title('Stuff vs. Time');
%     xlabel('Time (s)');
%     ylabel('Parameter (m or rad)');
%     legend('x', 'x predicted', 'y', 'y predicted', 'theta', 'theta predicted');
%     hold off
    
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
        e = [robot.encoders.LatestMessage.Vector.X; robot.encoders.LatestMessage.Vector.Y; tstamp];
    else
        e = [simle; simre; toc(tStart)];
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