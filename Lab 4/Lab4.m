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

global kp, global ki, global kd, global kcoeff
kp = 1;
ki = 0;
kd = 0.1;
kcoeff = 1;

echo on

global data, global vpid
vpid = [-1, -1];

PIDFFmove(1, 0.1, 1);
'Finished moving'


function FFmove(d, v, tr)
    global data
    a = v/tr;
    tf = d/v+tr;
    temp = getEncoders();
    tstart = temp(3);
    data = [0; 0; tstart];
    t = 0;
    while t < tr
        move(a*t, a*t);
        temp = getEncoders();
        data = [data, temp];
        t = temp(3) - tstart;
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    while t < tf - tr
        move(v, v);
        temp = getEncoders();
        data = [data, temp];
        t = temp(3) - tstart;
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    while t < tf
        move(a*(tf-t), a*(tf-t));
        temp = getEncoders();
        data = [data, temp];
        t = temp(3) - tstart;
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
end

function PIDFFmove(d, v, tr)
    global data, global vpid, global kp, global ki, global kd, global kcoeff
    a = v/tr;
    tf = d/v+tr;
    temp = getEncoders();
    tstart = temp(3);
    data = [0; 0; tstart];
    t = 0;
    dexp = [0, 0];
    olderr = [0, 0];
    cumerr = [0, 0];
    while t < tr
        dexp = [0.5*a*t^2, 0.5*a*t^2];
        temp = getEncoders();
        data = [data, temp];
        oldt = t;
        t = temp(3) - tstart;
        dt = t - oldt;
        err = [dexp - temp(1:2)];
        derr = (err - olderr)/dt;
        if abs(derr(1)) > 1
            'Warning: Error changed a lot in one time step'
            derr(1) = 1*sign(derr(1));
        end
        if abs(derr(2)) > 1
            'Warning: Error changed a lot in one time step'
            derr(2) = 1*sign(derr(2));
        end
        cumerr = cumerr + err*dt;
        if abs(cumerr(1)) > 10
            'Warning: Possible integral windup'
            cumerr(1) = 10*sign(cumerr(1));
        end
        if abs(cumerr(2)) > 10
            'Warning: Possible integral windup'
            cumerr(2) = 10*sign(cumerr(2));
        end
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr);
        vff = [a*t, a*t];
        vcomm = vpid + vff;
        move(vcomm(1), vcomm(2));
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    while t < tf - tr
        dexp = [0.5*a*tr^2 + v*(t-tr), 0.5*a*tr^2 + v*(t-tr)];
        temp = getEncoders();
        data = [data, temp];
        oldt = t;
        t = temp(3) - tstart;
        dt = t - oldt;
        err = [dexp - temp(1:2)];
        derr = (err - olderr)/dt;
        if abs(derr(1)) > 1
            'Warning: Error changed a lot in one time step'
            derr(1) = 1*sign(derr(1));
        end
        if abs(derr(2)) > 1
            'Warning: Error changed a lot in one time step'
            derr(2) = 1*sign(derr(2));
        end
        cumerr = cumerr + err*dt;
        if abs(cumerr(1)) > 10
            'Warning: Possible integral windup'
            cumerr(1) = 10*sign(cumerr(1));
        end
        if abs(cumerr(2)) > 10
            'Warning: Possible integral windup'
            cumerr(2) = 10*sign(cumerr(2));
        end
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr);
        vff = [v-1, v-1];
        vcomm = vpid + vff;
        move(vcomm(1), vcomm(2));
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    while t < tf
        dexp = [0.5*a*tr^2 + v*(t-tr) - 0.5*a*(t+tr-tf)^2, 0.5*a*tr^2 + v*(t-tr) - 0.5*a*(t+tr-tf)^2];
        temp = getEncoders();
        data = [data, temp];
        oldt = t;
        t = temp(3) - tstart;
        dt = t - oldt;
        err = [dexp - temp(1:2)];
        derr = (err - olderr)/dt;
        if abs(derr(1)) > 1
            'Warning: Error changed a lot in one time step'
            derr(1) = 1*sign(derr(1));
        end
        if abs(derr(2)) > 1
            'Warning: Error changed a lot in one time step'
            derr(2) = 1*sign(derr(2));
        end
        cumerr = cumerr + err*dt;
        if abs(cumerr(1)) > 10
            'Warning: Possible integral windup'
            cumerr(1) = 10*sign(cumerr(1));
        end
        if abs(cumerr(2)) > 10
            'Warning: Possible integral windup'
            cumerr(2) = 10*sign(cumerr(2));
        end
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr);
        vff = [a*(tf-t), a*(tf-t)];
        vcomm = vpid + vff;
        move(vcomm(1), vcomm(2));
        pause(0.05);
        plot(data(3, :), data(1, :));
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