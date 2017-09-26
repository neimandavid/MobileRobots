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
kp = 1.5;
ki = 1;
kd = 0.1;
kcoeff = 1;

global data, global vpid, global dtarget
data = [];
dtarget = [];
vpid = [-1, -1];

PIDFFmove(1, 0.25, 0.33);
move(0, 0);


plot(data(3, :), data(1, :)-dtarget);
xlabel('Time (s)')
ylabel('Error (m)')
title('Errors')
legend('Error')

figure;
plot(data(3, :), data(1, :));
hold on
plot(data(3, :), dtarget);
hold off
xlabel('Time (s)')
ylabel('Distance (m)')
title('Calculated and Target Distances')
legend('Actual distance', 'Target distance')

move(0, 0);

'Finished moving'

function PIDFFmove(d, v, tr)
    global data, global vpid, global kp, global ki, global kd, global kcoeff, global dtarget
    a = v/tr;
    tf = d/v+tr;
    temp = getEncoders();
    tstart = temp;
    data = [0; 0; 0];
    t = 0;
    dexp = [0, 0];
    dtarget = [dtarget, dexp(1)];
    olderr = [0, 0];
    cumerr = [0, 0];
    derr = [0, 0];
    while t < tr
        dexp = [0.5*a*t^2, 0.5*a*t^2];
        dtarget = [dtarget, dexp(1)];
        temp = getEncoders()-tstart;
        data = [data, temp];
        oldt = t;
        t = temp(3);
        dt = t - oldt;
        err = [dexp - (temp(1:2))'];
        if dt < 0
            t = oldt;
            continue
        end
        if dt > 0
            derr = (err - olderr)/dt;
        end
        if abs(derr(1)) > 1
            'Warning: Error changed a lot in one time step';
            derr(1) = 1*sign(derr(1));
        end
        if abs(derr(2)) > 1
            'Warning: Error changed a lot in one time step';
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
        dtarget = [dtarget, dexp(1)];
        temp = getEncoders()-tstart;
        data = [data, temp];
        oldt = t;
        t = temp(3);
        dt = t - oldt;
        err = [dexp - (temp(1:2))'];
        if dt < 0
            t = oldt;
            continue
        end
        if dt > 0
            derr = (err - olderr)/dt;
        end
        err = [dexp - (temp(1:2))'];
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
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr)
        vff = [v, v];
        vcomm = vpid + vff;
        move(vcomm(1), vcomm(2));
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    while t < tf
        dexp = [0.5*a*tr^2 + v*(t-tr) - 0.5*a*(t+(tr-tf))^2, 0.5*a*tr^2 + v*(t-tr) - 0.5*a*(t+tr-tf)^2];
        dtarget = [dtarget, dexp(1)];
        temp = getEncoders()-tstart;
        data = [data, temp];
        oldt = t;
        t = temp(3);
        dt = t - oldt;
        err = [dexp - (temp(1:2))'];
        if dt < 0
            t = oldt;
            continue
        end
        if dt > 0
            derr = (err - olderr)/dt;
        end
        err = [dexp - (temp(1:2))'];
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
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr)
        vff = [a*(tf-t), a*(tf-t)];
        vcomm = vpid + vff;
        move(vcomm(1), vcomm(2));
        pause(0.05);
        plot(data(3, :), data(1, :));
    end
    %Fourth loop added for that 1 second of sitting there adjusting
    while t < tf+1
        dexp = [d, d];
        dtarget = [dtarget, dexp(1)];
        temp = getEncoders()-tstart;
        data = [data, temp];
        oldt = t;
        t = temp(3);
        dt = t - oldt;
        err = [dexp - (temp(1:2))'];
        if dt < 0
            t = oldt;
            continue
        end
        if dt > 0
            derr = (err - olderr)/dt;
        end
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
        vpid = kcoeff*(kp*err + kd*derr + ki*cumerr)
        vff = [0, 0];
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
        e = [0.98*robot.encoders.LatestMessage.Vector.X; 0.98*robot.encoders.LatestMessage.Vector.Y; tstamp];
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