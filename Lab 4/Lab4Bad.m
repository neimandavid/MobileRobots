global robot, global isSim, global vMax, global tMove, global tStart, global wheelbase, global simle, global simre, global simlv, global simrv;
isSim = false;
if(~isSim)
    robot = raspbot();
end
simle = 0; simre = 0;
simlv = 0; simrv = 0;
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m

global kcoeff, global kp, global ki, global kd;
kp = 0.01;
ki = 0;
kd = 0;
kcoeff = 1;

global olderr, global cumerr
olderr = 0;
cumerr = 0;

global data, global origenc
data = [];
origenc = [];

tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

%PIDRamp(0, 0.1, 1);
PIDRamp(0.1, 0, 9);
%PIDRamp(0.1, -0.1, 1);
%PIDRamp(0, -0.1, 1);
%PIDRamp(-0.1, 0, 9);
%PIDRamp(-0.1, 0.1, 1);
move(0, 0);
'Done'

%Runs the robot, initial velocity v, acceleration a, for time t
function PIDRamp(v, a, tf)
    global olderr, global cumerr
    global kcoeff, global kp, global ki, global kd;
    
    t = 0;
    derr = 0;
    temp = getEncoders();
    encstart = temp;
    while t < tf
        %Get encoder data
        adjenc = getEncoders()-encstart;
        oldt = t;
        t = adjenc(3);
        dt = t - oldt;
        %Compute expected and current position
        dexp = v*t + a*t^2;
        dcurr = adjenc(1); %TODO: Do each component separately
        err = dexp-dcurr;
        if dt > 0
            derr = (olderr-err)/dt;
        end
        derr(abs(derr)>1) = derr(abs(derr)>1)/abs(derr(abs(derr)>1));
        cumerr = cumerr + err*dt;
        cumerr(abs(cumerr)>1) = cumerr(abs(cumerr)>1)/abs(cumerr(abs(cumerr)>1));
        %Plug in velocities and move
        vff = v + a*t;
        vpid = kcoeff*(kp*err + kd*derr +  ki*cumerr);
        vcomm = vff + vpid;
        move(vcomm, vcomm);
        pause(0.05);
        olderr = err;
    end
end

%Copy/pasted from lab 3

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
    global origenc
    if ~isSim
        tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
        e = [robot.encoders.LatestMessage.Vector.X; robot.encoders.LatestMessage.Vector.Y; tstamp];
    else
        e = [simle; simre; toc(tStart)];
    end
    if size(origenc,1) == 0
        origenc = e;
    end
    e = e - origenc;
    data = [data, e];
end

%Wrapper for setVelocity
%Sets simulated values and sends commands to the robot
function move(lv, rv)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vMax, global robot, global tStart, global tMove

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