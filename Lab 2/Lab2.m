global robot, global isSim, global vMax, global tMove, global wheelbase, global simle, global simre;
isSim = false;
if(~isSim)
    robot = raspbot();
    robot.startLaser();
    pause(1);
end
simle = 0;
simre = 0;
vMax = 0.5; %0.5 m/s
wheelbase = 0.09; %0.09 m
kpl = 1; %Go s.t. you reach the target pt. in 2s
kpa = 2*pi/180; %Reach target angle in 2s (convert to degrees first!)
%kpa is bugged...

data = [];
tStart = tic; %Start timer for everything
tMove = tic; %Start timer for simulated move function

hold all
while true
   temp = findClosest()
   if isnan(temp(1))
       move(0, 0)
   else
       smartMove(kpl*(temp(1)-1), kpa*(temp(2)-90));
   end
   tempenc = getEncoders();
   data = [data, [tempenc(1); tempenc(2); toc(tStart)]];
   clf;
   plot(data(3,:), data(1,:), data(3, :), data(2, :));
   xlabel ('Time (s)')
   ylabel ('Encoder distance (m)')
   legend('leftEnc', 'rightEnc')
   pause(0.05);
end

%Given a linear velocity V (m/s) and an angular velocity w (rad/s)
%compute and send the corresponding wheel velocities
function smartMove(V, w)
    global wheelbase;
    lv = V - w/2*wheelbase;
    rv = V + w/2*wheelbase;
    move(lv, rv);
end

function out = findClosest()
    ranges = getRanges();
    [r th] = min(ranges(1:180));
    out = [r, th];
end

%Sets data outside of [0.06, 2] to 123456789, which is clearly garbage data
%Considering setting it to its old value instead, but that might be bad
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
        for x = 1:360
            ranges(x) = 0.07; %Bigger than minimum range
        end
        ranges(90) = 1;
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
        e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y];
    else
        e = [simle, simre];
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
        robot.sendVelocity(rv, lv); %Flipping order because it works for now
        %Let's see what they do about that, but leaving it for now...
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