global simlv, global simrv, global simpos, global isSim, global vmax, global robot
isSim = false;
simlv = 0; %Left velocity
simrv = 0; %Right velocity
simpos = [0; 0; 0]; %x y theta relative to start
vmax = 0.5;

setup();


tdata = [];
for x = 1:10
    tdata = [tdata, moveDist(0.04, 0.3)]
end

function setup()
    global simlv, global simrv, global simpos, global isSim, global vmax, global robot

    if ~isSim
        robot = raspbot();
        'Real robot'
    else
        'Just simulating'
    end
end

function move(lv, rv)
    global simlv, global simrv, global isSim, global vmax, global robot

    if lv > vmax
        lv = vmax;
    end
    if lv < -vmax
        lv = -vmax;
    end
    if rv > vmax
        rv = vmax;
    end
    if rv < -vmax
        rv = vmax;
    end
    
    if ~isSim
        sendVelocity(robot, lv, rv);
    end
    simlv = lv;
    simrv = rv;
end

function goSpeedForTime(s, t)
    global simlv, global simrv, global simpos, global isSim, global vmax, global robot

    startpos = simpos;
    timearray = [];
    posarray = [];
    tic;
    while toc < t
       move(s, s);
       pause(0.005);
       simpos = startpos + [s*toc*1000*cos(simpos(3)); s*toc*1000*sin(simpos(3)); 0];
       timearray = [timearray, toc];
       posarray = [posarray, simpos];
       toc
    end
    move(0, 0);
    size(timearray)
    size(posarray)
    plot(timearray, posarray(1, :))
end

function e = getEncoders()
    global simlv, global simrv, global simpos, global isSim, global vmax, global robot

    e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y];
end

%dist in meters
function currenc = moveDist(v, d)
    global simlv, global simrv, global simpos, global isSim, global vmax, global robot

    temp = getEncoders();
    startenc = temp(1);
    currenc = 0;
    if(d > 0)
        while(currenc < d) %3s overshoot
           %Implement something clever, possibly PID
           move(v, v);
           pause(0.1);
           temp = getEncoders();
           currenc = temp(1)-startenc;
        end
    else
        while(currenc > d)
           %Implement something clever, possibly PID
           temp = getEncoders();
           currenc = temp(1)-startenc;
           move(-v, -v);
           pause(0.1);
        end
    end
    move(0, 0);
    pause(5);
    temp = getEncoders();
    currenc = temp(1)-startenc;
end