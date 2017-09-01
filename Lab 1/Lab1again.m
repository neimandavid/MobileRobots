%%Setup
global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove, global data, global origEnc
isSim = false;
simlv = 0; %Left velocity
simrv = 0; %Right velocity
simle = 1234; %Encoder values (random to start)
simre = 5678;
origEnc = getEncoders();
simpos = [0; 0; 0]; %x y theta relative to start
vmax = 0.5;

%2.54 cm = 1 in (exact conversion)

if ~isSim
    robot = raspbot()
end
tStart = tic; %Timer that never stops
tMove = tic; %Timer that's reset each call to the move function

%%Do challenge course
data = [];
startEnc = getEncoders;
tMove = tic; %Timer that's reset each call to the move function

hold all
moveDist(0.04, 0.3048);
pause(1);
moveDist(0.04, -0.3048);
hold off

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
function dt = move(lv, rv)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove, global data, global origEnc

    %Cap the speeds
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
        rv = -vmax;
    end
    
    if ~isSim
        sendVelocity(robot, lv, rv);
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

function moveDist(v, d)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove, global data, global origEnc
    startenc = getEncoders();
    currenc = 0;
    if(d > 0)
        while(currenc < d)
           move(v, v);
           
           pause(0.1);
           
           %Update encoder values
           temp = getEncoders();
           currenc = temp-startenc;
           
           %Plot things
           data = [data, [temp(1)-origEnc(1); temp(2)-origEnc(2); toc(tStart)]];
           %data is stored in meters; convert to cm when plotting
           plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
           xlabel = 'Time (s)';
           ylabel = 'Encoder distance (cm)';
           legend('leftEnc', 'rightEnc');
        end
    else
        while(currenc > d)
           move(-v, -v);
           
           pause(0.1);
           
           %Update encoder values
           temp = getEncoders();
           currenc = temp-startenc;
           
           %Plot things
           data = [data, [temp(1)-origEnc(1); temp(2)-origEnc(2); toc(tStart)]];
           %data is stored in meters; convert to cm when plotting
           plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
           xlabel = 'Time (s)';
           ylabel = 'Encoder distance (cm)';
           legend('leftEnc', 'rightEnc');
        end
    end
    move(0, 0);
    
    %Update encoder values (move is a step behind to compute distances)
    temp = getEncoders();
    currenc = temp-startenc;
    
   %Plot things
   data = [data, [temp(1)-origEnc(1); temp(2)-origEnc(2); toc(tStart)]];
   %data is stored in meters; convert to cm when plotting
   plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
   xlabel = 'Time (s)';
   ylabel = 'Encoder distance (cm)';
   legend('leftEnc', 'rightEnc');
end