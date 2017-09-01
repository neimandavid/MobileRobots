%%Setup
global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove
isSim = true;
simlv = 0; %Left velocity
simrv = 0; %Right velocity
simle = 1234; %Encoder values (random to start)
simre = 5678;
simpos = [0; 0; 0]; %x y theta relative to start
vmax = 0.5;

%2.54 cm = 1 in (exact conversion)

if ~isSim
    robot = raspbot()
end
tStart = tic; %Timer that never stops
tMove = tic; %Timer that's reset each call to the move function
%toc(tMove);

%%Do challenge course
tLoop = tic;
data = [];
startEnc = getEncoders;
hold all
tLoop = tic;
toc(tMove); %Comes back as 0.4 seconds; this seems bad, at least on the simulator end
tMove = tic; %Timer that's reset each call to the move function

%Forward for 3s
while toc(tLoop) < 3
   move(0.1016, 0.1016);
   temp = getEncoders;
   data = [data, [temp(1)-startEnc(1); temp(2)-startEnc(2); toc(tStart)]];
   %data is stored in meters; convert to cm when plotting
   plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
   xlabel = 'Time (s)';
   ylabel = 'Encoder distance (cm)';
   legend('leftEnc', 'rightEnc');
   pause(0.005);
end
%Stop for 1s
tLoop = tic;
while toc(tLoop) < 1
   move(0, 0);
   temp = getEncoders;
   data = [data, [temp(1)-startEnc(1); temp(2)-startEnc(2); toc(tStart)]];
   %data is stored in meters; convert to cm when plotting
   plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
   xlabel = 'Time (s)';
   ylabel = 'Encoder distance (cm)';
   legend('leftEnc', 'rightEnc');
   pause(0.005);
end
%Back up for 3s
tLoop = tic;
while toc(tLoop) < 3
   move(-0.1016, -0.1016);
   temp = getEncoders;
   data = [data, [temp(1)-startEnc(1); temp(2)-startEnc(2); toc(tStart)]];
   %data is stored in meters; convert to cm when plotting
   plot(data(3, :), data(1, :)*100, data(3, :), data(2, :)*100)
   xlabel = 'Time (s)';
   ylabel = 'Encoder distance (cm)';
   legend('leftEnc', 'rightEnc');
   pause(0.005);
end
hold off

function e = getEncoders()
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove
    if ~isSim
        e = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y];
    else
        e = [simle, simre];
    end
end

function dt = move(lv, rv)
    global simlv, global simrv, global simpos, global simle, global simre, global isSim, global vmax, global robot, global tStart, global tMove

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
        rv = vmax;
    end
    
    if ~isSim
        sendVelocity(robot, lv, rv);
    end
    dt = toc(tMove);
    ld = lv*dt;
    rd = rv*dt;
    simle = simle + ld; %Simulate encoder values forward
    simre = simre + rd;
    simlv = lv;
    simrv = rv;
    tMove = tic;
end