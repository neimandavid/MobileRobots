%I'm not convinced the sensor is actually off by 5 degrees
%My testing gets worse results if I fudge in either direction than if not
%Sensor can pick up my hand as about 2-3 degrees wide about 1.4m away
%Width accurate to about +/-1 degree, so anything less than 1 degree wide
%is scary to follow
%Filtering width seems to work pretty well
%Distance is consistent with what I'd expect

%0 degrees (maybe 1 degree, whatever...) is directly in front of the robot
%Let's use x_robot as forward, y_robot as left
%Maybe want to transform this so 0 is to the right, since we're not
%supposed to bother checking behind us
global robot

robot = raspbot();
robot.startLaser;
pause(1); %Get laser set up
while true
   ranges = getRanges();
   %ranges = robot.laser.LatestMessage.Ranges;
   for x = 1:360   
       if ranges(x) > 2 || ranges(x) < 0.06
           ranges(x) = 0;
       end
   end
   clf;
   plot(ranges);
   axis([1 10 0 2]);
   pause(0.05);
end

robot = raspbot();
robot.stopLaser;

function ranges = getRanges()
    global robot;
    
    tempRanges = robot.laser.LatestMessage.Ranges;
    ranges = zeros(360);
    ranges(1:355) = tempRanges(6:360);
    ranges(356:360) = tempRanges(1:5);
end