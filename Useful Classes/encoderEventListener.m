function encoderEventListener(handle, event)

global encI;
global encDT;
global T_R;
global X_R;
global Y_R;
global TH_R;
global leftPrev;
global rightPrev;
global timePrev;
global startTime;
global v_left;
global v_right;
global VL;
global VR;

left = event.Vector.X;
right = event.Vector.Y;
timestamp = double(event.Header.Stamp.Sec) + ...
 double(event.Header.Stamp.Nsec)/1000000000.0;
if (encI == 1)
    leftPrev = left;
    rightPrev = right;
    timePrev = timestamp;
    startTime = timestamp;
end

ds_left = left - leftPrev;
ds_right = right - rightPrev;
dt = timestamp - timePrev;
encDT(encI) = dt;
T_R(encI) = timestamp - startTime;
VL = ds_left/dt;
VR = ds_right/dt;
v_left(encI) = ds_left/dt;
v_right(encI) = ds_right/dt;

[X_R, Y_R, TH_R] = modelDiffSteerRobot(v_left, v_right, encDT);
%plot(X_R, Y_R);
%xlim([-1 0.5]);
%ylim([-0.5 0.5]);
%title('Location of Robot');
%xlabel('X Position');
%ylabel('Y Position');

% After calculations, update
leftPrev = event.Vector.X;
rightPrev = event.Vector.Y;
timePrev = timestamp;
encI = encI + 1;

end