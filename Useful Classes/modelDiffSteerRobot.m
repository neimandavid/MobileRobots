function [x, y, th] = modelDiffSteerRobot(vl, vr, dt)
    
    %t = t0:dt:tf;

    W = 0.089;
    V = (vr + vl) / 2;
    w = (vr - vl) / W;
    
    x = zeros(1);
    y = zeros(1);
    th = zeros(1);
    
    for i = 2:size(vl, 2)
        th(i) = th(i-1) + w(i)*dt(i)/2;
        x(i) = x(i-1) + V(i)*cos(th(i))*dt(i);
        y(i) = y(i-1) + V(i)*sin(th(i))*dt(i);
        th(i) = th(i-1) + w(i)*dt(i);
    end
    
    % figure;
    % plot(x, y);
    
    %x = x(end);
    %y = y(end);
    %th = th(end);
    
end