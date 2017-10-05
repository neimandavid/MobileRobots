classdef robotTrajectory < handle
    
    properties (Constant)
       dt = 0.001;
    end
    
    properties (Access = public)
        x;
        y;
        th;
        p;
        s;
        t;
        V;
        w;
        numSamples;
    end
    
    methods
        
        function obj = robotTrajectory(p0, s0, referenceControl)
            x(1) = p0(1);
            y(1) = p0(2);
            th(1) = p0(3);
            p(1, 1:3) = p0;  % pose
            s(1) = s0;  % distance
            
            duration = getTrajectoryDuration(referenceControl);
            obj.numSamples = round(duration / obj.dt);
            
            for (i=1:obj.numSamples-1)
                t(i) = (i-1) * obj.dt;
                
                [V_ref, w_ref] = computeControl(referenceControl, t(i));
                
                V(i) = V_ref;
                w(i) = w_ref;
                s(i+1) = s(i) + V(i)*obj.dt;
                
                x(i+1) = x(i) + V(i)*cos(th(i))*obj.dt;
                y(i+1) = y(i) + V(i)*sin(th(i))*obj.dt;
                th(i+1) = th(i) + w(i)*obj.dt;
                p(i+1, 1:3) = [x(i+1), y(i+1), th(i+1)];
            end
            t(obj.numSamples) = (obj.numSamples-1)*obj.dt;
            [V_ref, w_ref] = computeControl(referenceControl, t(obj.numSamples));
            V(obj.numSamples) = V_ref;
            w(obj.numSamples) = w_ref;
            
            obj.t = t;
            obj.V = V;
            obj.w = w;
            obj.s = s;
            obj.x = x;
            obj.y = y;
            obj.th = th;
            obj.p = p;
        end
        
        
        function pose = getPoseAtTime(obj, time)
            index = time / obj.dt;
            if (index > size(obj.t, 2))
                pose = obj.p(end, 1:3);
            elseif (index <= 0)
                pose = obj.p(1, 1:3);
            elseif (rem(index, 1) == 0)
                pose = obj.p(index, 1:3);
            else
                a = floor(index) + 1;
                b = ceil(index) + 1;
                
                tq = [obj.t(a), time, obj.t(b)];
                T = [obj.t(a), obj.t(b)];
                X = [obj.x(a), obj.x(b)];
                Y = [obj.y(a), obj.y(b)];
                TH = [obj.th(a), obj.th(b)];
                
                xq = interp1(T, X, tq);
                yq = interp1(T, Y, tq);
                thq = interp1(T, TH, tq);
                
                pose = [xq(2), yq(2), thq(2)];
            end
        end
        
        function [V, w] = getVwAtTime(obj, time)
            index = time / obj.dt;
            if (index > size(obj.t, 2))
                V = 0;
                w = 0;
            elseif (index <= 0)
                V = 0;
                w = 0;
            elseif (rem(index, 1) == 0)
                V = obj.V(index);
                w = obj.w(index);
            else
                a = floor(index) + 1;
                b = ceil(index) + 1;
                
                tq = [obj.t(a), time, obj.t(b)];
                T = [obj.t(a), obj.t(b)];
                Vsample = [obj.V(a), obj.V(b)];
                wsample = [obj.w(a), obj.w(b)];
                
                Vq = interp1(T, Vsample, tq);
                wq = interp1(T, wsample, tq);
                
                V = Vq(2);
                w = wq(2);
            end
        end
    end
    
   
end