classdef trajectoryFollower < handle
    
    properties (Constant)
    end
    
    properties (Access = public)
        vl;
        vr;
    end
    
    methods
        
        function obj = trajectoryFollower(model, traj)
            for i = 1:size(traj.t, 2)
                [vl(i), vr(i)] = model.VwTovlvr(traj.V(i), traj.w(i));
            end
            obj.vl = vl;
            obj.vr = vr;
        end
        
        function [vl, vr] = getvlvrAtTime(obj, model, traj, time)
            [V, w] = getVwAtTime(traj, time);
            [vl, vr] = model.VwTovlvr(V, w);
        end
        
    end
    
   
end