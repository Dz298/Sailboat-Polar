function F = rhs(z,p)
%{
% Calculate force and moment on the boat
% 
% Input
%     p: structure storing boat parameters, created by setBoatParam
%     function
%     z: current pose and (d/dt)pose
% Output
%     F: F(1,2) is the net force on boat, F(3) is the net moment
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}

[L_sail,D_sail,L_keel,D_keel,L_rudder,D_rudder,R_hull,M_rudder,M_keel,M_sail,M_hull] = FM_on_boat(z,p);

F(3)= M_rudder+M_keel+M_sail+M_hull;
Ftot=L_sail+D_sail+L_keel+D_keel+L_rudder+D_rudder+R_hull;
F(1) = Ftot(1);
F(2) = Ftot(2);
end

