% Main script for drawing the polar diagram for the given sailboat
% Date: Oct. 18 2020
% Author: Daisy Zhang

function ANS = heeling_angle(z,p)
%{
% Calculate the heeling angle of the boat
% 
% Input
%     p: structure storing boat parameters, created by setBoatParam
%     function
%     z: current pose and (d/dt)pose
% Output
%     ANS = heel angle
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}
options = optimoptions('fsolve','TolFun',1e-20,'Display','off');
for heel_guess = linspace(0,2*pi,40)
            z0 = heel_guess;
            [heel,~,exitflag,~] = fsolve(@(heel_angle) rolling_moment(heel_angle,z,p),z0,options);
            if exitflag > 0
                ANS = heel;
                return
            end
end
end


function M = rolling_moment(heel,z,p)
%{
% Calculate the rolling moment of the boat
% 
% Input
%     heel: the boat's heeling angle
%     p: structure storing boat parameters, created by setBoatParam
%     function
%     z: current pose and (d/dt)pose
% Output
%     M: the rolling moment
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}
[L_sail,D_sail,L_keel,D_keel,L_rudder,D_rudder] = FM_on_boat(z,p);
n = [sin(z(1)),-cos(z(1))];
F_a = L_sail+D_sail+L_rudder+D_rudder;
F_h = L_keel+D_keel;
M = abs(dot(F_a,n))*p.Ra*cos(abs(heel))+abs(dot(F_h,n))*p.Rh*cos(abs(heel))-p.mass*p.g*p.l*sin(abs(heel));
end
