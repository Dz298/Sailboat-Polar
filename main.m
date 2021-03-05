%{
% Main script for drawing the polar diagram for the given sailboat 
% by solving all possible velocities first then plotting the maximum speed
% in all directions. 
% Date: Oct. 18 2020
% Author: Daisy Zhang
%}
[p,z0]=setBoatParam; 
sail_angle= deg2rad([linspace(-90,0,15) linspace(0,90,15)]);
rudder_angle= deg2rad([linspace(-90,0,15) linspace(0,90,15)]);

[~,~,heel_angles,angles,labels] = polar_diagram(p,sail_angle,rudder_angle);

