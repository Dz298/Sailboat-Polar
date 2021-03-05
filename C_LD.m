function [c_lift,c_drag]=C_LD(alpha,p)
%{
% calculates local coeff. of lift and drag at a given angle of attack
% 
% Input
%     alpha: angle of attack
%     p: structure storing boat parameters, created by setBoatParam
% Output
%     c_lift: lift coefficient
%     c_drag: drag_coefficient
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}
%     c = 1/15;
%     attenuator = (pi-1)*(exp(-(sin(alpha).^4/c^2))) + 1;
    c_lift=p.C0*sin(2*alpha);
%       c_lift = c_lift.*attenuator; % fits to the NACA0015 data better but
                                       % is calculation heavy
    c_drag=p.paraDrag+p.C0*(1-cos(2*alpha));

end
