function ANS = root_finding(p)
%{
% Calculate the velocity and the heading of the boat by root finding
% 
% Input
%     p: structure storing boat parameters, created by setBoatParam
%     function
% Output
%     ANS : ANS(2,3) = the velocity of the boat
%           ANS(1) = the heading of the boat 
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}

options = optimoptions('fsolve','Display','off','TolFun',1e-20);
for vb_x_guess = linspace(-3,3,20)
    for vb_y_guess = linspace(-3,3,20)
        for t_guess = linspace(0,2*pi,10)
            z0 = [t_guess;vb_x_guess;vb_y_guess];
            [z,~,exitflag,~] = fsolve(@(z) rhs(z,p),z0,options);            
            if exitflag > 0
                vb_x=z(2);
                vb_y=z(3);
                t = z(1);
                ANS = [t;vb_x;vb_y];
                return
            end
        end
    end
end
end

