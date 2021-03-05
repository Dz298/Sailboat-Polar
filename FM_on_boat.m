function [L_sail,D_sail,L_keel,D_keel,L_rudder,D_rudder,R_hull,M_rudder,M_keel,M_sail,M_hull] = FM_on_boat(z,p)
%{
% Calculate force and moment on the boat
% 
% Input
%     p: structure storing boat parameters, created by setBoatParam
%     function
%     z: current pose and (d/dt)pose
% Output
%     L_sail: lift force on sail
%     D_Sail: drag force on sail
%     L_keel: lift force on keel
%     D_keel: drag force on keel
%     L_rudder: lift force on rudder
%     D_rudder: drag force on rudder
%     R_hull: resistance on hull
%     M_rudder: the area moment on rudder
%     M_keel: the area moment on keel
%     M_sail: the area moment on sail
%     M_hull: the area moment on hull
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}

vb_x = z(2);
vb_y = z(3);
t = z(1);
v_boat = [vb_x,vb_y];
omega  = 0;
%% SAIL
%%%Calculates lift drag forces on a sail at a fixed angle relative to the
%%%boat
%sail velocity relative to air
v_sail=v_boat+p.d_sail*omega*[-sin(t),cos(t)]-p.v_a;

%angle of attack of sail in air
alpha_sail= (t+p.angle_sRelb-atan2(v_sail(2),v_sail(1)));
[c_lift,c_drag]=C_LD(alpha_sail,p);

%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_lift*[-v_sail(2),v_sail(1)];


D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_drag*(-v_sail);

%Moment
M_sail=p.d_sail*cos(t)*(L_sail(2)+D_sail(2))-...
    p.d_sail*sin(t)*(L_sail(1)+D_sail(1));

%% RUDDER 
%%%Calculates the forces on the rudder given it is in the air
%velocity of rudder relative to air


v_rudder=v_boat+p.d_rudder*omega*[-sin(t+p.angle_sRelb),cos(t+p.angle_sRelb)]-p.v_a;

%angle of attack of rudder in air
alpha_rudder= (t+p.angle_rRels+p.angle_sRelb-atan2(v_rudder(2),v_rudder(1)));


[c_lift,c_drag]=C_LD(alpha_rudder,p);

%lift and drag on rudder
L_rudder=.5*p.rho_air*p.SA_rudder*norm(v_rudder)*...
    c_lift*[-v_rudder(2),v_rudder(1)];
D_rudder=.5*p.rho_air*p.SA_rudder*norm(v_rudder)*...
    c_drag*(-v_rudder);

%Moment
rx=p.d_rRels*cos(t+p.angle_sRelb)+p.d_sail*cos(t);
ry=p.d_rRels*sin(t+p.angle_sRelb)+p.d_sail*sin(t);
M_rudder=rx*(L_rudder(2)+D_rudder(2))-ry*(L_rudder(1)+D_rudder(1));

%% KEEL 
%%%Calculates lift drag forces on keel

%velocity of keel in water
v_keel=v_boat+p.d_keel*omega*[-sin(t),cos(t)];

%angle of attack of keel in water
alpha_keel= (t-atan2(v_keel(2),v_keel(1)));

[c_lift,c_drag]=C_LD(alpha_keel,p);

%lift and drag on keel
L_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_lift*[-v_keel(2),v_keel(1)];

D_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_drag*(-v_keel);

%Moment
M_keel=p.d_keel*cos(t)*(L_keel(2)+D_keel(2))-...
    p.d_keel*sin(t)*(L_keel(1)+D_keel(1));

%% HULL
%%%Calculates hull resistance and rotational damping

% Reynolds number hull
Rn_hull = norm(v_boat)*0.7*p.LWL/p.epsilon_sea;

% Hull frictional resistance
R_F = 0.5*p.rho_water*norm(v_boat)^2*p.immersed_SA_hull*(0.075/(log10(Rn_hull)-2)^2);

% Froude number
Fr = norm(v_boat)/sqrt(p.g*p.LWL);

% Hull residual resistance
if Fr< 1.25
R_R = p.fRR(Fr)*(0.5*p.rho_water*norm(v_boat)^2*p.immersed_SA_hull);
else
R_R = p.fRR(1.25)*(0.5*p.rho_water*norm(v_boat)^2*p.immersed_SA_hull);    
end
% solve for heeling angle
n = [sin(t),-cos(t)];
F_a = L_sail+D_sail+L_rudder+D_rudder;
F_h = L_keel+D_keel;
heeling = atan((abs(dot(F_a,n))*p.Ra+abs(dot(F_h,n))*p.Rh)/p.mass/p.g/p.l);

q = 0.5*p.rho_water*norm(v_boat)^2;
CH = 1/1e3*(6.747*(p.Tc/p.T)+2.517*(p.BWL/p.Tc)+3.71*(p.BWL/p.T));

%angle of attack of keel in water
alpha_hull= (t-atan2(v_boat(2),v_boat(1)));

[c_lift,c_drag]=C_LD(alpha_hull,p);

%lift and drag on keel
L_hull=.5*p.rho_air*(p.SA_hull-p.immersed_SA_hull)/2*norm(v_boat)*...
    c_lift*[-v_boat(2),v_boat(1)];

D_hull=.5*p.rho_air*(p.SA_hull-p.immersed_SA_hull)/2*norm(v_boat)*...
    c_drag*(-v_boat);

% Aspect ratio of keel and hull combined
ARE = p.T^2/(p.SA_keel+p.immersed_SA_hull);

% Side force of heel and keel combined
FH = abs(dot(F_h+L_hull+D_hull,n));

% Induced resistance
R_i = FH^2/(pi*q*p.immersed_SA_hull*ARE);


% Heeling resistance 
R_H = q*p.immersed_SA_hull*CH*Fr^2*heeling;


R_hull = -v_boat/norm(v_boat)*(R_F+R_H+R_R+R_i)+L_hull+D_hull;
%hull damping moment
M_hull=-2*omega;
end
function [c_lift,c_drag]=C_LD_hull(alpha,p)
%%%calculates local hull coeff. of lift and drag at a given angle of attack
%%%(alpha)

    c_lift=p.C0*sin(2*alpha);
    hull_paraDrag = 0.15;
    c_drag=hull_paraDrag+p.C0*(1-cos(2*alpha));

end
