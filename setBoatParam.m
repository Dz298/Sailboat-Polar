function [p,z0]=setBoatParam
%{
% Initializes various boat parameters in p structure
% 
% Input
%     None
% Output
%     p: structure storing boat parameters, created by setBoatParam
%     z0: initial conditions
% 
% Date: Oct. 19 2020
% Author: Daisy Zhang
%}
%% Initialized parameters
%initial conditions
x0=0; y0=0; th0=1; xdot0=0; ydot0=0; thdot0=0; %initial pose and (d/dt)pose
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';


%%%p.accuracy defines the accuracy with which lift and drag coeff. are
%%%interpolated
%1: most accurate but slower (using pchip interpolation)
%2: less accurate but fastest (approx. Clift Cdrag as sinusoidal)
p.accuracy=1;      

%sail
p.d_sail=0; %distance from C.O.M. to sail [m] (positive is infront COM)
p.SA_sail=.2; %surface area sail [m^2]
p.angle_sRelb=0; %angle of sail relative to boat [rad]
p.angle_sRelw=0.09;


%keel
p.d_keel=0; %distance from C.O.M. to keel [m] (positive is infront of COM)
p.SA_keel=0.1; %surface area keel [m^2]

%rudder
p.d_rudder=-0.5; %distance from C.O.M. to rudder [m] (positive is infront COM)
p.d_rRels=-0.5; %distance from sail center to rudder [m] (positive is infront of sail)
p.SA_rudder=0.05; %surface area rudder [m^2]
p.angle_rRelb=0*pi/180; %angle of rudder relative to boat [rad]
p.angle_rRels=0; %angle of rudder relative to sail [rad]
p.rudderType=3; %1: water rudder   2: air rudder    3: tail

%hull
p.SA_hull = 1;


p.v_airMag=-1; %magnitude of air velocity
p.v_airAngle=0; %direction of wind [rad]
p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; %x-y velocity componenets of air [m/s]
p.mass=7; %mass of boat [kg]
p.I=(1/12)*p.mass; %moment of inertia of boat about COM [kg*m^2]
p.rho_air=1.2; %density of air [kg/m^3]
p.rho_water=1000; %density of water [kg/m^3]


%%% Tabulated Data for NACA 0015 airfoil:
angle = [0,10,15,17,23,33,45,55,70,80,90,100,110,120,130,...
    140,150,160,170,180,190,200,210,220,230,240,250,260,...
    270,280,290,305,315,327,337,343,345,350,360]';

lift = [0,0.863,1.031,0.58,.575,.83,.962,.8579,.56,.327,...
    .074,-.184,-.427,-.63,-.813,-.898,-.704,-.58,-.813,0,...
    .813,.58,.704,.898,.813,.63,.427,.184,-.074,-.327,...
    -.56,-.8579,-.962,-.83,-.575,-.58,-1.031,-.863,0]';

drag = [0,.021,.058,.216,.338,.697,1.083,1.421,1.659,1.801,...
    1.838,1.758,1.636,1.504,1.26,.943,.604,.323,.133,0,...
    .133,.323,.604,.943,1.26,1.504,1.636,1.758,1.838,1.801,...
    1.659,1.421,1.083,.697,.338,.216,.058,.021,0]';

p.paraDrag=0.02; %parasitic drag used to limit max(Cl/Cd)=5
% drag=drag+p.paraDrag; %adjusted drag

p.C0=.9;  % nominal lift coefficient for sinusoidal lift/drag approx.

%%% Fit a pchip piecewise-polynomial curve fit to the data
% This is good because it preserves local maximum and minimum
% Another option would be to replace pchip() with spline(), which would
% produce a more smooth curve, but would add new peaks to the data.
p.lift = pchip(angle,lift);
p.drag = pchip(angle,drag);

p.lift_f = fit(angle,lift,'linearinterp');
p.drag_f = fit(angle,drag,'linearinterp');

p.Ra = 0.5; % distance between center and COE of sail
p.Rh = 0.1; % distance between center and COE of keel
p.l = 0.5; % distance between COM and center

p.heelinglimit = pi/12; % the limit of the heeling angle

%% Updated parameters for the sailboat prototype
p.epsilon_sea = 1.19e-6;
p.g = 9.81;

p.d_sail=0.117; 
p.SA_sail=0.64994;
p.m_sail = 6.39;
p.rho_sail = p.m_sail/p.SA_sail;

p.d_keel=0.103; 
p.SA_keel=0.38276;
p.m_keel = 6.39;
p.rho_keel = p.m_keel/p.SA_keel;
p.c_keel = 0.28219 ;% mean chord length keel 

p.d_rudder= -2.78; 
p.d_rRels=p.d_rudder-p.d_sail;   
p.SA_rudder=0.13224;
p.m_rudder = 1.4;
p.rho_rudder = p.m_rudder/p.SA_rudder;

p.LWL = 1.512; % water line length
p.BWL = 0.198; % water line breadth
p.Tc = 0.13; % hull draft 
p.T = 1.545; % total draft
r = p.BWL/2;
p.SA_hull = 0.6673;
p.immersed_SA_hull = 0.38569; % immersed hull area
p.immersed_vol_hull = 0.01788; % immersed hull volume
p.immersed_w_hull = p.rho_water*p.immersed_vol_hull*p.g; % immersed hull weight
p.Cp = 0.5487423121; % prismatic coefficient -> assume 
p.LCB = 0; % longitude center of buoyancy
p.Aw = 0.14285; % waterline area


p.mass=25.478;
p.I=15.073404; 

p.Ra = 0.758; 
p.Rh = 0.691; 
p.l = 1.054; 

% Residuary resistance polynomial coefficients (Gerritsma et al, 1992)
Fr_1 = (0.125:0.025:0.45)';
a0 = [ -6.735654  -0.382870 -1.503526 11.29218 22.17867 25.90867 40.97559 45.83759 89.20382...
    212.6788 336.2354 566.5476 743.4107 1200.62]';
a1 = [38.36831 38.17290 24.40803 -14.51947 -49.16784 -74.75668 -114.2855 -184.7646...
    -393.0127 -801.7908 -1085.134 -1609.632 -1708.263 -2751.715]';
a2 = [-0.008193 0.007243 0.0122 0.047182 0.085998 0.153521 0.207226 0.357031 0.617466...
    1.087307 1.644191 2.016090 2.435809 3.208577]';
a3 = [0.055234 0.026644 0.067221 0.085176 0.150725 0.188568 0.250827 0.338343 0.460472...
    0.538938 0.532702 0.265722 0.013553 0.254920]';
a4 = [-1.997242 -5.295332 -2.448582 -2.673016 -2.878684 -0.889467 -3.072662 3.871658 11.54327 ...
    10.80273 -1.223173 -29.24412 -81.16189 -132.0424]';
a5 = [-38.86081 -39.55032 -31.91370 -11.41819 7.167049 24.12137 53.01570 132.2568 331.1197...
    667.6445 831.1445 1154.091 937.4014 1489.269]';
a6 = [0.956591 1.219563 2.216098 5.654065 8.600272 10.48516 13.02177 10.86054 8.598136...
    12.39815 26.18321 51.46175 115.6006 196.3406]';
a7 = [-.002171 .000052 .000074 .007021 .012981 .025348 .035934 .066809 .104073...
    .166473 .238795 .2888046 .365071 .528225]';
a8 = [0.272895 .824568 .244345 -.094934 -.327085 -0.85494 -0.715457 -1.719215 -2.815203 -3.026131...
    -2.45047 -.0178354 1.838967 1.379102]';
a9 = [-0.017516 -0.047842 -0.015887 0.006325 0.018271 0.048449 0.039874 0.095977...
    0.15596 0.165055 0.139154 0.018446 -0.062023 0.013577]';

p.a0 = fit(Fr_1,a0,'cubicinterp');
p.a1 = fit(Fr_1,a1,'cubicinterp');
p.a2 = fit(Fr_1,a2,'cubicinterp');
p.a3 = fit(Fr_1,a3,'cubicinterp');
p.a4 = fit(Fr_1,a4,'cubicinterp');
p.a5 = fit(Fr_1,a5,'cubicinterp');
p.a6 = fit(Fr_1,a6,'cubicinterp');
p.a7 = fit(Fr_1,a7,'cubicinterp');
p.a8 = fit(Fr_1,a8,'cubicinterp');
p.a9 = fit(Fr_1,a9,'cubicinterp');

Fr_2 = (0.475:0.025:0.75)';
c0 = [180.1004 243.9994 282.9873 313.4109 337.0038 356.4572 324.7357 301.1268 292.0571...
    284.4641 256.6367 304.1803]';
c1 = [-31.50257 -44.52551 -51.51953 -56.58257 -59.19029 -62.85395 -51.31252 -39.79631...
    -31.85303 -25.14558 -19.31922 -30.11512]';
c2 = [-7.451141 -11.15456 -12.97310 -14.41978 -16.06975 -16.85112 -15.34595 -15.02299 ...
    -15.58548 -16.15423 -13.08450 -15.85429]';
c3 = [2.195042 2.179046 2.274505 2.326117 2.419156 2.437056 2.334146 2.059657 1.847926 1.703981...
    2.152824 2.863173]';
c4 = [2.689623 3.857403 4.343662 4.690432 4.766793 5.078768 3.855368 2.545676 1.569917 0.817912 0.348305 1.524379]';
c5 = [.00648 .009676 .011066 .012147 .014147 .014980 .013695 .013588 .014014 .014575 .011343 .014031]';
p.c0 = fit(Fr_2,c0,'cubicinterp');
p.c1 = fit(Fr_2,c1,'cubicinterp');
p.c2 = fit(Fr_2,c2,'cubicinterp');
p.c3 = fit(Fr_2,c3,'cubicinterp');
p.c4 = fit(Fr_2,c4,'cubicinterp');
p.c5 = fit(Fr_2,c5,'cubicinterp');

RR = [];
Fr_lst = [];
for v = 0.1:0.01:5
    Fr = v/sqrt(p.g*p.LWL);
    % Hull residual resistance 
    if Fr >= 0.125 && Fr < 0.45
        R_R = p.immersed_w_hull/1e3*(p.a0(Fr)+p.a1(Fr)*p.Cp+p.a2(Fr)*p.LCB+p.a3(Fr)*(p.BWL/p.Tc)...
            +p.a4(Fr)*(p.LWL/(p.immersed_vol_hull^(1/3)))+p.a5(Fr)*p.Cp^2+p.a6(Fr)*p.Cp*...
            (p.LWL/(p.immersed_vol_hull^(1/3)))+p.a7(Fr)*p.LCB^2+p.a8(Fr)*...
            (p.LWL/(p.immersed_vol_hull^(1/3)))^2+p.a9(Fr)*(p.LWL/(p.immersed_vol_hull^(1/3)))^3);
    elseif Fr >= 0.45 && Fr < 0.475
        if abs(Fr-0.45) <= abs(Fr-0.475)
            R_R = p.immersed_w_hull/1e3*(p.a0(.45)+p.a1(.45)*p.Cp+p.a2(.45)*p.LCB+p.a3(.45)*(p.BWL/p.Tc)...
            +p.a4(.45)*(p.LWL/(p.immersed_vol_hull^(1/3)))+p.a5(.45)*p.Cp^2+p.a6(.45)*p.Cp*...
            (p.LWL/(p.immersed_vol_hull^(1/3)))+p.a7(.45)*p.LCB^2+p.a8(.45)*...
            (p.LWL/(p.immersed_vol_hull^(1/3)))^2+p.a9(.45)*(p.LWL/(p.immersed_vol_hull^(1/3)))^3);
        else
            R_R = p.immersed_w_hull/1e3*(p.c0(.475)+p.c1(.475)*(p.LWL/p.BWL)+p.c2(.475)*(p.Aw/(p.immersed_vol_hull^(2/3)))+...
            p.c3(.475)*p.LCB+p.c4(.475)*(p.LWL/p.BWL)^2+p.c5(.475)*(p.LWL/p.BWL)*(p.Aw/(p.immersed_vol_hull^(2/3)))^3);
        end
    elseif Fr >= 0.475 && Fr < 0.75
        R_R = p.immersed_w_hull/1e3*(p.c0(Fr)+p.c1(Fr)*(p.LWL/p.BWL)+p.c2(Fr)*(p.Aw/(p.immersed_vol_hull^(2/3)))+...
            p.c3(Fr)*p.LCB+p.c4(Fr)*(p.LWL/p.BWL)^2+p.c5(Fr)*(p.LWL/p.BWL)*(p.Aw/(p.immersed_vol_hull^(2/3)))^3);
    elseif Fr>= 0.75
         R_R = p.immersed_w_hull/1e3*(p.c0(.75)+p.c1(.75)*(p.LWL/p.BWL)+p.c2(.75)*(p.Aw/(p.immersed_vol_hull^(2/3)))+...
            p.c3(.75)*p.LCB+p.c4(.75)*(p.LWL/p.BWL)^2+p.c5(.75)*(p.LWL/p.BWL)*(p.Aw/(p.immersed_vol_hull^(2/3)))^3);
    else
        R_R = 0;
    end
    RR = [RR R_R/(0.5*p.rho_water*norm(v)^2*p.immersed_SA_hull)];
    Fr_lst = [Fr_lst Fr];
end
% fit residual resistance to Fn  
p.fRR = fit(Fr_lst',RR','cubicinterp');

% Cr and Fn values read from (Dubrovsky et al, 2006)
Cr = [0.55,0.55,0.8,0.9,1.2,1.4,1.8,2.3,4.5,5.6,7.1,9.2,12]/1000;Fn = [0.15,0.2,0.24,0.25,0.27,0.28,0.3,0.32,0.36,0.37,0.38,0.39,0.4];
CroverLCB = [0.1,0.335];Fn4CroverLCB = [0.24,0.3];
delta_Cr = fit(Fn4CroverLCB',CroverLCB','poly1');
LCB = [1.5,-2]/100;Fn4LCB = [0.18,0.26];
delta_LCB = fit(Fn4LCB',LCB','poly1');
Cr = Cr+delta_Cr(Fn)'.*delta_LCB(Fn)'./1000+0.16*(p.BWL/p.Tc-2.5)/1000;

% extend ranges of Cr and of Fn by reading values from the plots generated
% from formulas in (Gerritsma et al, 1992)
Cr = [0,Cr,0.026,0.0168,0.0106,0.0091908,0.0078313,0.0068829,0.0059881,0.005127];
Fn = [0,Fn,0.51151,0.649,0.86983,0.93474,1.0126,1.0801,1.158,1.2515];
p.fRR=fit(Fn',Cr','cubicinterp');
end

