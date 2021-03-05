% test script: plot Fn vs Cr
% Date: Oct. 18 2020
% Author: Daisy Zhang

p=setBoatParam;
r1 = [];
r2 = [];
r3 = [];
r4 = [];
l = [];
d = [];
Fr = [];
spd = [];
dir = [];
spd_air = [];
heading = pi/10;
for v = 0.1:0.01:5
    z = [0,v*cos(heading),v*sin(heading)];
    [R_F,R_R,R_H,R_i] = FM_on_boat(z,p);
    r1 = [r1 R_F];
    r2 = [r2 R_R/(0.5*p.rho_water*norm(v)^2*p.immersed_SA_hull)];
    r3 = [r3 R_H];
    r4 = [r4 R_i];
    Fr = [Fr norm(v)/sqrt(p.g*p.LWL)];
end
% figure
hold on
% plot(Fr,r1,Fr,r2,Fr,r3,Fr,r4);
 plot(Fr,r2,'linewidth',2,'color','r')
%   plot(Fr,r2,'kx')
% legend('R_F','R_R','R_H','R_I')
xlabel('F_r')
ylabel('C_R')
ylim([0,0.03])
hold on
fRR = fit(Fr',r2','poly1');