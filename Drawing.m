% Draw boat and its FBD
% Date: Oct. 18 2020
% Author: Daisy Zhang

[p,z0]=setBoatParam;
p.angle_rRels=0;
p.angle_sRelb=-pi/7;
p.v_airAngle=0;
p.v_airMag=-70;
p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)];
ANS = root_finding(p);
z = ANS';
[L_sail,D_sail,L_keel,D_keel,L_rudder,D_rudder,R_hull,M_rudder,M_keel,M_sail,M_hull] = FM_on_boat(z,p);

%draw boat
heading = z(3);
t = linspace(-2/3*pi,2/3*pi) ;
a = 0.3 ; b = 0.1 ;
x = a*cos(t) ;
y = b*sin(t) ;
x = [x(end) x];
y = [y(end) y];
plot(x*cos(heading)-sin(heading)*y,sin(heading)*x+cos(heading)*y,'b')
axis equal
hold on

plot(0,0,'b*','MarkerSize', 12)

sail = [p.d_sail,p.d_sail-p.SA_sail];
plot(sail*cos(heading+p.angle_sRelb),sail*sin(heading+p.angle_sRelb),'m','linewidth',3)


keel = [p.d_keel,p.d_keel-p.SA_keel];
plot(keel*cos(heading),keel*sin(heading),'r','linewidth',2)


sail_connect = [p.d_sail-p.SA_sail,p.d_rudder];
sail_connect_x = sail_connect*cos(heading+p.angle_sRelb);
sail_connect_y =sail_connect*sin(heading+p.angle_sRelb);
h = plot(sail_connect_x,sail_connect_y,'m','linewidth',0.2);
set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

rudder =  [0,0-p.SA_rudder];
plot(sail_connect_x(end)+rudder*cos(heading+p.angle_rRelb)...
    ,sail_connect_y(end)+rudder*sin(heading+p.angle_rRelb),'color','#FF6666','linewidth',2)
legend('Hull','C.O.M.','Sail','Keel','Rudder')
 set(gca,'visible','off')

% % FBD
% 
% scale =0.05;
% 
% quiver(mean(sail*cos(heading+p.angle_sRelb)),mean(sail*sin(heading+p.angle_sRelb)),L_sail(1)*scale,scale*L_sail(2),...
%     'color','#7E2F8E','linewidth',1,'maxheadsize',2)
% quiver(mean(sail*cos(heading+p.angle_sRelb)),mean(sail*sin(heading+p.angle_sRelb)),D_sail(1)*scale,scale*D_sail(2),...
%     'color','#A2142F','linewidth',1,'maxheadsize',2)
% 
% quiver(mean(sail_connect_x(end)+rudder*cos(heading+p.angle_rRelb)),mean(sail_connect_y(end)+rudder*sin(heading+p.angle_rRelb)),L_rudder(1)*scale,scale*L_rudder(2),...
%      'color', '#4DBEEE','linewidth',1,'maxheadsize',2)
% quiver(mean(sail_connect_x(end)+rudder*cos(heading+p.angle_rRelb)),mean(sail_connect_y(end)+rudder*sin(heading+p.angle_rRelb)),D_rudder(1)*scale,scale*D_rudder(2),...
%    'color', '#0072BD','linewidth',1,'maxheadsize',2)
% 
% quiver(mean(keel*cos(heading)),mean(keel*sin(heading)),L_keel(1)*scale,scale*L_keel(2),...
%       'color', '#D95319','linewidth',1,'maxheadsize',2)
% quiver(mean(keel*cos(heading)),mean(keel*sin(heading)),D_keel(1)*scale,scale*D_keel(2),...
%       'color', '#EDB120','linewidth',1,'maxheadsize',2)
% 
% quiver(0,0,R_hull(1)*scale,scale*R_hull(2),...
%     'color','#77AC30','linewidth',1,'maxheadsize',2)
% 
% scale_wind = 0.001;
% vb = quiver(0,0,z(2)*0.05,z(3)*0.05,'linewidth',1,'color','b','maxheadsize',2);
% % text(vb.XData+vb.UData,vb.YData+vb.VData,'v_{boat}','color','b','FontSize',14)
% va = quiver(0.3,0.3,p.v_a(1)*scale_wind,p.v_a(2)*scale_wind, 'k','linewidth',1,'maxheadsize',2);
% % text(va.XData+va.UData,va.YData+va.VData,'v_{wind}','FontSize',14)
% 
% legend('Hull','C.O.M.','Sail','Keel','Rudder','L_{sail}','D_{sail}','L_{rudder}','D_{rudder}','L_{keel}','D_{keel}','R_{hull}','location','bestoutside')
% % Draw heeling
% F_a = norm(L_sail+D_sail+L_rudder+D_rudder);
% F_h = norm(L_keel+D_keel);
% p.Ra = 0.5; % distance between center and COE of sail
% p.Rh = 0.1; % distance between center and COE of keel
% p.l = 0.5; % distance between COM and center
% heeling = heeling_angle(ANS,p);
% 
% t = linspace(0,-pi) ;
% a = 0.15; b = 0.1 ;
% x = a*cos(t) ;
% y = b*sin(t) ;
% x = [x(end) x];
% y = [y(end) y];
% plot(x*cos(heeling)-sin(heeling)*y,sin(heeling)*x+cos(heeling)*y,'b')
% axis equal 
% hold on
% O.y =-0.04;O.x = 0;
% plot(-sin(heeling)*O.y,cos(heeling)*O.y,'b*')
% text(0.04,-0.02,'O','color','b')
% 
% upper_rod.y = [0,1.3*p.Ra];
% lower_rod.y = [-b,-p.Rh-p.l];
% upper_rod.x = [0,0];
% lower_rod.x = [0,0];
% 
% lower_rod.x =lower_rod.x*cos(heeling)-sin(heeling)*lower_rod.y;
% lower_rod.y =sin(heeling)*lower_rod.x+cos(heeling)*lower_rod.y;
% upper_rod.x = upper_rod.x*cos(heeling)-sin(heeling)*upper_rod.y;
% upper_rod.y = sin(heeling)*upper_rod.x+cos(heeling)*upper_rod.y;
% plot(lower_rod.x,lower_rod.y,'b')
% plot(upper_rod.x,upper_rod.y,'b')
% scale = 0.05;
% % fa = quiver(mean(upper_rod.x),mean(upper_rod.y),scale*-F_a,0,'linewidth',1,'maxheadsize',1);
% % fh = quiver(2/6*sum(lower_rod.x),2/6*sum(lower_rod.y),scale*(F_h),0,'linewidth',1,'maxheadsize',1);
% % w = quiver(4/6*sum(lower_rod.x),4/6*sum(lower_rod.y),0,0.01*(-p.mass*9.8),'linewidth',1,'maxheadsize',1);
% % text(fa.XData+fa.UData,fa.YData+fa.VData,'F_a','color',	'#0072BD')
% % text(fh.XData+fh.UData,fh.YData+fh.VData,'F_h','color','#D95319')
% % text(w.XData+w.UData,w.YData+w.VData,'w','color',	'#EDB120')
% 
% set(gca,'visible','off')
% 
% plot([0,0],upper_rod.y,'--k')
% ang([0,0],0.1,[pi/2,pi/2+heeling],'k')
% text(-0.014,0.14,'\theta')
% 
% wave.x = linspace(-0.5,0.5);
% wave.y = 0.02*sin(20*wave.x)+O.y;
% plot(wave.x,wave.y,'color','#0066cc')
function h = ang(centre,radius,span,style)
% ANG
% Plots an angle arc with specified position and circumference.
% Example:
%                 ang([3 2],5,[0 pi/2],'k-')
% Plots an arc with centre (3,2) and radius (5) that represents
% The angle specified from 0 to pi/2, and with the preferred style 'k-'.
% Draws heavily from Zhenhai Wang's circle function on the File Exchange.
%
% Husam Aldahiyat, October, 2008.
theta = linspace(span(1),span(2),100);
rho = ones(1,100) * radius;
[x,y] = pol2cart(theta,rho);
x = x + centre(1);
y = y + centre(2);
h = plot(x,y,style);
end
