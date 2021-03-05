% plot area vs VMG
% Date: Oct. 18 2020
% Author: Daisy Zhang

dir_lst = linspace(pi/3,pi/2,7);
wind_lst = 10;
[p,~]=setBoatParam;
VMG = cell(1,length(wind_lst));
Fn = VMG;
v_boat_x = VMG;
v_boat_y = VMG;
i=1;
p.opt_SA = VMG;
colors=['b','k','r'];

guess_x = [0.1,0.1];
wind = 10;
% % Find VMG vs SA_sail
% for SA_sail = 5:0.1:10
% % plot area vs VMG
%  p.SA_sail = SA_sail;
%  [x,fval] = find_VMG(guess_x,p,wind);
%  guess_x = x;
% end

% Find L/D vs SA_sail
p.m_sail = 1;
SA_sail = p.m_sail/p.rho_sail;
p.SA_sail = SA_sail;
while (p.m_sail<=5)
    [x,fval] = find_VMG(guess_x,p,wind);
    guess_x = x;
    SA_sail = SA_sail+0.01;
    p.SA_sail = SA_sail;
    p = update_boat(p,p.SA_keel,p.SA_sail,p.SA_rudder);
end


% for opt_wind = wind_lst
%      [x,fval,ANS]=optimize_VMG(p,opt_wind);
%     p = update_boat(p,x(3),2,x(4));
%     p.opt_SA{i} = x(3:4);
%     wind = 1:45;
%     h = waitbar(0,'Calculating...');
%     guess_x = [-pi/4,0.1];
%     for j = 1:length(wind)
%         [x_angles,vmg] = find_VMG(guess_x,p,wind(j));
%         guess_x = x_angles;
%         p.v_airMag=-wind(j); 
%         p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
%         p.angle_sRelb = x_angles(1);
%         p.angle_rRelb = x_angles(2);
%         p.angle_rRels =  p.angle_rRelb-p.angle_sRelb;
%         ANS = root_finding(p);
%         v_boat_x{i} = [v_boat_x{i},ANS(2)];
%         v_boat_y{i} = [v_boat_y{i},ANS(3)];
%         
%         waitbar(j/length(wind),h);
%         Fn{i} = [Fn{i} norm(ANS(2:3))/sqrt(p.g*p.LWL)];
%         if isnan(vmg)
%             VMG{i} = [VMG{i},nan];
%             break
%         else
%         VMG{i} = [VMG{i},-vmg];
%         end
%     end
%     close(h)
%     plot(wind(1:j),VMG{i},colors(i))
%     hold on
%     i = i+1;
% end
% xlabel('V_{wind} [m/s]');
% ylabel('VMG [m/s]');
% legend('Optimizied for 5 m/s wind','Optimizied for 10 m/s wind','Optimizied for 15 m/s wind')
%     
function [x,fval] = find_VMG(guess_x,p,wind)
%%% plot velocity made good vs sail/tail area
p.v_airMag=-wind; 
p.v_airAngle=0; 
p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
[x,fval] = optimize_fun(p,1,guess_x);
p.angle_sRelb = x(1);
p.angle_rRelb = x(2);
p.angle_rRels =  p.angle_rRelb-p.angle_sRelb;
ANS = root_finding(p);
SA_sail = p.SA_sail;


% plot SA_sail vs L/D
v_boat = ANS(2:3);
v_sail=v_boat-p.v_a;
t = ANS(1);
%angle of attack of sail in air
alpha_sail= (t+p.angle_sRelb-atan2(v_sail(2),v_sail(1)));
[c_lift,c_drag]=C_LD(alpha_sail,p);
%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_lift*[-v_sail(2),v_sail(1)];
D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_drag*(-v_sail);
plot(p.SA_sail,norm(L_sail)/norm(D_sail),'k.');
hold on
pause(.1)


% 
% % Plot data (area tendency plot)
% subplot(2,1,1)
% pp = polarplot(atan2(ANS(3),ANS(2)),norm(ANS(2:3)),'rx'); 
% pp.DataTipTemplate.DataTipRows(1).Label = 'Direction [deg]';
% pp.DataTipTemplate.DataTipRows(2).Label = 'Speed [m/s]';
% 
% % change
% row = dataTipTextRow('SA_sail AREA [m]',SA_sail);
% %
% 
% pp.DataTipTemplate.DataTipRows(end+1) = row;
% hold on
% pause(0.01)
% subplot(2,1,2)
% 
% % change
% plot(SA_sail,-fval,'bx');
% xlabel('Sail Projected Area [m^2]')
% %
% 
% ylabel('VMG [m/s]')
% axis equal
% hold on
% pause(0.01)
end

function [x,fval,ANS]=optimize_VMG(p,wind)
%%% optimize the best VMG by finding the best airfoil area
%%% x contains the sail angle, tail angle, keel area, and rudder area for
%%% the best VMG
p.v_airMag=-wind; 
p.v_airAngle=0; 
p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
guess_x0 = [0,0,0.4,0.05];
[x,fval] = optimize_fun(p,1,guess_x0);
p.angle_sRelb = x(1);
p.angle_rRelb = x(2);
p.angle_rRels =  p.angle_rRelb-p.angle_sRelb;
p = update_boat(p,x(3),2,x(4));
ANS = root_finding(p);
end

function p = update_boat(p,SA_keel,SA_sail,SA_rudder)
%%% update the boat parameters with airfoil areas changed
p.mass = p.mass-p.m_sail-p.m_rudder-p.m_keel;
p.I = p.I -p.m_sail*p.d_sail^2-p.m_keel*p.d_keel^2-p.m_rudder*p.d_rudder^2;
p.m_sail = SA_sail*p.rho_sail;
p.m_keel = SA_keel*p.rho_keel;
p.m_rudder = SA_rudder*p.rho_rudder;
p.SA_sail = SA_sail;
p.SA_keel = SA_keel;
p.SA_rudder = SA_rudder;
p.mass=p.mass+p.m_sail+p.m_rudder+p.m_keel;
p.I=p.I+p.m_sail*p.d_sail^2+p.m_keel*p.d_keel^2+p.m_rudder*p.d_rudder^2;
end


function optimize_polar(p,wind_lst,dir_lst)
%%% simplified polar_diagram specified dir_lst 
figure
pa = polaraxes;
h=waitbar(0,'Sailing...');
t=1;

tend = length(dir_lst)*length(wind_lst);
i=1;
spd = cell(1,length(wind_lst));
direct = spd;
no_sol_dir_lst = cell(1,length(wind_lst));
colors = ['k','r','m','b','g'];
for wind = wind_lst
    p.v_airMag=-wind; 
    p.v_airAngle=0; 
    p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
    guess_x0 = [0,-pi/2];
    for dir_1 = dir_lst 
        if ~(i>1&& any(no_sol_dir_lst{i-1}==dir_1))
            p.dir = dir_1;
             waitbar(t/tend,h);
            t=t+1;
            [x,fval,guess_x0] = optimize_fun(p,0,guess_x0);
            if ~isnan(x)
                spd{i} = [spd{i} -fval];
                direct{i} = [direct{i} dir_1];
                polarplot(dir_1,-fval,'marker','.','color',colors(i)); 
                hold on
                pa.ThetaLim = [0 90];
                pa.ThetaZeroLocation ='top';
            else
                no_sol_dir_lst{i} = [no_sol_dir_lst{i} dir_1];
            end
        end
    end
    i = i+1;
end

labels = {};
for j = 1:i-1
    labels{j} = sprintf('Speed = %.2f m/s',wind_lst(j));
end
legend(labels{1:i-1});

close(h)
figure(11);
% p3 = polaraxes('ThetaZeroLocation','top');

for j = 1:i-1
    [s_direct,s_index] = sort(direct{j});
    spd_j = spd{j};
    s_spd = spd_j(s_index);
    f =fit(s_direct',s_spd','cubicinterp');
    figure(11);
    polarplot(s_direct,f(s_direct),'linewidth',1);
%     p3.ThetaZeroLocation ='top';
%     p3.ThetaLim = [0 180];
    hold on
end
figure(11)
legend(labels{1:i-1});
end
function verify_dots_method(wind_lst)
%%% simplified polar_diagram 
a1 = gca;
f2 = figure(12);
a2 = copyobj(a1,f2);
figure(12)

color_default = {[0    0.4470    0.7410],
   [ 0.8500    0.3250    0.0980],
    [0.9290    0.6940    0.1250],
   [ 0.4940    0.1840    0.5560]};
color_default = {'b','r','y','m'};
sail_angle= deg2rad([linspace(-90,0,10) linspace(0,90,10)]);
rudder_angle= deg2rad([linspace(-90,0,10) linspace(0,90,10)]);
i = 1;
h=waitbar(0,'All possible velocities...');
tend = length(wind_lst)*length(sail_angle)*length(rudder_angle);
t=0;
pause(1)
dir_boat = cell(1,length(wind_lst));
spd_boat = cell(1,length(wind_lst));
for wind = wind_lst
    p.v_airMag=-wind; 
    p.v_airAngle=0; 
    p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
    guess_x0 = [0,-pi/2];
    for idx = 1 : length(sail_angle)
        p.angle_rRelb = rudder_angle(idx);
        for jdx = 1:length(rudder_angle)
            t = t+1;
            waitbar(t/tend,h);
            p.angle_sRelb = sail_angle(jdx);
            p.angle_rRels = p.angle_rRelb-p.angle_sRelb;
            ANS = root_finding(p);            
            v_boat_x= ANS(2);
            v_boat_y = ANS(3);
            try
            heeling = heeling_angle(ANS,p);
            catch 
                heeling = 0;
            end
             if heeling < p.heelinglimit
                v = [ v_boat_x;v_boat_y];
                dir_boat{i} = [dir_boat{i} atan2(v_boat_y,v_boat_x)];
                spd_boat{i} = [spd_boat{i} norm(v)]; 
               polarplot(atan2(v_boat_y,v_boat_x),norm(v),'marker','.',...
                    'color',color_default{i},'markersize',5); 
                hold on
                p3.ThetaLim = [0 180];
                p3.ThetaZeroLocation ='top';
             end
        end
    end
    i = i+1;
end
close(h)
figure;
i=1;
for wind = wind_lst
    polarplot(dir_boat{i},spd_boat{i},'d','marker','.',...
                    'color',color_default{i},'markersize',5); 
    p3.ThetaZeroLocation ='top';
    p3.ThetaLim = [0 180];
    hold on
    i=i+1;
end
end


function [x,fval,guess_next] = optimize_fun(p,ifVMG,guess_x0)
%%% optimize the VMG magnitude by finding the best airfoil area and
%%% sail/tail angles
opts = optimoptions('fmincon','Display','iter','Algorithm','interior-point','MaxIterations',100);
if ~ifVMG
[x,fval,eflag] = runobjconstr(guess_x0,p,ifVMG,opts);
if eflag>0
    guess_next = x;
    return
end
end

for s_angle_guess = -pi/2:.2:pi/2
    for r_angle_guess = -pi/2:.2:pi/2
        if ~ifVMG || length(guess_x0) ==2
            x0 = [s_angle_guess,r_angle_guess];
        else
            x0 = [s_angle_guess,r_angle_guess,guess_x0(3:4)];
        end
        [x,fval,eflag] = runobjconstr(x0,p,ifVMG,opts);
        if eflag > 0
            guess_next = x;
            return
        end
    end
end
x = [nan,nan];fval = nan;
guess_next = guess_x0;
end

function [x,f,eflag,outpt] = runobjconstr(x0,p,ifVMG,opts)
%%% optimize the objective by equality and inequality constraints

if nargin == 3 % No options supplied
    opts = [];
end

xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

fun = @(x) objfun(x,p); % the objective function, nested below
cfun = @(x) constr(x,p); % the constraint function, nested below

% Call fmincon
if length(x0)>2
    [x,f,eflag,outpt] = fmincon(fun,x0,[],[],[],[], [-inf,-inf,0,0],[inf,inf,inf,inf],cfun,opts);
else
    [x,f,eflag,outpt] = fmincon(fun,x0,[],[],[],[], [],[],cfun,opts);
end
    function y = objfun(x,p)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computeall(x,p,ifVMG);
            xLast = x;
        end
        % Now compute objective function
        if ~ifVMG
            y = -norm(myf(2:3));
        else
            y = -myf(2);
        end
    end

    function [c,ceq] = constr(x,p)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computeall(x,p,ifVMG);
            xLast = x;
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

end


function [f1,c1,ceq1] = computeall(x,p,ifVMG)
%%% produce the objective, inequality constraint and equality constraint

p.angle_sRelb = x(1); p.angle_rRelb = x(2);
p.angle_rRels =  p.angle_rRelb-p.angle_sRelb;
if length(x)>2
    p = update_boat(p,x(3),2,x(4));
end
f1 = root_finding(p);
c1 = heeling_angle(f1,p)-p.heelinglimit;
dir_of_b = atan2(f1(3),f1(2));
if ~ifVMG
    if round(dir_of_b,4) == round(-pi,4)
        dir_of_b = pi;
    end
    ceq1 =p.dir-dir_of_b;
else 
    if length(x)>2
    c1 = [c1,-dir_of_b,dir_of_b-pi/2];
    end
    ceq1 = [];
end

end