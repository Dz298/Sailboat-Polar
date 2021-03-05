%{
% Drawing the polar diagram for the given sailboat 
% by optimizing the maximum speed in all directions. 
% Date: Oct. 18 2020
% Author: Daisy Zhang
%}
 
figure
pa = polaraxes;
[p,z0]=setBoatParam;
h=waitbar(0,'Sailing...');
t=1;
dir_lst = -pi/2:0.1:pi/2;
wind_lst = 10:10:40;
tend = length(dir_lst)*length(wind_lst);
i=1;
spd = cell(1,length(wind_lst));
direct = spd;

colors = ['k','r','m','b','g'];

% optimize the maximum speed on all directions
for wind = wind_lst
    p.v_airMag=-wind; 
    p.v_airAngle=0; 
    p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)]; 
    for dir = dir_lst
        p.angle_rRelb = dir;
        waitbar(t/tend,h);
        t=t+1;
        [x,fval] = optimize_fun(dir,p); % find the sail and tail angles for max speed
        if ~isnan(x)
            p.angle_sRelb = x(1);%p.angle_rRelb = x(2);
            p.angle_rRels = p.angle_rRelb-p.angle_sRelb;
            ANS = root_finding(p);
        else
            ANS = [0;0;0];
        end
        spd{i} = [spd{i} norm(ANS(2:3))];
        direct{i} = [direct{i} atan2(ANS(3),ANS(2))];
        polarplot(atan2(ANS(3),ANS(2)),norm(ANS(2:3)),'marker','.','color',colors(i)); 
        hold on
        pa.ThetaLim = [0 180];
        pa.ThetaZeroLocation ='top';
    end
    i = i+1;
end
labels = {};
for j = 1:i-1
    labels{j} = sprintf('Speed = %.2f m/s',wind_lst(j));
end
legend(labels{1:i-1});

close(h)
figure(2);
p2 = polaraxes('ThetaZeroLocation','top');
figure(3);
p3 = polaraxes('ThetaZeroLocation','top');

% Fit the speed data points to corresponding directions by cubic interpolation 
for j = 1:i-1
    [s_direct,s_index] = sort(direct{j});
    spd_j = spd{j};
    s_spd = spd_j(s_index);
    rmv_out_spd =  filloutliers(s_spd,'pchip','movmedian',10);
    figure(2);
    polarplot(direct{j},spd_j,[colors(j) '.'],s_direct,rmv_out_spd,'rx');
    p2.ThetaZeroLocation ='top';
    hold on
    figure(3)
    f =fit(s_direct',rmv_out_spd','cubicinterp');
    polarplot(s_direct,f(s_direct))
    p3.ThetaZeroLocation ='top';
    hold on
end


function [x,fval] = optimize_fun(dir,p)
%%% optimize the VMG magnitude by finding 
%%% sail/tail angles
opts = optimoptions('fmincon','Display','iter','Algorithm','interior-point','MaxIterations',5e02);
p.angle_rRelb = dir;

for s_angle_guess = -pi/2:0.1:pi/2
        x0 = [s_angle_guess];
        [x,fval,eflag] = runobjconstr(x0,p,opts);
        if eflag > 0
            return
        elseif eflag == 0
            x = nan; fval = nan;
        end
end
end

function [x,f,eflag,outpt] = runobjconstr(x0,p,opts)
%%% optimize the objective by equality and inequality constraints

if nargin == 1 % No options supplied
    opts = [];
end

xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

fun = @(x) objfun(x,p); % the objective function, nested below
cfun = @(x) constr(x,p); % the constraint function, nested below

% Call fmincon
[x,f,eflag,outpt] = fmincon(fun,x0,[],[],[],[],[-pi/2],[pi/2],cfun,opts);

    function y = objfun(x,p)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computeall(x,p);
            xLast = x;
        end
        % Now compute objective function
        y = -norm(myf(2:3));
    end

    function [c,ceq] = constr(x,p)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computeall(x,p);
            xLast = x;
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

end


function [f1,c1,ceq1] = computeall(x,p)
%%% produce the objective, inequality constraint and equality constraint

p.angle_sRelb = x(1); 
p.angle_rRels = p.angle_rRelb-p.angle_sRelb;
f1 = root_finding(p);
c1 = [heeling_angle(f1,p)-pi/6 -atan2(f1(3),f1(2))];
ceq1 =[];
end

