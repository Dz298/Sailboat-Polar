function [speed,dir,heel_angles,angles,labels] = polar_diagram(p,sail_angle,rudder_angle)
%{
% Draw the polar diagram of the given boat.
% 
% Input
%     p: structure storing boat parameters, created by setBoatParam
%     function
%     sail_angle: list of sail angles, size: 1xn
%     rudder_angle: list of rudder angles, size: 1xm
% Output
%     speed: nxm matrix storing the speed of the boat. 
%             speed(i,j) = speed of the boat with sail angle = sail_angle(i)
%                                                 rudder angle = rudder_angle(j)
%     dir: nxm matrix storing the direction of the boat.
%             dir(i,j) = boat direction, sail = sail_angle(i),rudder =
%             rudder_angle(j)
%     heel_angles: cell array containing all possible heel angles of boat
%     angles: cell arrays s.t.
%             angles{1,:} = all boat rudder angle rel. to sail
%             angle{2,:} = all boat sail angle rel. to boat
%             angle{3,:} = all boat rudder angle rel. to boat
%             angle{4,:} = all boat course direction
%     labels: cell arrays containing the wind speed information string
%             for later labeling
% 
% Date: Oct. 18 2020
% Author: Daisy Zhang
%}

h=waitbar(0,'Sailing...');
t=1;
final = figure(3); 
p3 = polaraxes;

sail_angle_mat = zeros(length(sail_angle),length(rudder_angle));
rudder_angle_mat = zeros(length(sail_angle),length(rudder_angle));

n =3;
tend = length(sail_angle)*length(rudder_angle)*n;
spd = linspace(35,37,n);
best_dir = cell(1,n);
best_spd = cell(1,n);
color = {'r','b','k','g','m','c','y'};
heel_angles = cell(1,n);
angles = cell(4,n);

for i = 1:n
    speed = zeros(length(sail_angle),length(rudder_angle));
    dir = zeros(length(sail_angle),length(rudder_angle));
    h1 = figure;
    ph1 = polaraxes; 
    hold on
    p.v_airMag = spd(i);
    p.v_airAngle = pi;
    p.v_a=p.v_airMag*[cos(p.v_airAngle),sin(p.v_airAngle)];
    hold on
    for idx = 1 : length(rudder_angle)
        p.angle_rRelb = rudder_angle(idx);
        for jdx = 1:length(sail_angle)
            waitbar(t/tend,h);
            t=t+1;
            p.angle_sRelb = sail_angle(jdx);
            p.angle_rRels = p.angle_rRelb-p.angle_sRelb;
            ANS = root_finding(p); % calculate velocity of the boat
            v_boat_x= ANS(2);
            v_boat_y = ANS(3);
            % calculate heel angle
            try
            heeling = heeling_angle(ANS,p);
            catch 
                heeling = 0;
            end
            heel_angles{i} = [heel_angles{i},heeling];
            angles{1,i} = [angles{1,i} p.angle_rRels];
            angles{2,i} = [angles{2,i} p.angle_sRelb];
            angles{3,i} = [angles{3,i} p.angle_rRelb];
            angles{4,i} = [angles{4,i} atan2(v_boat_y,v_boat_x)];
            v = [ v_boat_x;v_boat_y];
            speed(idx,jdx) = norm(v); % store speed 
            dir(idx,jdx) = atan2(v_boat_y,v_boat_x);% store direction
            sail_angle_mat(idx,jdx) = sail_angle(jdx); 
            rudder_angle_mat(idx,jdx) = rudder_angle(idx);
            if dir(idx,jdx)<=pi && dir(idx,jdx)>=0 && heeling < p.heelinglimit 
                % plot direction in [0,pi/2] since the polar diagram is symmetric
                % and only plot if the velocity is safe for sailing
                if norm(v) ~=0 
                    polarplot(atan2(v_boat_y,v_boat_x),speed(idx,jdx),'k.'); 
                else
                   polarplot(atan2(v_boat_y,v_boat_x),speed(idx,jdx),'rx');
                end
                ph1.ThetaLim = [0 90];
                ph1.ThetaZeroLocation ='top';
            end
       end
    end
    figure(3)
    sail_angle_sol = [];
    rudder_angle_sol = [];
    for idx = 1 : length(sail_angle)
        for jdx = 1:length(rudder_angle)
           same_dir_config = round(rad2deg(dir)/10) == round(rad2deg(dir(idx,jdx))/10);
           spd_in_same_dir = speed(same_dir_config);
           sail_in_same_dir = sail_angle_mat(same_dir_config);
           rudder_in_same_dir = rudder_angle_mat(same_dir_config);
           [M,I] = max(spd_in_same_dir); % find the max speed in approximately the same direction
           sail_angle_sol = [sail_angle_sol sail_in_same_dir(I)];
           rudder_angle_sol = [rudder_angle_sol rudder_in_same_dir(I)];
           direction = deg2rad(round(rad2deg(dir(idx,jdx))/10)*10);
           best_dir{i} = [best_dir{i} direction];
           best_spd{i} = [best_spd{i} M];           
        end
    end
    [sorted_dir, sorted_I] = sort(best_dir{i});
    best_dir{i} = sorted_dir;
    sorted_spd = best_spd{i};
    best_spd{i} = sorted_spd(sorted_I);
    assignin('base','best_dir',best_dir)
    assignin('base','best_spd',best_spd)
    sail_angle_sol = [sail_angle_sol(sorted_I)];
    rudder_angle_sol = [ rudder_angle_sol(sorted_I) ];
    pp = polarplot(best_dir{i},best_spd{i},'color',color{i},'marker','.');%plot max speed in all directions
    p3.ThetaZeroLocation ='top';
    p3.ThetaLim = [0 180];
    pp.DataTipTemplate.DataTipRows(1).Label = 'Direction [deg]';
    pp.DataTipTemplate.DataTipRows(2).Label = 'Speed [m/s]';
    row = dataTipTextRow('Sail Angle [deg]',rad2deg(sail_angle_sol));
    row1 = dataTipTextRow('Rudder Angle [deg]',rad2deg(rudder_angle_sol));
    pp.DataTipTemplate.DataTipRows(end+1) = row;
    pp.DataTipTemplate.DataTipRows(end+1) = row1;
    hold on
end
close(h);
title(sprintf('Polar Diagram'))
labels = {};
for i = 1:n
    labels{i} = sprintf('Speed = %.2f m/s',spd(i));
end
legend(labels{1:n});
end