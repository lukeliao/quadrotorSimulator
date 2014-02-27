close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');

nmap = 3;

show_plot = 0; % Only set this to 1 if on map 5
switch nmap
    case 0
        map = load_map('maps/emptyMap.txt', 0.1, 2.0, 0.25);
        start = {[5 5 5]};
        stop  = {[5 5 2]};
    case 1
        map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);
        start = {[0.0  -4.9 0.2]};
        stop  = {[6.0  18.0-6 3.0]};
    case 2
        map = load_map('maps/map2.txt', 0.1, 2.0, 0.25);
        start = {[10,10,0.5]};
        stop  = {[4, 25, 3]};
    case 3
        map = load_map('maps/map3.txt', 0.1, 2.0, 0.25);
        start = {[18,3,5]};
        stop  = {[6, 4, 5]};
    case 4
        map = load_map('maps/mymap.txt', 0.1, 2.0, 0);
        start = {[7 5 5]};
        stop  = {[5 5 2]};
    case 5
        % Good for animating        
        show_plot = 1;
        map = load_map('maps/map1_mod.txt', 1, 1, 0.2);
        start = {[1  -4 0]};
        stop  = {[6.0  18 2.5]};
end


nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true, show_plot);
end
%%
if nquad == 1
    if ~show_plot
        plot_path(map, path{1});
    end
else
    % you could modify your plot_path to handle cell input for multiple robots
end

% %% Additional init script
init_script;
% %% Run trajectory
testtraj(tf);
if show_plot   
    hold on;
    if show_plot == 1
            for i = 1:25
            pause(1/25);
            animate_frame;
            end
    end
end
trajectory = test_trajectory(start, stop, map, path, true, show_plot);
if show_plot
    animate_frame(1); % close
    hold off
end