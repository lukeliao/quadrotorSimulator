close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');

nmap = 3;

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
end


nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

% %% Additional init script
hold on;
init_script;
hold off;
% %% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
% testtraj;