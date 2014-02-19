function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
figure(2)
xlim(map.boundary_dim([1 4]));
ylim(map.boundary_dim([2 5]));
zlim(map.boundary_dim([3 6]));
marg = map.margin;
hold on
for i = 1:length(map.block_dim(:,1))
% i = 1;  

b = map.block_dim(i,:);
%     x = [b(1) b(4)] + [marg -marg];
%     y = [b(2) b(5)] + [marg -marg];
%     z = [b(3) b(6)] + [marg -marg];
    x = [b(1) b(4)];
    y = [b(2) b(5)];
    z = [b(3) b(6)];
    col = b(7:9);
    verts = [];
    for xi = 1:2
        for yi = 1:2
            for zi = 1:2
                verts(end+1, 1:3) = [x(xi) y(yi) z(zi)];
            end
        end
    end
    faces = [1 3 4 2; 5 6 8 7;
             1 2 6 5; 3 4 8 7;
             1 3 7 5; 2 4 8 6];
   patch('Faces',faces,'Vertices',verts,'FaceColor',col./255);  
%    hold on;
end

plot3(path(:,1), path(:,2), path(:,3), 'g-');
hold off;
end