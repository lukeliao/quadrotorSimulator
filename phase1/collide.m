function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

min_m = map.boundary_dim(1:3);
max_m = map.boundary_dim(4:6);

C = any(bsxfun(@or, bsxfun(@gt, min_m,points), bsxfun(@lt, max_m, points)),2);
  
for i = 1:length(map.block_dim(:,1))
    min_b = map.block_dim(i,1:3) - map.margin - map.body_buffer;
    max_b = map.block_dim(i,4:6) + map.margin + map.body_buffer;
    block_result = all(bsxfun(@and, bsxfun(@ge, max_b,points), bsxfun(@le, min_b, points)),2);
    C = bsxfun(@or, C, block_result);
end
    
