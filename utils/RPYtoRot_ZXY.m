function R = RPYtoRot_ZXY(phi,theta,psi)
%RPYtoRot_ZXY Converts roll, pitch, yaw to a body-to-world Rotation matrix
%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose this matrix to get the body to world [wRb] such that
%   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
%   is a point in the world frame
%   written by Daniel Mellinger
%

R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), ...
     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), ...
     -cos(phi)*sin(theta); ...
     -cos(phi)*sin(psi),...
     cos(phi)*cos(psi), ...
     sin(phi);...
     cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),...
     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),...
     cos(phi)*cos(theta)];
 
end