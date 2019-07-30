function R2 = Rodrigues2(R1)

% This function reparametrizes a rodrigues vector to a rotation matrix
% or vice versa.
%
% INPUT
%   R1 - 3x1 or 1x3 vector representing a rodrigues vector. 
%        or
%        3x3 matrix representing a rotation matrix.
%
% OUTPUT
%   R2 - 3x3 matrix representing a rotation matrix.
%        or
%        3x1 vector representing a rodrigues vector.
%
% LICENSE
%   Copyright (C) 2018  Markus Hillemann
%
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License
%   as published by the Free Software Foundation; either version 3
%   of the License, or (at your option) any later version.
% 
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
%   GNU General Public License for more details.
% 
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, see <http://www.gnu.org/licenses/>. 
%
% CONTACT
%   Markus Hillemann
%   Institute of Photogrammetry and Remote Sensing,
%   Karlsruhe Institute of Technology (KIT)
%   email:   markus.hillemann@kit.edu
%
% DATE
%   30.08.2018

[r,c] = size(R1);

%% Rodrigues Rotation Vector to Rotation Matrix
if ((r == 3) && (c == 1)) || ((r == 1) && (c == 3))
    wx = [  0   -R1(3)  R1(2);
           R1(3)   0   -R1(1);
          -R1(2)  R1(1)   0   ];
      
    R1_norm = sqrt(R1(1)^2 + R1(2)^2 + R1(3)^2);
    
    if (R1_norm < eps)
        R2 = eye(3);
    else
        R2 = eye(3) + sin(R1_norm)/R1_norm*wx + (1-cos(R1_norm))/R1_norm^2*wx^2;
    end

%% Rotation Matrix to Rodrigues Rotation Vector
elseif (r == 3) && (c == 3)
    w_norm = acos((trace(R1)-1)/2);
    if (w_norm < eps)
        R2 = [0 0 0]';
    else
        R2 = 1/(2*sin(w_norm))*[R1(3,2)-R1(2,3);R1(1,3)-R1(3,1);R1(2,1)-R1(1,2)]*w_norm;
    end
end


