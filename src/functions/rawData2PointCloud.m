function ptCloud = rawData2PointCloud(scanPoints, Poses, TCalib, TOdom2World)

% This function transforms 3D-points from the laserscanner coordinate
% frame to the odometry frame. The result is transformed to the world
% frame. The transformed 3D-points are stored in a pointCloud object.
%
% INPUT
%   scanPoints  - Mx4xN Matrix. M homogeneous laserscanner points per pose
%   Poses       - 4x4xN Matrix. N homogeneous pose matrices of the form:
%                 [R(3x3), t(3x1);
%                  0,  0,  0,  1 ]
%   TCalib      - 4x4 homogeneous transformation matrix representing the
%                 extrinsic calibration. Euclidean transformation from the
%                 laserscanner frame to the odometry frame.
%   TOdom2World - 4x4 homogeneous transformation matrix representing the
%                 Euclidean transformation from the odometry frame to the
%                 world frame. This is and optional transformation that
%                 has no impact on the calibration result. Use eye(4) if
%                 you dont need it.
%
% OUTPUT
%   ptCloud     - pointCloud object representing the 3D point cloud in
%                 world coordinates. 
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


% transform scanPoints to world/odometry frame
scanPointsInWorld = zeros(size(scanPoints,1),4,size(scanPoints,3));
for iImg = 1:size(scanPoints,3)
    scanPointsInWorld(:,:,iImg) = (Poses(:,:,iImg)*(TCalib*scanPoints(:,:,iImg)'))' * TOdom2World;
end

% prepare scanPoints matrix for pcshow
ptCloud = scanLines2PointCloud(scanPointsInWorld);

end