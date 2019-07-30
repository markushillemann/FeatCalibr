function ptCloud = scanLines2PointCloud(scans)

% This function converts a multidimensional array with 3D points to a
% pointCloud object.
%
% INPUT
%   scans   - Mx3xN Matrix representing the 3D-points. M homogeneous
%               laserscanner points per pose. N poses.
%
% OUTPUT
%   ptCloud - pointCloud object representing the scans
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


numPointsPerScanLine = size(scans,1);
numScanLines = size(scans,3);

pointsXYZ = zeros(numScanLines*numPointsPerScanLine,3);
for iScanLine = 1:numScanLines
    startIdx = (iScanLine-1)*numPointsPerScanLine+1;
    endIdx = startIdx+numPointsPerScanLine-1;
    pointsXYZ(startIdx:endIdx,:) = scans(:,1:3,iScanLine);
end

ptCloud = pointCloud(pointsXYZ);


end