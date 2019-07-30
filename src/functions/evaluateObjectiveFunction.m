function objFunc = evaluateObjectiveFunction(x, scanPoints, Poses, TMobile2TLS, numPoints, param)

% This function evaluates the objective function based on the computation
% of geometric 3D features and optionally the huber cost function.
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
%   30.07.2018


% Position and rodrigues to homogeneous calibration matrix
TCalib = PositionRodrigues2HomMatrix(x);

% Compute PointCloud from scanPoints, Poses and Relative Pose
ptCloud = rawData2PointCloud(scanPoints, Poses, TCalib, TMobile2TLS);

%% Downsample point cloud
PtCloudDown = pcdownsample(ptCloud, 'gridAverage', param.gridStep);

% At least three points are needed to determine a 3-D transformation
if ptCloud.Count < 50
    error('At least fifty unique points are needed to determine the geometrix attributes.');
end

%% Compute attributes of the current cloud
feature = computeOmnivariance(PtCloudDown.Location, param.numNeighbours);
% feature = computeFeature(PtCloudDown.Location, param.numNeighbours);

%% Compute residuals

% Remove outliers
keepInlierA = false(PtCloudDown.Count, 1); 
[~, idx] = sort(feature);
keepInlierA(idx(1:floor(numPoints))) = true;
objFunc = feature(keepInlierA);

% apply Huber function to residuals
if param.useHuber
    objFunc = HuberWeights(objFunc, param.huberK);
end

end