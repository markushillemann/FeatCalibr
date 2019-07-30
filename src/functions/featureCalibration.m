function [TCalibOptim,ptCloudOptim,ptCloudIni,Err,c_xx] = featureCalibration(TCalibIni, scanPointsKart, Poses, TMobile2World, param, optimOptions)

% This is the main function of the extrinsic self-calibration. 
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


% Unknowns: 6 calibration parameters
x = zeros(6,1);
x(1:3) = Rodrigues2(TCalibIni(1:3,1:3));
x(4:6) = TCalibIni(1:3,4);
numUnknowns = size(x,1);

%% extrinsic self-calibration
tic

fprintf('Optimizing calibration parameters. This may take a while.\n');
fprintf('Recursive Calibration...\n');

% get current point cloud size
ptCloudIni = rawData2PointCloud(scanPointsKart, Poses, TCalibIni, TMobile2World);
ptCloudDown = pcdownsample(ptCloudIni, 'gridAverage', param.gridStep);
numPointsCurrent = ptCloudDown.Count;

% Start with 50% of the points
percent = 0.5;
numObservations = percent*numPointsCurrent;

iScale = 1;
while iScale <= param.numScales
    
    try
        % Optimization
        [x,~,objFunc,~,~,~,jacobian] = lsqnonlin(@evaluateObjectiveFunction,x,[],[],optimOptions, ...
            scanPointsKart, Poses, TMobile2World, numObservations, param);
    catch err
        if strcmp(err.identifier, 'MATLAB:badsubscript')
            percent = percent - 0.05;
            fprintf('The number of observations was too large. Try again with %2.0d%% of the points.\n', uint8(100*percent));
            numObservations = percent*numPointsCurrent;
            continue;
        else
            error(err.message);
        end
    end
    
    % Optimized Calibration
    TCalibOptim = PositionRodrigues2HomMatrix(x);
    ptCloudOptim = rawData2PointCloud(scanPointsKart, Poses, TCalibOptim, TMobile2World);
    
    % adjust gridStep
    param.gridStep = param.gridStep/param.scaleFactor;
    % reset percent
    percent = 0.5;
    
    % Update numPointsCurrent
    ptCloudDown = pcdownsample(ptCloudOptim, 'gridAverage', param.gridStep);
    numPointsCurrent = ptCloudDown.Count;
    
    iScale = iScale + 1;
end

t_optim = toc;
fprintf('Optimization took: %d', t_optim);

%% optional results

if nargout > 3
    Err = objFunc;
end

if nargout > 4
    % compute some statistics
    q_xx=jacobian'*jacobian;
    s0_2 = (objFunc'*objFunc)/(length(objFunc)-numUnknowns);
    c_xx = s0_2 .* eye(numUnknowns) / q_xx;
end

end