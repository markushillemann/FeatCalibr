% This is the main script that loads example data and calls the extrinsic
% self-calibration function
%
% TODO:
% Details can be found in the following submission:
%     M. Hillemann, M. Weinmann, M. S. Mueller, B. Jutzi:
%     Automatic Extrinsic Self-Calibration of Mobile Mapping Systems
%     Based on Geometric 3D Features
%
% LICENSE
%   Copyright (C) 2019  Markus Hillemann
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

format longG;

addpath('../data');
addpath('./functions');

%% Check for toolboxes
if ~license('checkout', 'Video_and_Image_Blockset')
    error('Computer Vision System Toolbox is required.')
elseif ~license('checkout', 'Optimization_Toolbox')
    error('Optimization Toolbox is required.')
elseif ~license('checkout', 'Statistics_Toolbox')
    error('Statistics and Machine Learning Toolbox is required.')
end

% Optional
global useParallel;
useParallel = true;
if ~license('checkout', 'Distrib_Computing_Toolbox')
    useParallel = false;
end

%% load data
% Required:
% Poses: 4x4xN Matrix. N homogeneous pose matrices of the form:
%        [R(3x3), t(3x1);
%         0,  0,  0,  1 ]
% scanPoints: Mx4xN Matrix. M homogeneous laserscanner points per pose

% Optional:
% TMobile2World: 4x4 Matrix. Rigid transformation from odometry frame to
% world frame. Rotates the point cloud such that the walls are parallel
% to the axes of the world frame. This matrix is not used for calibration,
% Just for display reasons.

load('exampleData.mat');

if ~exist('TMobile2World', 'var')
    TMobile2World = eye(4);
end

%% Set parameters

% parameter for research purposes
% parameters.featureName = 'omnivariance'; % Name of the feature to use.
% Must be one of the following character arrays: linearity, planarity,
% sphericity, omnivariance, eigenentropy, changeOfCurvature
% omnivariance achieved best results in our experiments, eigenentropy second best.
% to be able to adjust the feature one needs to comment line 47 in
% functions/evaluateObjectiveFunction.m and uncomment line 48 in
% functions/evaluateObjectiveFunction.m.

parameters.gridStep = 0.3; % Size of the voxel grid filter. Finer is more precise, but less robust.

parameters.numScales = 2; % Number of scales. Higher is more robust to initial calibration errors, but slower.
% 4 should be enough in most cases. For good initial calibrations even 1 might be enough.

parameters.scaleFactor = 5; % Factor between two scales. Value greater than 1. Should be higher for a lower number of scales to ensure accurate results.

parameters.numNeighbours = 50; % Number of neighbours, that are considered for the computation of the feature. Can be adjusted alternatively to gridStep. We recommend to adjust gridStep and to keep a number neighbours of 50.

parameters.useHuber = 1; % boolean. 1: robust cost function is used (m-estimator huberK). 0: common nonlinear least squares
% Should be set to 1 if there is much vegetation or other scattering objects in the scene.

parameters.huberK = 0.2; % tuning constant of huberK estimator. Adjustig this parameter only has impact if useHuber is set to 1. Depends on the feature:
% For omnivariance and eigenentropy this parameter should be between 0 and 1/3, for all other features between 0 and 1.

parameters.optimizeJustRotation = 0; % boolean. 0: All extrinsic calibration parameters are optimized. 1: Just the rotation parameters (bore-sight) are optimized.
% Set this parameter to 1 if the translation parameters (leverarm) are negligible or if you already know the exact parameters

% optimization parameters
optimOptions = optimset('TolFun', 1e-6, ...
                        'Display', 'Iter', ...
                        'TolX', 1e-6, ...
                        'MaxIter', 5);

if useParallel
    optimOptions.UseParallel = true;
else
    optimOptions.UseParallel = false;
end

%% set initial calibration parameters

% adjust this!

% rough guess of initial translation parameters
if parameters.optimizeJustRotation 
    % we already know the exact translation parameters (leverarm)
    translation = [0.14251    0.067203    0.086904]';
else
    % we only have a rough guess of the translation (leverarm)
    translation = [0,0,0]';
end

% rough guess of initial rotation parameters
rotation = deg2rad([0, -60, 0]); % axis-angle representation

% homogeneous transformation matrix of calibration parameters
TCalibIni = [Rodrigues2(rotation), translation;
    0,    0,    0,             1     ];

%% Compute initial point cloud
ptCloudIni = rawData2PointCloud(scanPoints, Poses, TCalibIni, TMobile2World);

%% Show the initial point cloud

% Downsample for visualization
ptCloudIniDown = pcdownsample(ptCloudIni, 'gridAverage', 0.05);

% Compute feature for visualization
ftOptimCalib = computeOmnivariance(ptCloudIniDown.Location, parameters.numNeighbours);
% ftOptimCalib = computeFeature(ptCloudOptimDown.Location, parameters.featureName, parameters.numNeighbours);

% show the initial cloud
subplot(2,1,1);
pcshow(ptCloudIniDown.Location, lin2rgbLinear(ftOptimCalib));
title('Initial point cloud');
axis off
view(0,0)

drawnow;

%% ------- Call the calibration function -------------
[TCalib, ptCloudOptim, ptCloudIni] = featureCalibration(...
    TCalibIni, ...
    scanPoints, ...
    Poses, ...
    TMobile2World, ...
    parameters, ...
    optimOptions);

%% Display Result
disp(' ');
disp(' ----------  Result  ---------');
disp(' ');
disp('Initial calibration: ');
disp(['r in deg: ', num2str(rad2deg(Rodrigues2(TCalibIni(1:3,1:3))')), ' (axis-angle parametrization)']);
disp(['t in m: ', num2str(TCalibIni(1:3,4)')]);

disp(' ');
disp('Optimized calibration: ');
disp(['r in deg: ', num2str(rad2deg(Rodrigues2(TCalib(1:3,1:3))')), ' (axis-angle parametrization)']);
disp(['t in m: ', num2str(TCalib(1:3,4)')]);

%% Show final cloud colored by the chosen feature value
% Downsample for visualization
ptCloudOptimDown = pcdownsample(ptCloudOptim, 'gridAverage', 0.05);

% Compute feature values for visualization
ftOptimCalib = computeOmnivariance(ptCloudOptimDown.Location, parameters.numNeighbours);
% ftOptimCalib = computeFeature(ptCloudOptimDown.Location, parameters.featureName, parameters.numNeighbours);

% Show point cloud
subplot(2,1,2);
pcshow(ptCloudOptimDown.Location, lin2rgbLinear(ftOptimCalib));
title('Point cloud after calibration');
axis off
view(0,0)

drawnow;


