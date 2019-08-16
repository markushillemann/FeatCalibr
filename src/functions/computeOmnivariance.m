function feature = computeOmnivariance(XYZ,numNeighbours)
% DESCRIPTION
%   For each point in the point cloud, this function derives the
%   omnivariance of the respective local neighborhood.
%
% INPUT VARIABLES
%   XYZ           -   matrix containing the point coordinates [n x 3]
%   numNeighbours -   vector containing the number of neighbors for each
%                     3D point
%
% OUTPUT VARIABLES
%   feature    -   vector containing the values of the selected redefined 
%                  feature
%
% LITERATURE
%   [West et al., 2004]     - West, K.F., Webb, B.N., Lersch, J.R.,
%                             Pothier, S., Triscari, J.M.,
%                             Iverson, A.E., 2004. Context-driven automated
%                             target detection in 3-d data, in: Proceedings
%                             of SPIE 5426, Automatic Target Recognition
%                             XIV, SPIE, pp. 133–143.
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

global useParallel;

% get point IDs
numPoints = size(XYZ,1);

% get local neighborhoods consisting of k neighbors (the maximum k value is chosen here in order to conduct knnsearch only once)
numLocalPoints = numNeighbours+1;
[neighbourIndices, ~] = knnsearch(XYZ,XYZ,'Distance','euclidean','NSMethod','kdtree','K',numLocalPoints);

% initialization
feature = zeros(numPoints,1);

% loop over all 3D points
if useParallel
    parfor iPoint=1:numPoints

        % select neighboring points
        P = XYZ(neighbourIndices(iPoint,:),:);

        % calculate covariance matrix C
        P = P-ones(numLocalPoints,1)*(sum(P,1)/numLocalPoints);
        C = P'*P./numNeighbours;

        % instead of calculating the eigenvalues we can use the determinant of
        % the covariance matrix to determine the omnivariance (this is much
        % faster)
        C = C ./ (C(1,1)+C(2,2)+C(3,3));    
        feature(iPoint) = det(C);

    end  % iPoint
else % no Parallel Computing Toolbox
    for iPoint=1:numPoints

        % select neighboring points
        P = XYZ(neighbourIndices(iPoint,:),:);

        % calculate covariance matrix C
        P = P-ones(numLocalPoints,1)*(sum(P,1)/numLocalPoints);
        C = P'*P./numNeighbours;

        % instead of calculating the eigenvalues we can use the determinant of
        % the covariance matrix to determine the omnivariance (this is much
        % faster)
        C = C ./ (C(1,1)+C(2,2)+C(3,3));    
        feature(iPoint) = det(C);

    end  % iPoint
end

% feature: omnivariance
feature = feature.^(1/3);
    
end

