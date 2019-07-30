function feature = computeFeature(XYZ,featureName,numNeighbours)
% THIS IS A MODYFIED VERSION OF "geoFEX (geometric Feature EXtraction)"
% DESCRIBED IN [Weinmann et al., 2015] AND PUBLISHED ON
% https://www.ipf.kit.edu/code.php#3d_scene
%
% DESCRIPTION
%   For each point in the point cloud, this function derives basic
%   geometric properties of the respective local neighborhood by using
%   the respective number of nearest neighbors. More
%   details can be found in [Weinmann et al., 2015]
%
% INPUT VARIABLES
%   XYZ           -   matrix containing the point coordinates [n x 3]
%   numNeighbours -   vector containing the number of neighbors for each
%                     3D point
%   featureName   -   String containing the name of the feauture to use.
%                     One of the following strings:
%                     - linearity
%                     - planarity
%                     - scattering
%                     - omnivariance
%                     - eigenentropy
%                     - changeOfCurvature
%
% OUTPUT VARIABLES
%   feature    -   vector containing the values of the selected redefined 
%                  feature
%
% LITERATURE
%   [Pauly et al., 2003]    - Pauly, M., Keiser, R., Gross, M., 2003.
%                             Multi-scale feature extraction on
%                             point-sampled surfaces. Computer Graphics
%                             Forum 22 (3), 281–289.
%   [West et al., 2004]     - West, K.F., Webb, B.N., Lersch, J.R.,
%                             Pothier, S., Triscari, J.M.,
%                             Iverson, A.E., 2004. Context-driven automated
%                             target detection in 3-d data, in: Proceedings
%                             of SPIE 5426, Automatic Target Recognition
%                             XIV, SPIE, pp. 133–143.
%   [Rusu et al., 2009]     - Rusu, R.B., Blodow, N., Beetz, M., 2009.
%                             Fast point feature histograms (FPFH) for 3d
%                             registration, in: Proceedings of the IEEE
%                             International Conference on Robotics and
%                             Automation, IEEE, pp. 3212–3217.
%   [Mallet et al., 2011]   - Mallet, C., Bretar, F., Roux, M.,
%                             Soergel, U., Heipke, C., 2011. Relevance
%                             assessment of full-waveform lidar data for
%                             urban area classification. ISPRS Journal of
%                             Photogrammetry and Remote Sensing 66 (6),
%                             S71–S84.
%   [Weinmann et al., 2015] - M. Weinmann, S. Urban, S. Hinz, B. Jutzi,
%                             and C. Mallet (2015): Distinctive 2D and 3D
%                             features for automated large-scale scene
%                             analysis in urban areas. Computers &
%                             Graphics, Vol. 49, pp. 47-57.
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


% get point IDs
numPoints = size(XYZ,1);

% get local neighborhoods consisting of k neighbors (the maximum k value is chosen here in order to conduct knnsearch only once)
numLocalPoints = numNeighbours+1;
[neighbourIndices, ~] = knnsearch(XYZ,XYZ,'Distance','euclidean','NSMethod','kdtree','K',numLocalPoints);

% devide pointCloud into voxels

% do some initialization stuff for speed improvement
% EVs = zeros(numPoints,3);
% epsilon2Add = 1e-8;
feature = zeros(numPoints,1);

% loop over all 3D points
parfor iPoint=1:numPoints
    
    % select neighboring points
    P = XYZ(neighbourIndices(iPoint,:),:);

    % calculate covariance matrix C
    P = P-ones(numLocalPoints,1)*(sum(P,1)/numLocalPoints);
    C = P'*P./numNeighbours;
       
    % EVs(iPoint,:) = svd(C); % this is slower even if the EVs are always >0.
    % EVs(iPoint,:) = svd(C,0); % this is also slower
    
    C = C ./ (C(1,1)+C(2,2)+C(3,3));    
    feature(iPoint) = det(C);
    
    % get the eigenvalues of C (sorting is already done by Matlab routine eig)
    % ... and remove negative eigenvalues (NOTE: THESE APPEAR ONLY BECAUSE OF NUMERICAL REASONS AND ARE VERY VERY CLOSE TO 0!)
    % ... and later avoid NaNs resulting for eigenentropy if one EV is 0
%     EV = eig(C);
%     EV3 = EV(1); EV2 = EV(2); EV1 = EV(3);
%     
%     if EV3 <= 0; EV3 = epsilon2Add;
%         if EV2 <= 0; EV2 = epsilon2Add;
%             if EV1 <= 0; EV1 = epsilon2Add; end
%         end
%     end
%     
%     EVs(iPoint,:) = [EV1, EV2, EV3];
        
end  % iPoint

% normalization of eigenvalues
sumEVs = sum(EVs,2);
EVs(:,1) = EVs(:,1) ./ sumEVs;
EVs(:,2) = EVs(:,2) ./ sumEVs;
EVs(:,3) = EVs(:,3) ./ sumEVs;

% feature: omnivariance
feature = ( EVs(:,1) .* EVs(:,2) .* EVs(:,3) ).^(1/3);

switch featureName
    
        % Now, get eigenvalue-based features by vectorized calculations:
        % 1.) properties of the structure tensor according to [West et al.,
        %     2004; Mallet et al., 2011]
    case 'linearity'
        feature = 1 - (( EVs(:,1) - EVs(:,2) ) ./ EVs(:,1));
    case 'planarity'
        feature = 1 - (( EVs(:,2) - EVs(:,3) ) ./ EVs(:,1));
    case 'sphericity'
        feature = EVs(:,3) ./ EVs(:,1);
    case 'omnivariance'
        feature = ( EVs(:,1) .* EVs(:,2) .* EVs(:,3) ).^(1/3);
    case 'eigenentropy'
        feature = ( EVs(:,1).*log(EVs(:,1)) + EVs(:,2).*log(EVs(:,2)) + EVs(:,3).*log(EVs(:,3)) ) ./ (1/3*(log(EVs(:,1)) + log(EVs(:,2)) + log(EVs(:,3))));
        
        % 2.) get surface variation, i.e. the change of curvature 
        %     [Pauly et al., 2003; Rusu, 2009], namely the variation of a
        %     point along the surface normal (i.e. the ratio between the
        %     minimum eigenvalue and the sum of all eigenvalues
        %     approximates the change of curvature in a neighborhood
        %     centered around this point; note this ratio is invariant
        %     under rescaling)
    case 'changeOfCurvature'
        feature = EVs(:,3) ./ ( EVs(:,1) + EVs(:,2) + EVs(:,3) );
    otherwise
        error('Feature name must be one of the following character arrays: ''linearity'', ''planarity'', ''sphericity'', ''omnivariance'', ''eigenentropy'', ''changeOfCurvature''. ');
end

end

