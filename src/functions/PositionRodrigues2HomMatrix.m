function HomMatrix = PositionRodrigues2HomMatrix(PositionRodrigues)

% This function reparametrizes a pose from a rodrigues vector and a
% translation to a homogeneous matrix.
%
% INPUT
%   PositionRodrigues - [6x1] vector representing a pose: 
%                       [r1, r2, r3, t1, t2, t3]
%
% OUTPUT
%   HomMatrix         - [4x4] matrix representing the same pose.
%                       [R,t
%                        0,1]
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

rMCS2RB = [PositionRodrigues(1); PositionRodrigues(2); PositionRodrigues(3)];
tMCS2RB = [PositionRodrigues(4); PositionRodrigues(5); PositionRodrigues(6)];
HomMatrix = [Rodrigues2(rMCS2RB) tMCS2RB; 0 0 0 1];

end