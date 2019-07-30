function rRobust = HuberWeights(r,varargin)

% This function weights the values in r based on a robust m-estimator,
% namely the huber cost function [Huber, 2011].
%
% INPUT
%   r       - 1xN or Nx1 vector representing residuals.
%
% OUTPUT
%   rRobust - Nx1 vector representing the weighted residuals.
%
% LITERATURE
%   [Huber, 2011] - P. J. Huber (2011): Robust statistics. International
%                   Encyclopedia of Statistical Science, pp. 1248--1251
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


if nargin==1
    k = 1.345;
elseif nargin==2
    k = varargin{1};
else
    error('Wrong number of input variables for function HuberWeights.')
end
        
a = abs(r) < k;
b = abs(r) >= k;
rRobust(a) = 1/2 * r(a).^2;
rRobust(b) = k.*(abs(r(b)) - 1/2*k);
rRobust = rRobust';     

end
