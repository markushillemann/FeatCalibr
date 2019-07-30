function rgb = lin2rgbLinear(lin)

% This function maps linear values to rgb values using a colormap and
% linear interpolation.
%
% INPUT VARIABLES
%   lin - m-by-1 Vector or 1-by-m vector
%
% OUTPUT VARIABLES
%   rgb - m-by-3 Matrix
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


if size(lin,1) ~= 1 && size(lin,2) ~= 1
    error('Input must be a vector');
end

m = length(lin(:));

cmapSize = 256;
cmap = jet(cmapSize);

linMax = max(lin(:));
linMin = min(lin(:));

%     linMax = 1;
%     linMin = 0;

% linear interpolation
idxs = round(1 + ((cmapSize-1)/(linMax-linMin)) * (lin - linMin));

rgb = zeros(m, 3);
for i=1:m
    rgb(i,:) = cmap(idxs(i),:);
end

end