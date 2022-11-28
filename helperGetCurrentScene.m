function sceneName = helperGetCurrentScene()
%helperGetCurrentScene Get currently selected scene
%   sceneName = helperGetCurrentScene() queries the curent system to find
%   the selected scene. Note that this relies on the name of the
%   "Simulation 3D Config" block.
%
%   See also gcs, find_system.

% Copyright 2019 The MathWorks, Inc.

% Get the Simulation 3D Scene Configuration block
blockName = sprintf('Simulation 3D Scene Configuration');

configSystem = find_system(gcs, 'Name', blockName);

validateattributes(configSystem, {'cell'}, {'scalar'}, mfilename);

% Get the scene description parameter
sceneName = get_param(configSystem{1}, 'SceneDesc');

% Remove spaces
sceneName(isspace(sceneName)) = [];
end