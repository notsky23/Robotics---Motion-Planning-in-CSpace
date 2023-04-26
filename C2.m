% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    % Initialize the configuration space
    cspace = zeros(length(q_grid), length(q_grid));
    
    % Loop over the grid points
    for i = 1:length(q_grid)
        for j = 1:length(q_grid)
            % Compute the polygon of the robot at the current grid point
            [poly1, poly2, pivot1, pivot2] = q2poly(robot, [q_grid(i), q_grid(j)]);
    
            % Check for collision with each obstacle
            for k = 1:length(obstacles)
        
                % Get the polygon of the current obstacle
                inter1 = intersect(poly1, obstacles(k));
                inter2 = intersect(poly2, obstacles(k));
        
                % Exit the loop if collision is detected
                if inter1.NumRegions > 0 || inter2.NumRegions >0
                    cspace(i,j) = 1;
                    break;
                end
            end
        end
    end
end