% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    % Initialize Variables
    N = size(q_path, 1);
    num_collisions = 0;

    % Set a small tolerance value for comparing vertex locations
    tolerance = 1e-10;

    for i = 1:N-1
        [poly1_i, poly2_i, pivot1_i, pivot2_i] = q2poly(robot, q_path(i,:));
        [poly1_j, poly2_j, pivot1_j, pivot2_j] = q2poly(robot, q_path(i+1,:));
    
        % Compute the swept volume
        poly1_i_vertices = poly1_i.Vertices;
        poly2_i_vertices = poly2_i.Vertices;
        poly1_j_vertices = poly1_j.Vertices;
        poly2_j_vertices = poly2_j.Vertices;
        
        swept_vertices = [poly1_i_vertices; poly2_i_vertices; poly1_j_vertices; poly2_j_vertices];
        swept_vertices_x = swept_vertices(:,1);
        swept_vertices_y = swept_vertices(:,2);

        K = convhull(swept_vertices_x, swept_vertices_y);
        swept_poly = polyshape(swept_vertices_x(K), swept_vertices_y(K));
    
        % Check for collisions with obstacles
        for k = 1:length(obstacles)
            % Get the polygon of the current obstacle
            inter1 = intersect(swept_poly, obstacles(k));

            % Check if the center of the swept polygon is within the obstacle polygon
            center = centroid(swept_poly);

            if inter1.NumRegions > 0
                num_collisions = num_collisions + 1;
                
                % Plot the links
                plot(polyshape(poly1_i_vertices(:,1), poly1_i_vertices(:,2)), 'FaceColor', 'r');
                plot(polyshape(poly2_i_vertices(:,1), poly2_i_vertices(:,2)), 'FaceColor', 'b');

                % Plot the pivot points
                plot(pivot1_i(1), pivot1_i(2), 'k.', 'MarkerSize', 10);
                plot(pivot2_i(1), pivot2_i(2), 'k.', 'MarkerSize', 10);

                break;
            end
        end
    end

end