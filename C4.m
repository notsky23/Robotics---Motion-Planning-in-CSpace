% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    % Find the grid cell containing q_start
    start_cell = [find(q_grid <= q_start(1), 1, 'last'), find(q_grid <= q_start(2), 1, 'last')];
    
    % Find the goal cell by searching for the cell with value 2
    [goal_row, goal_col] = find(distances == 2);
    goal_cell = [goal_row(1), goal_col(1)];
    
    % Initialize the path with the start cell
    path = start_cell;
    
    % Define a set of relative neighbors, including diagonal moves
    neighbors = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    
    % Greedily descend the distance transform until the goal is reached
    while ~isequal(path(end, :), goal_cell)
        curr_cell = path(end, :);
        curr_dist = distances(curr_cell(1), curr_cell(2));
        
        % Find the neighboring cells with the lowest distance
        next_cells = [];
        next_dists = [];
        for i = 1:size(neighbors, 1)
            neighbor = curr_cell + neighbors(i, :);
            if neighbor(1) >= 1 && neighbor(1) <= size(distances, 1) && neighbor(2) >= 1 && neighbor(2) <= size(distances, 2)
                neighbor_dist = distances(neighbor(1), neighbor(2));
                if neighbor_dist > 1 && neighbor_dist < curr_dist
                    next_cells = [next_cells; neighbor];
                    next_dists = [next_dists; neighbor_dist];
                end
            end
        end
        
        % Break ties by choosing the first neighbor with the lowest distance
        if ~isempty(next_cells)
            [~, idx] = min(next_dists);
            path = [path; next_cells(idx, :)];
        else
            break;
        end
    end

end
