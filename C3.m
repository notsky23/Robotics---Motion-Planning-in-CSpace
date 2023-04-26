% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    % Initialize variables
    distances = cspace;
    goal_idx = [find(q_grid <= q_goal(1), 1, 'last'), find(q_grid <= q_goal(2), 1, 'last')];
    distances(goal_idx(1), goal_idx(2)) = 2;

%     goalList = goal_idx;

    % Apply fast marching method to compute the distance transform
    step = 3;
    while size(goal_idx, 1) >= 1
        g = goal_idx(1,:);
        goal_idx(1,:) = [];
        
        neighbors = [g(1)-1, g(2)-1; g(1)-1, g(2); g(1)-1, g(2)+1; g(1), g(2)-1; g(1), g(2)+1; g(1)+1, g(2)-1; g(1)+1, g(2); g(1)+1, g(2)+1];
        neighbors = unique(neighbors,'rows');
    
        for k = 1:size(neighbors, 1)
            i = neighbors(k, 1);
            j = neighbors(k, 2);
            if i >= 1 && i <= size(distances,1) && j >= 1 && j <= size(distances,2) && distances(i, j) == 0
                distances(i, j) = step;
                goal_idx(end+1,:)= [i,j];
            end
        end
        step = step + 1;
    end
    
    % Set unreachable cells to 0
    distances(cspace == 1) = 0;

end