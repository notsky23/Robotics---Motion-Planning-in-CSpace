% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    % Initialize padded_cspace with the same values as cspace
    [row, col] = size(cspace);
    padded_cspace = zeros(row, col);

    % Loop through each cell in cspace
    for i = 1:row
        for j = 1:col
            % If the current cell is in collision, mark all its neighbors
            % (including diagonal neighbors) as in collision as well
            if cspace(i, j) == 1
                padded_cspace(i, j) = 1;

%                 if i > 1
%                     padded_cspace(i-1, j) = 1;
%                 end
                if i < 100
                    padded_cspace(i+1, j) = 1;
                end
%                 if j > 1
%                     padded_cspace(i, j-1) = 1;
%                 end
                if j < 100
                    padded_cspace(i, j+1) = 1;
                end

                if i < 100 && j < 100
                    padded_cspace(i+1, j+1) = 1;
                end

                if j > 1 && i < 100
                    padded_cspace(i+1, j-1) = 1;
                end

                continue
            end

        end
    end
end