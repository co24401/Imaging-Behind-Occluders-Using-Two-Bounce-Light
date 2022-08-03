n_smooth = zeros(size(n));

for ii = 1:size(n, 1)
    col = find(fv.faces(:, 1)==ii);
    for jj = 1:length(col)
        n_smooth(ii, :) = n_smooth(ii, :) + n(fv.faces(col(jj), 2), :) + n(fv.faces(col(jj), 3), :);
    end

    col = find(fv.faces(:, 2)==ii);
    for jj = 1:length(col)
        n_smooth(ii, :) = n_smooth(ii, :) + n(fv.faces(col(jj), 1), :) + n(fv.faces(col(jj), 3), :);
    end
    
    col = find(fv.faces(:, 3)==ii);
    for jj = 1:length(col)
        n_smooth(ii, :) = n_smooth(ii, :) + n(fv.faces(col(jj), 1), :) + n(fv.faces(col(jj), 2), :);
    end
end

smooth_norms = sqrt(n_smooth(:, 1).^2 + n_smooth(:, 2).^2 + n_smooth(:, 3).^2);
n_smooth = n_smooth./smooth_norms;