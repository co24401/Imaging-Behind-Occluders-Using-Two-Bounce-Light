function [volume, probability_volume] = visualize_probablistic(inside_voxel, outside_voxel ,PO, T)
    inside_volume = inside_voxel.volume;
    outside_volume = outside_voxel.volume;
    volume = zeros(size(inside_volume));
    probability_volume = zeros(size(inside_volume));
    for ii = 1:size(inside_volume,1)
        for j = 1:size(inside_volume,2)
            for k = 1:size(inside_volume,3)
                probability_volume(ii,j,k) = PO(round(inside_volume(ii,j,k))+1, round(outside_volume(ii,j,k))+1);
                if PO(round(inside_volume(ii,j,k))+1, round(outside_volume(ii,j,k))+1) > T
                    volume(ii,j,k) = 1;
                end
            end
        end
    end
    
    voxel_x_size = (inside_voxel.x_lims(2) - inside_voxel.x_lims(1))/inside_voxel.num_x;
    voxel_y_size = (inside_voxel.y_lims(2) - inside_voxel.y_lims(1))/inside_voxel.num_y;
    voxel_z_size = (inside_voxel.z_lims(2) - inside_voxel.z_lims(1))/inside_voxel.num_z;
    fill_indices = find (volume == 1);
    [xx, yy, zz] = ind2sub(size(inside_voxel.volume), fill_indices);
    xx = inside_voxel.x_lims(1) + voxel_x_size * xx;
    yy = inside_voxel.y_lims(1) + voxel_y_size * yy;
    zz = inside_voxel.z_lims(1) + voxel_z_size * zz;
    
    scatter3(xx, zz, -yy, 100, '.')
    
    xlim(inside_voxel.x_lims)
    ylim(inside_voxel.y_lims)
    zlim(inside_voxel.z_lims)
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis equal
    view(-87, -13)

end

