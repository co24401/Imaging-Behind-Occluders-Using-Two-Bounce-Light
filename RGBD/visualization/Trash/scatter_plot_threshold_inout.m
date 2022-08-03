function [] = scatter_plot_threshold_inout(inside_voxel, outside_voxel, inside_threshold, outside_threshold)
    voxel_x_size = (inside_voxel.x_lims(2) - inside_voxel.x_lims(1))/inside_voxel.num_x;
    voxel_y_size = (inside_voxel.y_lims(2) - inside_voxel.y_lims(1))/inside_voxel.num_y;
    voxel_z_size = (inside_voxel.z_lims(2) - inside_voxel.z_lims(1))/inside_voxel.num_z;
    
    volume = (inside_voxel.volume >= inside_threshold) .* (outside_voxel.volume <= outside_threshold);
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