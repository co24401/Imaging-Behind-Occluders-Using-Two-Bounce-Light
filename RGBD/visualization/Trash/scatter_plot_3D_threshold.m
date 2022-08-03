function [] = scatter_plot_3D_threshold(voxel, threshold)
    voxel_x_size = (voxel.x_lims(2) - voxel.x_lims(1))/voxel.num_x;
    voxel_y_size = (voxel.y_lims(2) - voxel.y_lims(1))/voxel.num_y;
    voxel_z_size = (voxel.z_lims(2) - voxel.z_lims(1))/voxel.num_z;
    
    voxel.volume = voxel.volume <= threshold;
    fill_indices = find (voxel.volume == 1);
    [xx, yy, zz] = ind2sub(size(voxel.volume), fill_indices);
    xx = voxel.x_lims(1) + voxel_x_size * xx;
    yy = voxel.y_lims(1) + voxel_y_size * yy;
    zz = voxel.z_lims(1) + voxel_z_size * zz;
    
    scatter3(xx, zz, -yy, 100, '.')
    
    xlim(voxel.x_lims)
    ylim(voxel.y_lims)
    zlim(voxel.z_lims)
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis equal
    view(-87, -13)

end