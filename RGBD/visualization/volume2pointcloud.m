function point_cloud = volume2pointcloud(voxel)

    voxel_x_size = (voxel.x_lims(2) - voxel.x_lims(1))/voxel.num_x;
    voxel_y_size = (voxel.y_lims(2) - voxel.y_lims(1))/voxel.num_y;
    voxel_z_size = (voxel.z_lims(2) - voxel.z_lims(1))/voxel.num_z;
    
    fill_indices = find (voxel.volume == 1);
    [xx, yy, zz] = ind2sub(size(voxel.volume), fill_indices);
    xx = voxel.x_lims(1) + voxel_x_size * xx;
    yy = voxel.y_lims(1) + voxel_y_size * yy;
    zz = voxel.z_lims(1) + voxel_z_size * zz;
    
    point_cloud = zeros(size(xx,1),3);
    for ii = 1:size(xx,1)
       point_cloud(ii,:) = [xx(ii), yy(ii), zz(ii)]; 
    end

end