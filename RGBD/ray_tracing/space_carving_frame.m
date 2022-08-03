function voxel_out = space_carving_frame(voxel, illumination_point, detection_point_list)
    %input
    % voxel: voxel specification
    % illumination_point: position of illumination point in world
    % coordinate
    % detection_point_list: position of detection point in world coordinate
    
    grid3D.nx = voxel.num_x;
    grid3D.ny = voxel.num_y;
    grid3D.nz = voxel.num_z;
    grid3D.minBound = [voxel.x_lims(1), voxel.y_lims(1), voxel.z_lims(1)]';
    grid3D.maxBound = [voxel.x_lims(2), voxel.y_lims(2), voxel.z_lims(2)]';
    
    voxel_out = voxel;
    volume = voxel.volume;
    
    line_origin = illumination_point';
    
    for ii = 1:size(detection_point_list,1) %for each line
        line_destination = detection_point_list(ii,:)';
        line_direction = line_destination - line_origin;
        line_direction = line_direction/norm(line_direction);
        %remove voxels intersecting the line
        volume = volume + line_carving(volume, line_origin, line_direction, grid3D); 
    end
    volume(volume>1) = 1;
    voxel_out.volume = voxel_out.volume + volume;
    
end

