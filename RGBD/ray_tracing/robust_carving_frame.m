function [outside_voxel, inside_voxel] = robust_carving_frame(outside_voxel, inside_voxel, illumination_point, lit_point_list, shadow_point_list)
%input
% voxel: voxel specification
% illumination_point: position of illumination point in world
% coordinate
% detection_point_list: position of detection point in world coordinate

grid3D.nx = outside_voxel.num_x;
grid3D.ny = outside_voxel.num_y;
grid3D.nz = outside_voxel.num_z;
grid3D.minBound = [outside_voxel.x_lims(1), outside_voxel.y_lims(1), outside_voxel.z_lims(1)]';
grid3D.maxBound = [outside_voxel.x_lims(2), outside_voxel.y_lims(2), outside_voxel.z_lims(2)]';

%outside_voxel = outside_voxel;
%volume = voxel.volume;
lit_volume = zeros([outside_voxel.num_x, outside_voxel.num_y, outside_voxel.num_z]);
shadow_volume = lit_volume;

line_origin = illumination_point';

for ii = 1:size(lit_point_list,1) %for each line
    line_destination = lit_point_list(ii,:)';
    line_direction = line_destination - line_origin;
    line_direction = line_direction/norm(line_direction);
    %+1 to voxels intersecting the line
    lit_volume = increment_line(lit_volume, line_origin, line_direction, grid3D);
end

for ii = 1:size(shadow_point_list,1) %for each line
    line_destination = shadow_point_list(ii,:)';
    line_direction = line_destination - line_origin;
    line_direction = line_direction/norm(line_direction);
    %+1 to voxels intersecting the line
    shadow_volume = increment_line(shadow_volume, line_origin, line_direction, grid3D);
end
% Again this is going to reset voxels to one if they were anything but
% one.
%volume(volume>1) = 1;

%(lit_volume ~= 0) & (shadow_volume ~= 0)
outside_voxel.volume = outside_voxel.volume + ((lit_volume > 0) & (shadow_volume == 0));
inside_voxel.volume = inside_voxel.volume + ((shadow_volume > 0) & (lit_volume == 0));
%inside_voxel.volume = inside_voxel.volume + (shadow_volume > 0);
% % Uncomment for ray-by-ray instead of frame-by-frame
%outside_voxel.volume = outside_voxel.volume + lit_volume;

end

