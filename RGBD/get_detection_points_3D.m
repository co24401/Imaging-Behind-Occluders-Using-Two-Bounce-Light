function detection_point_list = get_detection_points_3D(shadow_frame, point_cloud_frame)
    
    no_shadow_indices = find( shadow_frame == 1);
    detection_point = zeros(size(no_shadow_indices,1),3);
    [no_shadow_indices_row, no_shadow_indices_col] = ind2sub(size(shadow_frame), no_shadow_indices);
    for ii = 1:size(no_shadow_indices)
       detection_point_list(ii,:) = squeeze(point_cloud_frame(no_shadow_indices_row(ii), no_shadow_indices_col(ii), :)); 
    end
end
