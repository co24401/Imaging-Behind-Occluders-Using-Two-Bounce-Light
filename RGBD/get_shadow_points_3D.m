function detection_point_list = get_shadow_points_3D(shadow_frame, point_cloud_frame)
    
    shadow_indices = find( shadow_frame == 1);
    detection_point = zeros(size(shadow_indices,1),3);
    [shadow_indices_row, shadow_indices_col] = ind2sub(size(shadow_frame), shadow_indices);
    for ii = 1:size(shadow_indices)
       detection_point_list(ii,:) = squeeze(point_cloud_frame(shadow_indices_row(ii), shadow_indices_col(ii), :)); 
    end
end
