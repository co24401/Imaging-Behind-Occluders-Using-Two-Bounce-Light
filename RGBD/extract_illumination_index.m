function illumination_index = extract_illumination_index(image_stack)
    num_frames = length(image_stack);
    illumination_index = zeros(num_frames, 2);
    for ii = 1:num_frames
        curr_img = image_stack{ii}(:,:,2);
        max_of_img = max(curr_img(:));
        labeledImage = bwlabel(squeeze(255*(image_stack{ii}(:,:,2)/max_of_img)) > 200); %
        measurements = regionprops(labeledImage, squeeze(image_stack{ii}(:,:,2)), 'WeightedCentroid');
        center_1 = measurements.WeightedCentroid;
        x = round(center_1(1));
        y = round(center_1(2));
        illumination_index(ii,:) = [x,y];
    end
end

