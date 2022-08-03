dir = 'data/wall_paper4/';
addpath('ray_tracing');
addpath('visualization');

downsample_factor = 2;

frames_filename = 'mannequin.mat';
mask_filename = 'mask.mat';
point_cloud_filename = 'pt.mat';
illumination_filename = 'laser_exposure1.mat';

shadow_threshold = 0.25;
carving_threshold = 0.4;
W2_num_frames = 25;

%% load frames
load([dir,frames_filename], 'rgb_list');
num_frames = length(rgb_list);
[h,w,c] = size(rgb_list{1});

h = round(h/downsample_factor);
w = round(w/downsample_factor);


frames = zeros(h, w, num_frames);

for ii = 1:num_frames
        frames(:,:,ii) = rgb_list{ii}(1:downsample_factor:end, 1:downsample_factor:end, 2);
end
clear rgb_list

disp('Frames loaded.')

%% Denoise frames

% sigma_r = 300;
% sigma_s = 2;
% 
% for ii = 1:size(frames, 3)
%     frames(:, :, ii) = imbilatfilt(frames(:, :, ii), sigma_r, sigma_s);
% end
% 
% disp('Frames denoised.')

%% load point_cloud
load([dir,point_cloud_filename], 'point_cloud');

pt_invalid = (point_cloud(:,:,1)==0) + (point_cloud(:,:,2)==0) + (point_cloud(:,:,3)==0) ;
pt_valid = pt_invalid == 0;

point_cloud_downsampled = point_cloud(1:downsample_factor:end, 1:downsample_factor:end, :);
pt_valid_downsampled = pt_valid(1:downsample_factor:end, 1:downsample_factor:end);

disp('Point cloud loaded.')

%% Load illumination points
load([dir, illumination_filename], 'rgb_list');
illumination_index_list = extract_illumination_index(rgb_list);
illumination_point_list = zeros(num_frames,3);
illumination_valid_list = false(num_frames,1);
illumination_intensity_list = zeros(num_frames,1);
for ii = 1:size(illumination_index_list,1)
    illumination_point_list(ii,:) = point_cloud(illumination_index_list(ii,2), illumination_index_list(ii,1),:);
    illumination_valid_list(ii,:) = pt_valid(illumination_index_list(ii,2),illumination_index_list(ii,1))==true;
    illumination_intensity_list(ii) = max(rgb_list{ii}(:));
end
clear rgb_list;

%% Load mask
load([dir, mask_filename], 'W1_mask', 'W2_mask');
W1_mask = W1_mask(1:downsample_factor:end, 1:downsample_factor:end);
W2_mask = W2_mask(1:downsample_factor:end, 1:downsample_factor:end);

%% Distance correction from the illumination_source

W2_normal = [-1,0,0];
for ii = 1:W2_num_frames
    illumination_point = illumination_point_list(ii,:);
    distance_map = zeros(size(point_cloud_downsampled));
    theta_map = zeros(size(point_cloud_downsampled,1), size(point_cloud_downsampled,2));
    for j = 1:size(distance_map,1)
        for k = 1:size(distance_map,2)
            if W2_mask(j,k) == 1
                distance = reshape(point_cloud_downsampled(j,k,:),[3,1]) - reshape(illumination_point,[3,1]);
                distance_map(j,k,:) = distance;
                theta_map(j,k) = abs(dot(W2_normal, normalize(distance,'norm'))); 
            end
        end
    end
    distance_map(distance_map==0) = nan;
    theta_map(theta_map==0) = nan;
    distance_square_map = sum(distance_map.^2,3);
    frames(:,:,ii) = (frames(:,:,ii) .* distance_square_map ./ theta_map)./illumination_intensity_list(ii);

end

W1_normal = [1,0,0];
for ii =  W2_num_frames+1:num_frames
    illumination_point = illumination_point_list(ii,:);
    distance_map = zeros(size(point_cloud_downsampled));
    theta_map = zeros(size(point_cloud_downsampled,1), size(point_cloud_downsampled,2));
    for j = 1:size(distance_map,1)
        for k = 1:size(distance_map,2)
            if W1_mask(j,k) == 1
                distance = reshape(point_cloud_downsampled(j,k,:),[3,1]) - reshape(illumination_point,[3,1]);
                distance_map(j,k,:) = distance;
                theta_map(j,k) = abs(dot(W1_normal, normalize(distance,'norm'))); 
            end
        end
    end
    distance_map(distance_map==0) = nan;
    theta_map(theta_map==0) = nan;
    distance_square_map = sum(distance_map.^2,3);
    frames(:,:,ii) = (frames(:,:,ii) .* distance_square_map ./ theta_map)./illumination_intensity_list(ii);
end

%% per-pixel shaodw segmentation

shadow_frames = false(size(frames));
carving_frames = false(size(frames));
for ii = 1:size(frames,1)
    for j = 1:size(frames,2)
        if W1_mask(ii,j) + W2_mask(ii,j) == 1
            pixel_frames = frames(ii,j,:);
            pixel_frames = pixel_frames./max(pixel_frames);
            shadow_frames(ii,j,:) = pixel_frames < shadow_threshold;
            carving_frames(ii,j,:) = pixel_frames > carving_threshold;
        end
    end
end
%carving_frames = (shadow_frames == 0);
%shadow_frames: 1 if shadow, 0 if not shadow
disp('Shadow Segmentation finished.')

%% apply mask to remove regions that are not valid

for ii = 1:W2_num_frames
   shadow_frames(:,:,ii) = shadow_frames(:,:,ii) .* W2_mask .* pt_valid_downsampled; 
   carving_frames(:,:,ii) = carving_frames(:,:,ii) .* W2_mask .* pt_valid_downsampled; 
end
for ii = W2_num_frames+1:num_frames
   shadow_frames(:,:,ii) = shadow_frames(:,:,ii) .* W1_mask .* pt_valid_downsampled; 
   carving_frames(:,:,ii) = carving_frames(:,:,ii) .* W1_mask .* pt_valid_downsampled; 
end

%% Robust Carving
disp('Generate carving frames.')

% Define voxel grid.
outside_voxel.x_lims = [-0.3, 0.3];
outside_voxel.y_lims = [-0.3, 0.3];
outside_voxel.z_lims = [0.8, 1.2];
outside_voxel.num_x = 100;
outside_voxel.num_y = 100;
outside_voxel.num_z = 100;
outside_voxel.volume = zeros([outside_voxel.num_x, outside_voxel.num_y, outside_voxel.num_z]);

inside_voxel = outside_voxel;

% Reconstruction via carving
disp('Commence robust carving.');
for ii = 1:num_frames
    if illumination_valid_list(ii) 
        if sum(sum(shadow_frames(:,:,ii))) > 0
            illumination_point = squeeze(illumination_point_list(ii,:));
            carving_frame = squeeze(carving_frames(:,:,ii));
            lit_point_list = get_detection_points_3D(carving_frame, point_cloud_downsampled);
            shadow_frame = squeeze(shadow_frames(:,:,ii));
            shadow_point_list = get_shadow_points_3D(shadow_frame, point_cloud_downsampled);
            [outside_voxel, inside_voxel] = robust_carving_frame(outside_voxel, inside_voxel, illumination_point, lit_point_list, shadow_point_list);
            disp(ii)
        end
    end
end

reconstruction_voxel = inside_voxel;

eta = .3; % Probability occupied voxel is traced to illuminated region (miss probability)
xi = .5; % Probability that an empty voxel is traced to shadow (probability false alarm)
p0 = 0.9; % Prior probability that any voxel is empty
p1 = 0.1; % Prior probability that any voxel is occupied
T = 0.95; % Probabilitzy threshold to decide that voxel is occupied

mm = 0:num_frames; 
nn = 0:num_frames;
[M, N] = meshgrid(mm, nn);
PO = p1*(eta.^M).*((1-eta).^N)./(p0*((1-xi).^M).*(xi.^N) + p1*(eta.^M).*((1-eta).^N));
[reconstruction_voxel.volume, probability_volume] = visualize_probablistic(inside_voxel, outside_voxel ,PO, T);

%save([dir, 'result_supp.mat'], 'inside_voxel', 'outside_voxel', 'reconstruction_voxel', 'point_cloud', 'frames', 'shadow_frames', 'carving_frames', 'eta', 'xi', 'p0', 'p1', 'T', 'shadow_threshold', 'carving_threshold', 'downsample_factor')
