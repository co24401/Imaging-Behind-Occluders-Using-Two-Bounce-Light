% reconstruction_robust_carving.m
% Connor Henley, Tomohiro Maeda
% 3/5/2020
%
% Main driver script that implements imaging behind occluders algorithm.
% This script assumes that the user has access to background frames that
% can be used for shadow segmentation.  This script was used to generate
% results in Sections 5 and 6 of the main paper and is described in
% additional detail in Supplementary Section 2.2.
%
% Primary outputs are reconstruction_voxel, which contains a binary 3D 
% occupancy grid of the hidden space, and probability_volume, which
% contains the probability of occupancy for each voxel in the grid.

dir = 'data/shapes5/';
addpath('ray_tracing');
addpath('visualization');

downsample_factor = 4; % Downsample pixels of input frames by this factor.  
% Reduces run-time and over-carving (in some circumstances).

frames_filename = 'mannequin.mat'; % File with RGB frames
background_filename = 'background.mat'; % File with background frames, collected with hidden object removed
mask_filename = 'mask.mat'; % Masks define region of interest to search for shadows in images (created with MATLAB function "roipoly")
point_cloud_filename = 'pt.mat'; % Point cloud from camera depth channel
illumination_filename = 'laser.mat'; % RGB frames from short exposures

shadow_ratio_threshold = 0.9; % Shadow segmentation parameter
valid_background_threshold = 50; % Determines which background pixels are valid for use in shadow segmentation
W2_num_frames = 15; % Num frames that use W2 ROI (the rest will use W1 ROI)
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

%% load background frames
load([dir, background_filename], 'rgb_list');
background_frames = zeros(h, w, num_frames);
for ii = 1:num_frames
    background_frames(:,:,ii) = rgb_list{ii}(1:downsample_factor:end, 1:downsample_factor:end, 2); 
end
clear rgb_list

disp('Background frames loaded.')

%% shadow segmentation using background frames

% Bilateral filter parameters
sigma_r = 300;
sigma_s = 2;

for ii = 1:size(frames, 3)
    frames(:, :, ii) = imbilatfilt(frames(:, :, ii), sigma_r, sigma_s);
    background_frames(:, :, ii) = imbilatfilt(background_frames(:, :, ii), sigma_r, sigma_s);
end

shadow_frames = frames ./ background_frames;
shadow_frames = (shadow_frames < shadow_ratio_threshold) .* (background_frames > valid_background_threshold);
carving_frames = (shadow_frames == 0) .* (background_frames > valid_background_threshold);
% shadow_frames: 1 if shadow, 0 if not shadow

disp('Shadow Segmentation finished.')

%% load point_cloud
load([dir,point_cloud_filename], 'point_cloud');

% Determine valid points
pt_invalid = (point_cloud(:,:,1)==0) + (point_cloud(:,:,2)==0) + (point_cloud(:,:,3)==0) ;
pt_valid = pt_invalid == 0;

point_cloud_downsampled = point_cloud(1:downsample_factor:end, 1:downsample_factor:end, :);
pt_valid_downsampled = pt_valid(1:downsample_factor:end, 1:downsample_factor:end);

disp('Point cloud loaded.')

%% apply mask to remove regions that are not valid

load([dir, mask_filename], 'W1_mask', 'W2_mask');
W1_mask = W1_mask(1:downsample_factor:end, 1:downsample_factor:end);
W2_mask = W2_mask(1:downsample_factor:end, 1:downsample_factor:end);
for ii = 1:W2_num_frames
   shadow_frames(:,:,ii) = shadow_frames(:,:,ii) .* W2_mask .* pt_valid_downsampled; 
   carving_frames(:,:,ii) = carving_frames(:,:,ii) .* W2_mask .* pt_valid_downsampled; 
end
for ii = W2_num_frames+1:num_frames
   shadow_frames(:,:,ii) = shadow_frames(:,:,ii) .* W1_mask .* pt_valid_downsampled; 
   carving_frames(:,:,ii) = carving_frames(:,:,ii) .* W1_mask .* pt_valid_downsampled; 

end

%% Load illumination points (laser spot positions)

load([dir, illumination_filename], 'rgb_list');
illumination_index_list = extract_illumination_index(rgb_list);
clear rgb_list;
illumination_point_list = zeros(num_frames,3);
illumination_valid_list = false(num_frames,1);
for ii = 1:size(illumination_index_list,1)
    illumination_point_list(ii,:) = point_cloud(illumination_index_list(ii,2), illumination_index_list(ii,1),:);
    illumination_valid_list(ii,:) = pt_valid(illumination_index_list(ii,2),illumination_index_list(ii,1))==true;
end

%% Robust Carving

disp('Generate carving frames.')

% Define voxel grid
outside_voxel.x_lims = [-0.3, 0.3];
outside_voxel.y_lims = [-0.3, 0.3];
outside_voxel.z_lims = [0.8, 1.4];
outside_voxel.num_x = 100;
outside_voxel.num_y = 100;
outside_voxel.num_z = 100;
outside_voxel.volume = zeros([outside_voxel.num_x, outside_voxel.num_y, outside_voxel.num_z]);

inside_voxel = outside_voxel;

% Reconstruction via carving
disp('Commence robust carving.');
for ii = 1:num_frames
    if illumination_valid_list(ii) 
        illumination_point = squeeze(illumination_point_list(ii,:));
        carving_frame = squeeze(carving_frames(:,:,ii));
        lit_point_list = get_detection_points_3D(carving_frame, point_cloud_downsampled);
        shadow_frame = squeeze(shadow_frames(:,:,ii));
        shadow_point_list = get_shadow_points_3D(shadow_frame, point_cloud_downsampled);
        [outside_voxel, inside_voxel] = robust_carving_frame(outside_voxel, inside_voxel, illumination_point, lit_point_list, shadow_point_list);
        disp(ii)
    end
end

reconstruction_voxel = inside_voxel;
reconstruction_voxel.volume = scatter_plot_threshold_ratio(inside_voxel, outside_voxel, 2, 10, 1);

eta = .1; % Probability occupied voxel is traced to illuminated region (miss probability)
xi = .5; % Probability that an empty voxel is traced to shadow (probability false alarm)
p0 = 0.9; % Prior probability that any voxel is empty
p1 = 0.1; % Prior probability that any voxel is occupied
T = 0.9; % Probability threshold to decide that voxel is occupied

% Generate lookup table "PO" that stores voxel occupancy probability as a 
% function of the number of inside and outside classifications
mm = 0:num_frames; 
nn = 0:num_frames;
[M, N] = meshgrid(mm, nn);
PO = p1*(eta.^M).*((1-eta).^N)./(p0*((1-xi).^M).*(xi.^N) + p1*(eta.^M).*((1-eta).^N));

[reconstruction_voxel.volume, probability_volume] = visualize_probablistic(inside_voxel, outside_voxel, PO, T);

%save([dir, 'result_bl_nomix_gameday.mat'], 'inside_voxel', 'outside_voxel', 'reconstruction_voxel', 'point_cloud')
