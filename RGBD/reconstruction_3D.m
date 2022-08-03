initialize = false;

downsample = false;
downsample_rate = 2';


%% Load shadow segmentation data

load('data/planar_mannequin/3D_plane_scene.mat');
shadowArray = shadows;
clear shadows laser_position;

%% load planar surface mask
load('data/planar_mannequin/mask.mat');
num_frames = size(shadowArray,3);
for ii = 1:num_frames/2
   shadowArray(:,:,ii) = shadowArray(:,:,ii) .* W2_mask; 
end

for ii = 1:num_frames/2
   shadowArray(:,:,ii + num_frames/2) = shadowArray(:,:,ii + num_frames/2) .* W1_mask;
end

%% load point cloud. Ignore invalid pointcloud (x=0, y=0, or z=0)
%see generate_clean_point_cloud for creating pt.mat
load('data/planar_mannequin/pt.mat');

pt_invalid = (point_cloud(:,:,1)==0) + (point_cloud(:,:,2)==0) + (point_cloud(:,:,3)==0) ;
pt_valid = pt_invalid == 0;
for ii = 1:num_frames
   shadowArray(:,:,ii) = shadowArray(:,:,ii) .* pt_valid;
end

%% Illumination point
load('data/planar_mannequin/3D_plane_scene_laser2.mat', 'laser_position')
illumination_index_list = laser_position';
illumination_point_list = zeros(size(illumination_index_list,1),3);
illumination_valid_list = false(size(illumination_index_list,1),1);
for ii = 1:size(illumination_index_list,1)
    illumination_point_list(ii,:) = point_cloud(illumination_index_list(ii,2), illumination_index_list(ii,1),:);
    illumination_valid_list(ii,:) = pt_valid(illumination_index_list(ii,2),illumination_index_list(ii,1))==true;
end

%% downsample if we want it faster 
if downsample
    [w,h,f] = size(shadowArray);
    shadowArray_downsampled = zeros( [floor(w/downsample_rate), floor(h/downsample_rate), f], 'logical');
    for ii = 1:f
        frame = squeeze(shadowArray(:,:,ii));
        shadowArray_downsampled(:,:,ii) = frame(1:downsample_rate:end,1:downsample_rate:end);
    end
    shadowArray = shadowArray_downsampled;
    
    point_cloud_frame_downsampled = point_cloud_frame(1:downsample_rate:end, 1:downsample_rate:end, :); 
    point_cloud_frame = point_cloud_frame_downsampled;
    clear point_cloud_frame_downsampled
    illumination_index_list = round(illumination_index_list./downsample_rate);
end



%% Define Voxels
voxel.x_lims = [-0.3, 0.0];
voxel.y_lims = [-0.2, 0.1];
voxel.z_lims = [1.1, 1.3];
voxel.num_x = 80;
voxel.num_y = 80;
voxel.num_z = 80;
voxel.volume = zeros([voxel.num_x, voxel.num_y, voxel.num_z], 'logical');

%% Initialize volume
disp('initializing the volume')
if initialize
    voxel.volume = zeros([voxel.num_x, voxel.num_y, voxel.num_z], 'logical');
    valid_detection_map = squeeze(sum(shadowArray,3));
    valid_detection_map = valid_detection_map > 0;
    
    %set voxel to 1 if line intersects with a voxel
    for ii = 1:size(illumination_point_list)
        illumination_point = squeeze(illumination_point_list(ii,:));
        detection_point_list = get_detection_points_3D(valid_detection_map, point_cloud_frame);
        voxel = space_carving_frame_initialize(voxel, illumination_point, detection_point_list);
        disp(ii)
    end

end

%% Space Carving
disp('Reconstruction');
curve_frame_list = [1,10,91,100, 101, 110, 191, 197];
se_erode = strel('disk',4);
for ii = curve_frame_list
    if illumination_valid_list(ii) 
        illumination_point = squeeze(illumination_point_list(ii,:));
        shadow_frame = squeeze(shadowArray(:,:,ii));
        shadow_frame = imerode(shadow_frame, se_erode);
        detection_point_list = get_detection_points_3D(shadow_frame, point_cloud);
        voxel = space_carving_frame(voxel, illumination_point, detection_point_list);
        disp(ii)
    end
end

%% Visualize
% 
% [fv,n] = volume2Surface(voxel);
figure;
scatter_plot_3D(voxel);

% voxel.volume = bwmorph3(voxel.volume,'clean');
% scatter_plot_3D(voxel);
% 
% se = strel('cube',8);
voxel.volume = imerode(voxel.volume,se);
% voxel.volume = imdilate(voxel.volume,se);
% figure;
% scatter_plot_3D(voxel);
%save
% save('mannequin8by8_test.mat')
