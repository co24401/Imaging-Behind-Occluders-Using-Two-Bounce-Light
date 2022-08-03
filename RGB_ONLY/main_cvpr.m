folder = 'C:\Users\Tomohiro Maeda\Documents\MATLAB\TwoBounce\CVPR2020\Spring\';
filename = 'output';

%% Load observation plane homography data

wallsFile = 'C:\Users\Tomohiro Maeda\Documents\MATLAB\TwoBounce\CVPR2020\walls.mat';
load(wallsFile)

homog = @(x, H) H*x./(H(3, :)*x);

%% Load shadow images and laser spots

acqFile = [folder 'frames.mat'];
load(acqFile, 'lw1', 'lw2', 'cFrameArray')

shadowSegTimer = tic;

num_frames1 = size(lw1, 2);
num_frames2 = size(lw2, 2);

shadowArray = zeros(size(cFrameArray), 'logical');
medFilterSize = [7 7];
se = ones(3, 5);
mask1 = imdilate(mask1, se);
mask2 = imdilate(mask2, se);

for ii = 1:num_frames1
    %    shadowArray(:, :, ii) = medfilt2(double(cFrameArray(:, :, ii)).*mask2, medFilterSize)>shadowThreshold;
    [~,shadow]=kmeans(medfilt2(double(cFrameArray(:, :, ii)).*mask2, medFilterSize),2);
    shadow = (shadow == 2);
    shadow = imerode(shadow, se);
    shadowArray(:, :, ii) = shadow;
    if mod(ii, 10) == 0
        disp(ii)
    end
end

for ii = 1:num_frames2
    %    shadowArray(:, :, ii+num_frames1) = medfilt2(double(cFrameArray(:, :, ii+num_frames1)).*mask1, medFilterSize)>shadowThreshold;
    [~,shadow]=kmeans(medfilt2(double(cFrameArray(:, :, ii+num_frames1)).*mask1, medFilterSize),2);
    shadow = (shadow==2);
    shadow = imerode(shadow, se);
    shadowArray(:, :, ii+num_frames1) = shadow;
    if mod(ii, 10) == 0
        disp(ii+num_frames1)
    end
end

shadowSegTime = toc(shadowSegTimer);

%% Space carve (Arguments inside)
spaceCarveTimer = tic;

spaceCarve;

spaceCarveTime = toc(spaceCarveTimer);
%%
figure;   scatter3(xx(storepoints(1, :)), ...
    yy(storepoints(2, :)), ...
    zz(storepoints(3, :)), ...
    100, yy(storepoints(2, :)), '.')

xlim(x_lims)
ylim(y_lims)
zlim(z_lims)
axis equal
view(30, -55)
set(gca, 'zdir', 'reverse')

xlabel('X (cm)')
ylabel('Y (cm)')
zlabel('Z (cm)')

saveas(gcf, [folder filename '_scatter3.fig'])

%% Convert occupancy grid to surface and plot
mCubesTimer = tic;
[fv,n] = points2Surface(storepoints, num_x, num_y, num_z, x_lims, y_lims, z_lims);
mCubesTime = toc(mCubesTimer);
%% Save to workspace

saveas(gcf, [folder filename '.fig'])
save([folder filename '.mat'], 'shadowArray', 'lw1', 'lw2', 'medFilterSize', 'storepoints', 'checkedPoints', 'x_lims', 'y_lims', 'z_lims', 'num_x', 'num_y', 'num_z', 'fv', 'n', 'shadowSegTime', 'spaceCarveTime', 'mCubesTime');