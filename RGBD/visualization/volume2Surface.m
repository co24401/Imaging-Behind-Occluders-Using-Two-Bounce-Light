function [fv,n] = volume2Surface(voxel)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

V = voxel.volume;
num_x = voxel.num_x;
num_y = voxel.num_y;
num_z = voxel.num_z;
x_lims = voxel.x_lims;
y_lims = voxel.y_lims;
z_lims = voxel.z_lims;

fv = isosurface(V, 0.5);  % Check that 0.5 is the correct value.
figure; 
p = patch(fv);
n = isonormals(V, p);

fv.vertices(:, 1) = ((x_lims(2)-x_lims(1))/(num_x-1))*(fv.vertices(:, 1) - 1) + x_lims(1);
fv.vertices(:, 2) = ((y_lims(2)-y_lims(1))/(num_y-1))*(fv.vertices(:, 2) - 1) + y_lims(1);
fv.vertices(:, 3) = ((z_lims(2)-z_lims(1))/(num_z-1))*(fv.vertices(:, 3) - 1) + z_lims(1);

p.Vertices = fv.vertices;

smoothNorms;
p.VertexNormals = n_smooth;  % Smooth norms a la Kwong to make surface appear less jagged when illuminated

set(p,'FaceColor','b','EdgeColor','none');
light
camlight left
lighting phong

axis equal
view(-145, -1)

end

