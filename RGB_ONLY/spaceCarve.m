%% Define 3D grid
x_lims = [10 65];
y_lims = [10 55];
z_lims = [10 65];

num_x = round((x_lims(2)-x_lims(1))*4);
num_y = round((y_lims(2)-y_lims(1))*4);
num_z = round((z_lims(2)-z_lims(1))*4);

xx = linspace(x_lims(1), x_lims(2), num_x);
yy = linspace(y_lims(1), y_lims(2), num_y);
zz = linspace(z_lims(1), z_lims(2), num_z);

storepoints = [kron(ones(1, num_y*num_z, 'uint8'), uint8(1:num_x)); ...
    kron( ones(1, num_z, 'uint8'), kron(uint8(1:num_y), ones(1, num_x, 'uint8')) ); ...
    kron(uint8(1:num_z), ones(1, num_x*num_y, 'uint8'))];
checkedPoints = false(1, size(storepoints, 2));

disp('Defined 3D grid.')

%% Space carving
% Right wall shadows

disp('Processing shadows cast onto right wall.')
for ii = 1:num_frames1
       
    los2plane = @(x, y, z) [lw1(1, ii) + (x-lw1(1, ii)).*(z2w-lw1(3, ii))./(z-lw1(3, ii)); ...
        lw1(2, ii) + (y-lw1(2, ii)).*(z2w-lw1(3, ii))./(z-lw1(3, ii)) ];
    
    shadow = int8(shadowArray(:, :, ii));
    [startrow, startcol] = find(diff(shadow, 1, 2)==1);
    [stoprow, stopcol] = find(diff(shadow, 1, 2)==-1);
    
    insidepoints = true(1, size(storepoints, 2));
    cw = los2plane(xx(storepoints(1, :)), ...
        yy(storepoints(2, :)), ...
        zz(storepoints(3, :)) );  % Projection of point to obs. plane
    cp = homog([cw; ones(1, size(cw, 2))], H2wp); % Convert to pixel coordinates

    obsPts = observable2(cp); % Which intersection points actually lie on observed plane
    checkedPoints(obsPts) = true;
    
    for jj = 1:size(cp, 2)
        if obsPts(jj)
            outchk = any(cp(1, jj) > startcol(startrow == round(cp(2, jj))) & ...
                cp(1, jj) <= stopcol(stoprow == round(cp(2, jj))));
            if outchk
                insidepoints(jj) = false;
            end
        end
    end
    
    storepoints(:, ~insidepoints) = [];
    checkedPoints(:, ~insidepoints) = [];
    
    if mod(ii, 10) == 0
        disp(['Frame ' num2str(ii) ' processed.'])
    end
    
end

% Left wall shadows

disp('Processing shadows cast onto left wall.')
for ii = 1:num_frames2

   los2plane = @(x, y, z) [lw2(1, ii) + (x-lw2(1, ii)).*(z1w-lw2(3, ii))./(z-lw2(3, ii)); ...
        lw2(2, ii) + (y-lw2(2, ii)).*(z1w-lw2(3, ii))./(z-lw2(3, ii)) ];
        
    shadow = int8(shadowArray(:, :, ii+num_frames1));
    [startrow, startcol] = find(diff(shadow, 1, 2)==1);
    [stoprow, stopcol] = find(diff(shadow, 1, 2)==-1);
    
    %% Carve away grid points
    insidepoints = true(1, size(storepoints, 2));
    cw = los2plane(xx(storepoints(1, :)), ...
        yy(storepoints(2, :)), ...
        zz(storepoints(3, :)) );  % Projection of point to obs. plane
    cp = homog([cw; ones(1, size(cw, 2))], H1wp); % Convert to pixel coordinates

    obsPts = observable1(cp); % Which intersection points actually lie on observed plane
    checkedPoints(obsPts) = true;
    
    for jj = 1:size(cp, 2)
        if obsPts(jj)
            outchk = any(cp(1, jj) > startcol(startrow == round(cp(2, jj))) & ...
                cp(1, jj) <= stopcol(stoprow == round(cp(2, jj))));
            if outchk
                insidepoints(jj) = false;
            end
        end
    end
    
    storepoints(:, ~insidepoints) = [];
    checkedPoints(:, ~insidepoints) = [];
    
    if mod(ii, 10) == 0
        disp(['Frame ' num2str(ii+num_frames1) ' processed.'])
    end
    
end