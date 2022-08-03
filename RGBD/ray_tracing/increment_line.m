function volume = increment_line(volume, origin, direction, grid3D)

    % A fast and simple voxel traversal algorithm through a 3D space partition (grid)
    % proposed by J. Amanatides and A. Woo (1987).
    %
    % Input:
    %    origin.
    %    direction.
    %    grid3D: grid dimensions (nx, ny, nz, minBound, maxBound).
    % Original Implementation: https://www.mathworks.com/matlabcentral/fileexchange/26852-a-fast-voxel-traversal-algorithm-for-ray-tracing?focused=5151138&tab=function
    %    Jesús P. Mena-Chalco.
    
    %Modified for space curving
    
    %volume_out = volume;

    [flag, tmin] = rayBoxIntersection(origin, direction, grid3D.minBound, grid3D.maxBound);
    
    if flag == 1 %if flag=0, the line is not intersectin. ignore
        if (tmin<0)
            tmin = 0;
        end

        start   = origin + tmin*direction;
        boxSize = grid3D.maxBound-grid3D.minBound;
        
        x = floor( ((start(1)-grid3D.minBound(1))/boxSize(1))*grid3D.nx )+1;
        y = floor( ((start(2)-grid3D.minBound(2))/boxSize(2))*grid3D.ny )+1;
        z = floor( ((start(3)-grid3D.minBound(3))/boxSize(3))*grid3D.nz )+1;               

        if (x==(grid3D.nx+1));  x=x-1;  end
        if (y==(grid3D.ny+1));  y=y-1;  end            
        if (z==(grid3D.nz+1));  z=z-1;  end
        
        if (direction(1)>=0)
            tVoxelX = (x)/grid3D.nx;
            stepX = 1;
        else
            tVoxelX = (x-1)/grid3D.nx;
            stepX = -1;  
        end
        
        if (direction(2)>=0)
            tVoxelY = (y)/grid3D.ny;
            stepY = 1;
        else
            tVoxelY = (y-1)/grid3D.ny;
            stepY = -1;
        end
        
        if (direction(3)>=0)
            tVoxelZ = (z)/grid3D.nz; 
            stepZ = 1;
        else
            tVoxelZ = (z-1)/grid3D.nz;
            stepZ = -1;  
        end
                
        voxelMaxX  = grid3D.minBound(1) + tVoxelX*boxSize(1);
        voxelMaxY  = grid3D.minBound(2) + tVoxelY*boxSize(2);
        voxelMaxZ  = grid3D.minBound(3) + tVoxelZ*boxSize(3);

        tMaxX      = tmin + (voxelMaxX-start(1))/direction(1);
        tMaxY      = tmin + (voxelMaxY-start(2))/direction(2);
        tMaxZ      = tmin + (voxelMaxZ-start(3))/direction(3);
        
        voxelSizeX = boxSize(1)/grid3D.nx;
        voxelSizeY = boxSize(2)/grid3D.ny;
        voxelSizeZ = boxSize(3)/grid3D.nz;        
        
        tDeltaX    = voxelSizeX/abs(direction(1));
        tDeltaY    = voxelSizeY/abs(direction(2));
        tDeltaZ    = voxelSizeZ/abs(direction(3));
                
        while ( (x<=grid3D.nx)&&(x>=1) && (y<=grid3D.ny)&&(y>=1) && (z<=grid3D.nz)&&(z>=1) )
            %set the voxel to 0
            %volume_out(x,y,z) = 1;
            volume(x, y, z) = volume(x, y, z) + 1;
            
            % ---------------------------------------------------------- %
            % check if voxel [x,y,z] contains any intersection with the ray
            %
            %   if ( intersection )
            %       break;
            %   end;
            % ---------------------------------------------------------- %
            
            if (tMaxX < tMaxY)
                if (tMaxX < tMaxZ)
                    x = x + stepX;
                    tMaxX = tMaxX + tDeltaX;
                else
                    z = z + stepZ;
                    tMaxZ = tMaxZ + tDeltaZ;
                end
            else
                if (tMaxY < tMaxZ)
                    y = y + stepY;
                    tMaxY = tMaxY + tDeltaY;             
                else
                    z = z + stepZ;
                    tMaxZ = tMaxZ + tDeltaZ;
                end
            end
        end       
    end
    
    %volume_out = volume;
end

