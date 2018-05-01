8% clear the screen
clear all;
close all;
clc;

% Define the 3-D coordinates of the marker features. 
%In this simulation, to simply the process, the calibration boards mounted on both sides of the rig are the same, 
%which means the coordinates represented in each marker's frame are the same. 
%In this measurement generation process, we use 4 feature points(4 corners).

BackSideMarker4CornerPoints_3D = [ 0        0        0;
                                   0        0.225    0;
                                   0.175    0        0;
                                   0.175    0.225    0];
                               
%%% Used for P3P as inputs. Each column is an point.%%%                               
BackSideMarker3CornerPoints_3D = [  0     0       0.175;
                                    0     0.225   0;
                                    0     0       0]; 
                                
%%% Used for 3D-2D projection process.                               
BackSideMarker4CornerPoints_homog = [ 0       0        0.175     0.175;
                                      0       0.225    0         0.225;
                                      0       0        0         0;
                                      1       1        1         1];
                                  
%%% Used for matlab function "estimateWorldCameraPose".
FrontSideMarker4CornerPoints_3D = [ 0       0      0;
                                    0       0.4    0;
                                    0.28    0      0;
                                    0.28    0.4    0];
                               
%%% Used for P3P as inputs. Each column is an point.%%%                               
FrontSideMarker3CornerPoints_3D = [ 0     0      0.28;
                                    0     0.4    0;
                                    0     0      0]; 
                                
%%% Used for 3D-2D projection process.                               
FrontSideMarker4CornerPoints_homog = [ 0      0      0.28    0.28;
                                       0      0.4    0       0.4;
                                       0      0      0       0;
                                       1      1      1       1];
                                  
                                  
focal_length = 500;
principal_point_x = 320;
principal_point_y = 240;

focal_length_vec = [focal_length  focal_length];                                
principalPoint_vec = [principal_point_x  principal_point_y];
imageSize_vec = [principal_point_x*2  principal_point_y*2];

% In this simulation we assume that the two cameras used in the experiment have the following intrinsic parameters.
K_leftcam = cameraIntrinsics( focal_length_vec, principalPoint_vec, imageSize_vec );
K_rightcam = cameraIntrinsics( focal_length_vec, principalPoint_vec, imageSize_vec );
K_backcam = cameraIntrinsics( focal_length_vec, principalPoint_vec, imageSize_vec );

K_leftcam_intrinsic = [ focal_length       0                 principal_point_x;
                        0                  focal_length      principal_point_y;
                        0                  0                 1];

% Corresponding projection matrix:            
P_leftcam = [ focal_length       0                 principal_point_x     0;
              0                  focal_length      principal_point_y     0;
              0                  0                 1                     0];

K_rightcam_intrinsic = [ focal_length      0                 principal_point_x;
                         0                 focal_length      principal_point_y;
                         0                 0                 1];
                       
% Corresponding projection matrix:
P_rightcam = [ focal_length      0                principal_point_x     0;
               0                 focal_length     principal_point_y     0;
               0                 0                1                     0];
          
K_backcam_intrinsic = [ focal_length      0                 principal_point_x;
                        0                 focal_length      principal_point_y;
                        0                 0                 1];
                       
% Corresponding projection matrix:
P_backcam = [ focal_length      0                principal_point_x     0;
              0                 focal_length     principal_point_y     0;
              0                 0                1                     0];          

LeftCamParams = cameraParameters('IntrinsicMatrix', K_leftcam.IntrinsicMatrix, 'WorldUnits', 'm', 'ImageSize', [640,480]);
RightCamParams = cameraParameters('IntrinsicMatrix', K_rightcam.IntrinsicMatrix, 'WorldUnits', 'm', 'ImageSize', [640,480]);
BackCamParams = cameraParameters('IntrinsicMatrix', K_backcam.IntrinsicMatrix, 'WorldUnits', 'm', 'ImageSize', [640,480]);

% Since the car is not moving, choose the frame of the leftside camera mounted on the car as the world reference frame.
% W represents the world frame, leftcam means the left camera frame, in this experiment, they are aligned.
T_W_leftcam = [1   0   0   0;
               0   1   0   0;
               0   0   1   0;
               0   0   0   1]; 

% The right camera board mounted on the right side of the car also has its own coordinate frame.
% In this case, they are stereo camera, with baseline of 1 meter.
T_leftcam_rightcam_true = [ 1       0       0      0.2;
                            0       1       0      0;
                            0       0       1      0;
                            0       0       0      1];
% In this setting, the back camera is mounted on the leftside of the car, but looking towards backside.
% With the translation in Z and Y direction of -0.3 meter.
T_leftcam_backcam1 = [ -1      0      0      0.1;
                        0      1      0     -0.3;
                        0      0     -1     -0.3;
                        0      0      0      1];
% Then rotate around Y asix about pi/4;
T_backcam1_backcam2 = [  cos(pi/4)      0      sin(pi/4)   0;
                         0              1      0           0;
                        -sin(pi/4)      0      cos(pi/4)   0;
                           0            0      0           1];
% Then rotate around X axis about -pi/6;
T_backcam2_backcam3 = [  1      0               0              0
                         0      cos(-pi/6)     -sin(-pi/6)     0;
                         0      sin(-pi/6)      cos(-pi/6)     0;
                         0      0               0              1];
T_leftcam_backcam_true = T_leftcam_backcam1 * T_backcam1_backcam2 * T_backcam2_backcam3;
                             
% W represents the world frame, rightcam means the right camera frame, T_W_rightcam is the ground truth of unknown Y.
% AX = YB.
T_W_rightcam = T_leftcam_rightcam_true;     
T_rightcam_leftcam_true = inv( T_leftcam_rightcam_true );
T_backcam_leftcam_true = inv( T_leftcam_backcam_true );
T_backcam_rightcam_true = T_backcam_leftcam_true * T_leftcam_rightcam_true;
T_rightcam_backcam_true = inv( T_backcam_rightcam_true );

frontmarker_projection_points_2d = [ 160  112; 480 112; 160 336; 480 336];
backmarker_projection_points_2d = [ 140  100; 500 100; 140 380; 500 380];

[ R_world_rightcam, t_world_rightcam ] = estimateWorldCameraPose( frontmarker_projection_points_2d, FrontSideMarker4CornerPoints_3D, RightCamParams);
T_frontmarker_rightcam_projected = [R_world_rightcam, transpose(t_world_rightcam);
                                        0        0        0       1];

[ R_world_backcam, t_world_backcam ] = estimateWorldCameraPose( backmarker_projection_points_2d, BackSideMarker4CornerPoints_3D, BackCamParams);
T_backmarker_backcam_projected = [R_world_backcam, transpose(t_world_backcam);
                                          0        0       0       1];

% This is the ground truth of X in the equation AX = YB.               
T_frontmarker_backmarker_true = T_frontmarker_rightcam_projected * T_rightcam_backcam_true * inv(T_backmarker_backcam_projected);                              
T_backmarker_frontmarker_true = inv( T_frontmarker_backmarker_true );

x_axis_minimum_distance = 150;
y_axis_minimum_distance = 120;

axis_increasement = 10;
larger_axis_increasement = 15;
projection_area_thred = 0.12;

minimum_translation_thred = 0.08;
minimum_rotation_thred = 0.04;

%% Set the threshold after reprojection using the result from p3p to be 2 pixel(Normally it would be much smaller, but this threshold is enough).
p3p_dif_threshold = 2; 

full_area = imageSize_vec(1,1)*imageSize_vec(1,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% First Point: ( 30, 20 )---( 400, 300 )%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Second Point: ( point1_x + 100, 20 )---( 620, 300 )%%%%%%%%%%%%
%%%%%%%%%%% Third Point: ( 30, point1_y+80 )---( 400,460 ) %%%%%%%%%%%%%%%%
%%%%%%%%%%% Forth Point: ( point3_x+100, point2_y+80 )---( 620 ,460 )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

point1_x_start = 30;      point1_x_end = 400;
point1_y_start = 20;      point1_y_end = 300;
point2_x_start = 0 + 0;   point2_x_end = 620;
point2_y_start = 20;      point2_y_end = 300;
point3_x_start = 30;      point3_x_end = 400;
point3_y_start = 0 + 0;   point3_y_end = 460;

to_minus = mod( point1_x_start, larger_axis_increasement );
point1_x_start = point1_x_start - to_minus;
to_minus = mod( point1_x_end, larger_axis_increasement );
point1_x_end = point1_x_end - to_minus;

to_minus = mod( point1_y_start, larger_axis_increasement );
point1_y_start = point1_y_start - to_minus;
to_minus = mod( point1_y_end, larger_axis_increasement );
point1_y_end = point1_y_end - to_minus;

to_minus = mod( point2_x_end, axis_increasement );
point2_x_end = point2_x_end - to_minus;

to_minus = mod( point2_y_start, axis_increasement );
point2_y_start = point2_y_start - to_minus;
to_minus = mod( point2_y_end, axis_increasement );
point2_y_end = point2_y_end - to_minus;

to_minus = mod( point3_x_start, axis_increasement );
point3_x_start = point3_x_start - to_minus;
to_minus = mod( point3_x_end, axis_increasement );
point3_x_end = point3_x_end - to_minus;
        
to_minus = mod( point3_y_end, axis_increasement );
point3_y_end = point3_y_end - to_minus;

good_pose_number = 0;
AreaPoseSet_backcam_backmarker = [];  %% Quaternion and Translation set of transform T_leftcam_backmarker;
AreaPoseSet_rightcam_frontmarker = [];  %% Quaternion and Translation set of transform T_rightcam_frontmarker;
AreaPoseSet_leftcam_frontmarker = [];

for point1_x = point1_x_start:larger_axis_increasement:point1_x_end
    for point1_y = point1_y_start:larger_axis_increasement:point1_y_end
        for point2_y = point2_y_start:axis_increasement:point2_y_end
            point2_x_start = point1_x + x_axis_minimum_distance;
            to_minus = mod( point2_x_start, axis_increasement );
            point2_x_start = point2_x_start - to_minus;
            for point2_x = point2_x_start:axis_increasement:point2_x_end
                point3_y_start = point1_y + y_axis_minimum_distance;
                to_minus = mod( point3_y_start, axis_increasement );
                point3_y_start = point3_y_start - to_minus;
                for point3_y = point3_y_start:axis_increasement:point3_y_end
                    for point3_x = point3_x_start:axis_increasement:point3_x_end
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %%%%%%%%%%%%% Three Points Iteration P3P %%%%%%%%%%%%%%%%%%
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %% Each column is an image point%%
                        Projected_3Points = [point1_x   point2_x   point3_x;
                                             point1_y   point2_y   point3_y;
                                             1          1          1];
                        Projected_3Points = inv(K_backcam_intrinsic) * Projected_3Points;
                        
                        normalizer_1 = sqrt( Projected_3Points(1,1)*Projected_3Points(1,1) + Projected_3Points(2,1)*Projected_3Points(2,1) + Projected_3Points(3,1)*Projected_3Points(3,1));
                        Projected_3Points_normalized(:,1) = Projected_3Points(:,1)./[normalizer_1; normalizer_1; normalizer_1];
                        normalizer_2 = sqrt( Projected_3Points(1,2)*Projected_3Points(1,2) + Projected_3Points(2,2)*Projected_3Points(2,2) + Projected_3Points(3,2)*Projected_3Points(3,2));
                        Projected_3Points_normalized(:,2) = Projected_3Points(:,2)./[normalizer_2; normalizer_2; normalizer_2];
                        normalizer_3 = sqrt( Projected_3Points(1,3)*Projected_3Points(1,3) + Projected_3Points(2,3)*Projected_3Points(2,3) + Projected_3Points(3,3)*Projected_3Points(3,3));
                        Projected_3Points_normalized(:,3) = Projected_3Points(:,3)./[normalizer_3; normalizer_3; normalizer_3];
                        %%% T_backmarker_backcam_4_solution is a 3*16 matrix, which contains 4 solutions.
                        T_backmarker_backcam_4_solution = p3p( BackSideMarker3CornerPoints_3D, Projected_3Points_normalized );
                        
                        for j = 0:3
                            %%% FIRST project the 4-th Point on the marker board and check whether it is projected on the image plane.
                            T_backmarker_backcam =[ T_backmarker_backcam_4_solution(:,2+4*j:(4+4*j)), T_backmarker_backcam_4_solution(:,1+4*j);
                                0                0                 0                1;];
                            T_backcam_backmarker = inv(T_backmarker_backcam);
                            z_depth_backcam_backmarker = T_backcam_backmarker(3,4);
                            
                            if ( z_depth_backcam_backmarker>0 )
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %%%%%%%%% Check whether the 4th point is correctly projected %%%%%%%%%%%%%%
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                backMarker4Points_homo_3d = P_backcam * T_backcam_backmarker * BackSideMarker4CornerPoints_homog;
                                normalizer = backMarker4Points_homo_3d(3, :);
                                backMarker4Points_homo_2d = backMarker4Points_homo_3d(1:2, :);
                                backMarker4Points_homo_2d = backMarker4Points_homo_2d./normalizer;
                                backmarker_point1_x = backMarker4Points_homo_2d(1,1);
                                backmarker_point1_y = backMarker4Points_homo_2d(2,1);
                                backmarker_point2_x = backMarker4Points_homo_2d(1,2);
                                backmarker_point2_y = backMarker4Points_homo_2d(2,2);
                                backmarker_point3_x = backMarker4Points_homo_2d(1,3);
                                backmarker_point3_y = backMarker4Points_homo_2d(2,3);
                                backmarker_point4_x = backMarker4Points_homo_2d(1,4);
                                backmarker_point4_y = backMarker4Points_homo_2d(2,4);
                                point4_min_x = point3_x + x_axis_minimum_distance/2;
                                point4_min_y = point2_y + y_axis_minimum_distance/2;
                                
                                point1_x_dif = abs( backmarker_point1_x - point1_x );
                                point1_y_dif = abs( backmarker_point1_y - point1_y );
                                point2_x_dif = abs( backmarker_point2_x - point2_x );
                                point2_y_dif = abs( backmarker_point2_y - point2_y );
                                point3_x_dif = abs( backmarker_point3_x - point3_x );
                                point3_y_dif = abs( backmarker_point3_y - point3_y );
                                p3p_accuracy = 0;
                                
                                if( point1_x_dif<p3p_dif_threshold && point1_y_dif<p3p_dif_threshold && point2_x_dif<p3p_dif_threshold && ...
                                        point2_y_dif<p3p_dif_threshold && point3_x_dif<p3p_dif_threshold && point3_y_dif<p3p_dif_threshold )
                                    p3p_accuracy = 1;
                                end
                                
                                if( p3p_accuracy==1 && backmarker_point4_x>point4_min_x && backmarker_point4_x<640 && backmarker_point4_y>point4_min_y && backmarker_point4_y<480 )
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    %%%%%%%% Now check the reprojection area of back side Marker %%%%%%%
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    side_length_1 = sqrt( (backmarker_point1_x-backmarker_point2_x)^2 + (backmarker_point1_y-backmarker_point2_y)^2 );
                                    side_length_2 = sqrt( (backmarker_point1_x-backmarker_point3_x)^2 + (backmarker_point1_y-backmarker_point3_y)^2 );
                                    side_length_diag = sqrt( (backmarker_point2_x-backmarker_point3_x)^2 + (backmarker_point2_y-backmarker_point3_y)^2 );
                                    half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                    area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                    
                                    side_length_3 = sqrt( (backmarker_point3_x-backmarker_point4_x)^2 + (backmarker_point3_y-backmarker_point4_y)^2 );
                                    side_length_4 = sqrt( (backmarker_point4_x-backmarker_point2_x)^2 + (backmarker_point4_y-backmarker_point2_y)^2 );
                                    half_perim = ( side_length_3 + side_length_4 + side_length_diag )/2;
                                    area_2 = sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag));
                                    area_backMarker = area_1 + area_2;
                                    
                                    projectedArea_backmarker_ratio = area_backMarker/full_area;
                                    
                                    if( projectedArea_backmarker_ratio>projection_area_thred )
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        %%%%%% Project the front side %%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        projectedArea_frontmarker_rightcam_ratio = 0;
                                        projectedArea_frontmarker_leftcam_ratio = 0;
                                        
                                        T_leftcam_frontmarker = T_leftcam_backcam_true * T_backcam_backmarker * T_backmarker_frontmarker_true;
                                        T_rightcam_frontmarker = T_rightcam_backcam_true * T_backcam_backmarker * T_backmarker_frontmarker_true;
                                        z_depth_leftcam_frontmarker = T_leftcam_frontmarker(3,4);
                                        z_depth_rightcam_frontmarker = T_rightcam_frontmarker(3,4);
                                        
                                        if( z_depth_rightcam_frontmarker>0 || z_depth_leftcam_frontmarker>0 )
                                            frontMarker_rightcam_4Points_homo_3d = P_rightcam * T_rightcam_frontmarker * FrontSideMarker4CornerPoints_homog;
                                            normalizer = frontMarker_rightcam_4Points_homo_3d(3, :);
                                            frontMarker_rightcam_4Points_homo_2d = frontMarker_rightcam_4Points_homo_3d(1:2, :);
                                            frontMarker_rightcam_4Points_homo_2d = frontMarker_rightcam_4Points_homo_2d./normalizer;
                                            frontmarker_rightcam_point1_x = frontMarker_rightcam_4Points_homo_2d(1,1);
                                            frontmarker_rightcam_point1_y = frontMarker_rightcam_4Points_homo_2d(2,1);
                                            frontmarker_rightcam_point2_x = frontMarker_rightcam_4Points_homo_2d(1,2);
                                            frontmarker_rightcam_point2_y = frontMarker_rightcam_4Points_homo_2d(2,2);
                                            frontmarker_rightcam_point3_x = frontMarker_rightcam_4Points_homo_2d(1,3);
                                            frontmarker_rightcam_point3_y = frontMarker_rightcam_4Points_homo_2d(2,3);
                                            frontmarker_rightcam_point4_x = frontMarker_rightcam_4Points_homo_2d(1,4);
                                            frontmarker_rightcam_point4_y = frontMarker_rightcam_4Points_homo_2d(2,4);
                                            
                                            frontMarker_leftcam_4Points_homo_3d = P_leftcam * T_leftcam_frontmarker * FrontSideMarker4CornerPoints_homog;
                                            normalizer = frontMarker_leftcam_4Points_homo_3d(3, :);
                                            frontMarker_leftcam_4Points_homo_2d = frontMarker_leftcam_4Points_homo_3d(1:2, :);
                                            frontMarker_leftcam_4Points_homo_2d = frontMarker_leftcam_4Points_homo_2d./normalizer;
                                            frontmarker_leftcam_point1_x = frontMarker_leftcam_4Points_homo_2d(1,1);
                                            frontmarker_leftcam_point1_y = frontMarker_leftcam_4Points_homo_2d(2,1);
                                            frontmarker_leftcam_point2_x = frontMarker_leftcam_4Points_homo_2d(1,2);
                                            frontmarker_leftcam_point2_y = frontMarker_leftcam_4Points_homo_2d(2,2);
                                            frontmarker_leftcam_point3_x = frontMarker_leftcam_4Points_homo_2d(1,3);
                                            frontmarker_leftcam_point3_y = frontMarker_leftcam_4Points_homo_2d(2,3);
                                            frontmarker_leftcam_point4_x = frontMarker_leftcam_4Points_homo_2d(1,4);
                                            frontmarker_leftcam_point4_y = frontMarker_leftcam_4Points_homo_2d(2,4);
                                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            %%%% Check the projected points %%%
                                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            if( z_depth_rightcam_frontmarker>0 )
                                            if (    frontmarker_rightcam_point1_x>0 && frontmarker_rightcam_point1_x<640 && frontmarker_rightcam_point1_y>0 && frontmarker_rightcam_point1_y<480 && ...
                                                    frontmarker_rightcam_point2_x>0 && frontmarker_rightcam_point2_x<640 && frontmarker_rightcam_point2_y>0 && frontmarker_rightcam_point2_y<480 && ...
                                                    frontmarker_rightcam_point3_x>0 && frontmarker_rightcam_point3_x<640 && frontmarker_rightcam_point3_y>0 && frontmarker_rightcam_point3_y<480 && ...
                                                    frontmarker_rightcam_point4_x>0 && frontmarker_rightcam_point4_x<640 && frontmarker_rightcam_point4_y>0 && frontmarker_rightcam_point4_y<480)
                                                
                                                side_length_1 = sqrt( (frontmarker_rightcam_point1_x - frontmarker_rightcam_point2_x)^2 + ( frontmarker_rightcam_point1_y - frontmarker_rightcam_point2_y )^2 );
                                                side_length_2 = sqrt( (frontmarker_rightcam_point1_x - frontmarker_rightcam_point3_x)^2 + ( frontmarker_rightcam_point1_y - frontmarker_rightcam_point3_y )^2 );
                                                side_length_diag = sqrt( ( frontmarker_rightcam_point2_x - frontmarker_rightcam_point3_x )^2 + ( frontmarker_rightcam_point2_y - frontmarker_rightcam_point3_y)^2 );
                                                half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                                area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                                side_length_3 = sqrt( ( frontmarker_rightcam_point3_x - frontmarker_rightcam_point4_x )^2 + ( frontmarker_rightcam_point3_y - frontmarker_rightcam_point4_y )^2 );
                                                side_length_4 = sqrt( ( frontmarker_rightcam_point4_x - frontmarker_rightcam_point2_x )^2 + ( frontmarker_rightcam_point4_y - frontmarker_rightcam_point2_y )^2 );
                                                half_perim = ( side_length_3 + side_length_4 + side_length_diag )/2;
                                                area_2 = sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag));
                                                area_frontMarker_rightcam = area_1 + area_2;
                                                projectedArea_frontmarker_rightcam_ratio = area_frontMarker_rightcam/full_area;
                                            end
                                            end
                                            
                                            if( z_depth_leftcam_frontmarker>0)
                                            if(     frontmarker_leftcam_point1_x>0 && frontmarker_leftcam_point1_x<640 && frontmarker_leftcam_point1_y>0 && frontmarker_leftcam_point1_y<480 && ...
                                                    frontmarker_leftcam_point2_x>0 && frontmarker_leftcam_point2_x<640 && frontmarker_leftcam_point2_y>0 && frontmarker_leftcam_point2_y<480 && ...
                                                    frontmarker_leftcam_point3_x>0 && frontmarker_leftcam_point3_x<640 && frontmarker_leftcam_point3_y>0 && frontmarker_leftcam_point3_y<480 && ...
                                                    frontmarker_leftcam_point4_x>0 && frontmarker_leftcam_point4_x<640 && frontmarker_leftcam_point4_y>0 && frontmarker_leftcam_point4_y<480)
                                                
                                                side_length_1 = sqrt( (frontmarker_leftcam_point1_x - frontmarker_leftcam_point2_x)^2 + ( frontmarker_leftcam_point1_y - frontmarker_leftcam_point2_y )^2 );
                                                side_length_2 = sqrt( (frontmarker_leftcam_point1_x - frontmarker_leftcam_point3_x)^2 + ( frontmarker_leftcam_point1_y - frontmarker_leftcam_point3_y )^2 );
                                                side_length_diag = sqrt( ( frontmarker_leftcam_point2_x - frontmarker_leftcam_point3_x )^2 + ( frontmarker_leftcam_point2_y - frontmarker_leftcam_point3_y)^2 );
                                                half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                                area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                                side_length_3 = sqrt( ( frontmarker_leftcam_point3_x - frontmarker_leftcam_point4_x )^2 + ( frontmarker_leftcam_point3_y - frontmarker_leftcam_point4_y )^2 );
                                                side_length_4 = sqrt( ( frontmarker_leftcam_point4_x - frontmarker_leftcam_point2_x )^2 + ( frontmarker_leftcam_point4_y - frontmarker_leftcam_point2_y )^2 );
                                                half_perim = ( side_length_3 + side_length_4 + side_length_diag )/2;
                                                area_2 = sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag));
                                                area_frontMarker_leftcam = area_1 + area_2;
                                                projectedArea_frontmarker_leftcam_ratio = area_frontMarker_leftcam/full_area;
                                            end
                                            end
                                            
                                            if ( projectedArea_frontmarker_rightcam_ratio>projection_area_thred || projectedArea_frontmarker_leftcam_ratio>projection_area_thred )
                                                
                                                if( good_pose_number==0 )
                                                    good_pose_number = good_pose_number + 1;
                                                    
                                                    r_backcam_backmarker = T_backcam_backmarker(1:3, 1:3);
                                                    q_backcam_backmarker = rotm2quat( r_backcam_backmarker );
                                                    t_backcam_backmarker = T_backcam_backmarker(1:3,4);
                                                    t_backcam_backmarker = transpose( t_backcam_backmarker );
                                                    AreaPoseSet_backcam_backmarker = [ AreaPoseSet_backcam_backmarker; projectedArea_backmarker_ratio, q_backcam_backmarker, t_backcam_backmarker ];
                                                    
                                                    if( projectedArea_frontmarker_rightcam_ratio>projection_area_thred )
                                                        r_rightcam_frontmarker = T_rightcam_frontmarker(1:3, 1:3);
                                                        q_rightcam_frontmarker = rotm2quat( r_rightcam_frontmarker );
                                                        t_rightcam_frontmarker = T_rightcam_frontmarker(1:3,4);
                                                        t_rightcam_frontmarker = transpose( t_rightcam_frontmarker );
                                                        AreaPoseSet_rightcam_frontmarker = [AreaPoseSet_rightcam_frontmarker; projectedArea_frontmarker_rightcam_ratio, q_rightcam_frontmarker t_rightcam_frontmarker ];
                                                    else
                                                        AreaPoseSet_rightcam_frontmarker = [AreaPoseSet_rightcam_frontmarker; 0, 0, 0, 0, 0, 0, 0, 0];
                                                    end
                                                    
                                                    if( projectedArea_frontmarker_leftcam_ratio>projection_area_thred )
                                                        r_leftcam_frontmarker = T_leftcam_frontmarker(1:3, 1:3);
                                                        q_leftcam_frontmarker = rotm2quat( r_leftcam_frontmarker );
                                                        t_leftcam_frontmarker = T_leftcam_frontmarker(1:3,4);
                                                        t_leftcam_frontmarker = transpose( t_leftcam_frontmarker );
                                                        AreaPoseSet_leftcam_frontmarker = [AreaPoseSet_leftcam_frontmarker; projectedArea_frontmarker_leftcam_ratio, q_leftcam_frontmarker t_leftcam_frontmarker ];
                                                    else
                                                        AreaPoseSet_leftcam_frontmarker = [AreaPoseSet_leftcam_frontmarker; 0, 0, 0, 0, 0, 0, 0, 0];
                                                    end
                                                end
                                                
                                                if( good_pose_number>0 )
                                                    need_to_add_rightcam_pose = 1;
                                                    need_to_add_leftcam_pose = 1;
                                                    
                                                    [set_size, columns] = size(AreaPoseSet_backcam_backmarker);
                                                    pose_difference_index_backcam = 1;
                                                    pose_difference_index_rightcam = 1;
                                                    pose_difference_index_leftcam = 1;
                                                    
                                                    rotm1_to_add = T_backcam_backmarker(1:3,1:3);
                                                    quaternion1_to_add = rotm2quat(rotm1_to_add);
                                                    translation1_to_add = T_backcam_backmarker(1:3,4);
                                                    translation1_to_add = transpose( translation1_to_add );
                                                    
                                                    if( projectedArea_frontmarker_rightcam_ratio>projection_area_thred )
                                                        rotm2_to_add = T_rightcam_frontmarker(1:3,1:3);
                                                        quaternion2_to_add = rotm2quat(rotm2_to_add);
                                                        translation2_to_add = T_rightcam_frontmarker(1:3,4);
                                                        translation2_to_add = transpose( translation2_to_add );
                                                    else
                                                        need_to_add_rightcam_pose = 0;
                                                    end
                                                    
                                                    if( projectedArea_frontmarker_leftcam_ratio>projection_area_thred )
                                                        rotm3_to_add = T_leftcam_frontmarker(1:3,1:3);
                                                        quaternion3_to_add = rotm2quat(rotm3_to_add);
                                                        translation3_to_add = T_leftcam_frontmarker(1:3,4);
                                                        translation3_to_add = transpose( translation3_to_add );
                                                    else
                                                        need_to_add_leftcam_pose = 0;
                                                    end
                                                    
                                                    for index = 1:set_size
                                                        
                                                        quat1_to_compare = AreaPoseSet_backcam_backmarker(index, 2:5);
                                                        trans1_to_compare = AreaPoseSet_backcam_backmarker(index, 6:8);
                                                        quaternion1_difference_value = compare_quaternion_difference( quaternion1_to_add, quat1_to_compare );
                                                        translation1_difference_value = compare_sum_of_3dim_difference( translation1_to_add, trans1_to_compare);
                                                        
                                                        if ( translation1_difference_value<minimum_translation_thred && quaternion1_difference_value<minimum_rotation_thred )
                                                            pose_difference_index_backcam = 0;
                                                            break;
                                                        end
                                                        
                                                        if( need_to_add_rightcam_pose )
                                                            quat2_to_compare = AreaPoseSet_rightcam_frontmarker(index, 2:5);
                                                            trans2_to_compare = AreaPoseSet_rightcam_frontmarker(index, 6:8);
                                                            quaternion2_difference_value = compare_quaternion_difference( quaternion2_to_add, quat2_to_compare );
                                                            translation2_difference_value = compare_sum_of_3dim_difference( translation2_to_add, trans2_to_compare);
                                                            
                                                            if ( translation2_difference_value<minimum_translation_thred && quaternion2_difference_value<minimum_rotation_thred )
                                                                pose_difference_index_rightcam = 0;
                                                                break;
                                                            end
                                                        end
                                                        
                                                        if( need_to_add_leftcam_pose )
                                                            quat3_to_compare = AreaPoseSet_leftcam_frontmarker(index, 2:5);
                                                            trans3_to_compare = AreaPoseSet_leftcam_frontmarker(index, 6:8);
                                                            quaternion3_difference_value = compare_quaternion_difference( quaternion3_to_add, quat3_to_compare );
                                                            translation3_difference_value = compare_sum_of_3dim_difference( translation3_to_add, trans3_to_compare);
                                                            
                                                            if ( translation3_difference_value<minimum_translation_thred && quaternion3_difference_value<minimum_rotation_thred )
                                                                pose_difference_index_leftcam = 0;
                                                                break;
                                                            end
                                                        end
                                                    end
                                                    
                                                    if (    pose_difference_index_backcam==1 && ( (need_to_add_rightcam_pose==0) || (need_to_add_rightcam_pose && pose_difference_index_rightcam==1) ) &&...
                                                            ( (need_to_add_leftcam_pose==0) || (need_to_add_leftcam_pose && pose_difference_index_leftcam==1) ) &&...
                                                            ( need_to_add_rightcam_pose || need_to_add_leftcam_pose )   ) 
                                                        
                                                        good_pose_number = good_pose_number + 1;
                                                        AreaPoseSet_backcam_backmarker = [AreaPoseSet_backcam_backmarker; projectedArea_backmarker_ratio, quaternion1_to_add, translation1_to_add ];
                                                        
                                                        if( need_to_add_rightcam_pose )
                                                            AreaPoseSet_rightcam_frontmarker = [AreaPoseSet_rightcam_frontmarker; projectedArea_frontmarker_rightcam_ratio, quaternion2_to_add, translation2_to_add];
                                                        else
                                                            AreaPoseSet_rightcam_frontmarker = [AreaPoseSet_rightcam_frontmarker; 0, 0, 0, 0, 0, 0, 0, 0];
                                                        end
                                                        
                                                        if( need_to_add_leftcam_pose )
                                                            AreaPoseSet_leftcam_frontmarker = [AreaPoseSet_leftcam_frontmarker; projectedArea_frontmarker_leftcam_ratio, quaternion3_to_add, translation3_to_add];
                                                        else
                                                            AreaPoseSet_leftcam_frontmarker = [AreaPoseSet_leftcam_frontmarker; 0, 0, 0, 0, 0, 0, 0, 0];  
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    end 
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end


h = msgbox('Operation Completed.');