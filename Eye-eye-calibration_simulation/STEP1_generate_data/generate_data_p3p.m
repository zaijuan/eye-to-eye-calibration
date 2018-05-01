% clear the screen
clear all;
close all;
clc;

% Define the 3-D coordinates of the marker features. 
%In this simulation, to simply the process, the calibration boards mounted on both sides of the rig are the same, 
%which means the coordinates represented in each marker's frame are the same. 
%In this measurement generation process, we use 4 feature points while for the optimization we use 25 points.

%%% Used for matlab function "estimateWorldCameraPose".
LeftSideMarker4CornerPoints_3D = [-0.2    0.2    0;
                                   0.2    0.2    0;
                                  -0.2   -0.2    0;
                                   0.2   -0.2    0];
                               
%%% Used for P3P as inputs. Each column is an point.%%%                               
LeftSideMarker3CornerPoints_3D = [ -0.2   0.2   -0.2;
                                    0.2   0.2   -0.2;
                                    0     0      0]; 
                                
%%% Used for 3D-2D projection process.                               
LeftSideMarker4CornerPoints_homog = [-0.2     0.2    -0.2     0.2;
                                      0.2     0.2    -0.2    -0.2;
                                      0       0       0       0;
                                      1       1       1       1];
                                  
%%% Used for matlab function "estimateWorldCameraPose".                               
RightSideMarker4CornerPoints_3D = [-0.2    0.2    0;
                                    0.2    0.2    0;
                                   -0.2   -0.2    0;
                                    0.2   -0.2    0];
                                
%%% Used for 3D-2D projection process.                                                               
RightSideMarker4CornerPoints_homog = [-0.2     0.2    -0.2     0.2;
                                       0.2     0.2    -0.2    -0.2;
                                       0       0       0       0;
                                       1       1       1       1];
focal_length = 500;
principal_point_x = 320;
principal_point_y = 240;

focal_length_vec = [focal_length  focal_length];                                
principalPoint_vec = [principal_point_x  principal_point_y];
imageSize_vec = [principal_point_x*2  principal_point_y*2];

% In this simulation we assume that the two cameras used in the experiment have the following intrinsic parameters.
K_leftcam = cameraIntrinsics( focal_length_vec, principalPoint_vec, imageSize_vec );
K_rightcam = cameraIntrinsics( focal_length_vec, principalPoint_vec, imageSize_vec );

% Corresponding projection matrix:
K_leftcam_intrinsic = [focal_length       0                 principal_point_x;
                       0                  focal_length      principal_point_y;
                       0                  0                 1];
                   
P_leftcam = [focal_length       0                 principal_point_x     0;
             0                  focal_length      principal_point_y     0;
             0                  0                 1                     0];

K_rightcam_intrinsic = [focal_length      0                 principal_point_x;
                        0                 focal_length      principal_point_y;
                        0                 0                 1];
                       
% Corresponding projection matrix:
P_rightcam = [focal_length      0                principal_point_x     0;
              0                 focal_length     principal_point_y     0;
              0                 0                1                     0];

LeftCamParams = cameraParameters('IntrinsicMatrix', K_leftcam.IntrinsicMatrix, 'WorldUnits', 'm', 'ImageSize', [640,480]);
RightCamParams = cameraParameters('IntrinsicMatrix', K_rightcam.IntrinsicMatrix, 'WorldUnits', 'm','ImageSize', [640,480]);

% Since the car is not moving, choose the frame of the leftside camera mounted on the car as the world reference frame.
% W represents the world frame, leftcam means the left camera frame, in this experiment, they are aligned.
T_W_leftcam = [1   0   0   0;
               0   1   0   0;
               0   0   1   0;
               0   0   0   1]; 

% The right camera board mounted on the right side of the car also has its own coordinate frame.
T_leftcam_rightcam_true = [     0.5          0          sqrt(3)/2      0.6*sqrt(3);
                                 0           1            0               0;
                               -sqrt(3)/2    0            0.5            -0.6;
                                 0           0            0               1];
                             
% W represents the world frame, rightcam means the right camera frame, T_W_rightcam is the ground truth of unknown Y.
% AX = YB.
T_W_rightcam = T_leftcam_rightcam_true;     
T_rightcam_leftcam_true = inv(T_leftcam_rightcam_true);

half_projection_points_2d = [ 160  80; 480 80; 160 400; 480 400];
[ R_world_leftcam, t_world_leftcam ] = estimateWorldCameraPose( half_projection_points_2d, LeftSideMarker4CornerPoints_3D, LeftCamParams);
T_leftmarker_leftcam_centerprojected = [R_world_leftcam, transpose(t_world_leftcam);
                                        0        0        0       1];

[ R_world_rightcam, t_world_rightcam ] = estimateWorldCameraPose( half_projection_points_2d, LeftSideMarker4CornerPoints_3D, LeftCamParams);
T_rightmarker_rightcam_centerprojected = [R_world_rightcam, transpose(t_world_rightcam);
                                          0        0       0       1];

% This is the ground truth of X in the equation AX = YB.               
T_leftmarker_rightmarker_true = T_leftmarker_leftcam_centerprojected * T_leftcam_rightcam_true * inv(T_rightmarker_rightcam_centerprojected);                              
T_rightmarker_leftmarker_true = inv( T_leftmarker_rightmarker_true );

x_axis_minimum_distance = 120;
y_axis_minimum_distance = 100;

axis_increasement = 10;
larger_axis_increasement = 20;
projection_area_thred = 0.1;

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

load AreaPoseSet_leftcam_leftmarker_240.mat;  %% Quaternion and Translation set of transform T_leftcam_leftmarker;
load AreaPoseSet_rightcam_rightmarker_240.mat;  %% Quaternion and Translation set of transform T_rightcam_rightmarker;
[ good_pose_number, columns ] = size(AreaPoseSet_leftcam_leftmarker);

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
                        Projected_3Points = inv(K_leftcam_intrinsic) * Projected_3Points;
                        
                        normalizer_1 = sqrt( Projected_3Points(1,1)*Projected_3Points(1,1) + Projected_3Points(2,1)*Projected_3Points(2,1) + Projected_3Points(3,1)*Projected_3Points(3,1));
                        Projected_3Points_normalized(:,1) = Projected_3Points(:,1)./[normalizer_1; normalizer_1; normalizer_1];
                        normalizer_2 = sqrt( Projected_3Points(1,2)*Projected_3Points(1,2) + Projected_3Points(2,2)*Projected_3Points(2,2) + Projected_3Points(3,2)*Projected_3Points(3,2));
                        Projected_3Points_normalized(:,2) = Projected_3Points(:,2)./[normalizer_2; normalizer_2; normalizer_2];
                        normalizer_3 = sqrt( Projected_3Points(1,3)*Projected_3Points(1,3) + Projected_3Points(2,3)*Projected_3Points(2,3) + Projected_3Points(3,3)*Projected_3Points(3,3));
                        Projected_3Points_normalized(:,3) = Projected_3Points(:,3)./[normalizer_3; normalizer_3; normalizer_3];
                        %%% T_leftmarker_leftcam_4_solution is a 3*16 matrix, which contains 4 solutions.
                        T_leftmarker_leftcam_4_solution = p3p( LeftSideMarker3CornerPoints_3D, Projected_3Points_normalized );
                        
                        for j = 0:3
                            %%% FIRST project the 4-th Point on the marker board and check whether it is projected on the image plane.
                            T_leftmarker_leftcam =[ T_leftmarker_leftcam_4_solution(:,2+4*j:(4+4*j)), T_leftmarker_leftcam_4_solution(:,1+4*j);
                                                    0              0               0              1;];
                            T_leftcam_leftmarker = inv(T_leftmarker_leftcam);
                            z_depth_leftcam_leftmarker = T_leftcam_leftmarker(3,4);
                            
                            if ( z_depth_leftcam_leftmarker>0 )
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %%%%%%%%% Check whether the 4th point is correctly projected %%%%%%%%%%%%%%
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                leftMarker4Points_homo_3d = P_leftcam * T_leftcam_leftmarker * LeftSideMarker4CornerPoints_homog;
                                normalizer = leftMarker4Points_homo_3d(3, :);
                                leftMarker4Points_homo_2d = leftMarker4Points_homo_3d(1:2, :);
                                leftMarker4Points_homo_2d = leftMarker4Points_homo_2d./normalizer;
                                leftmarker_point1_x = leftMarker4Points_homo_2d(1,1);
                                leftmarker_point1_y = leftMarker4Points_homo_2d(2,1);
                                leftmarker_point2_x = leftMarker4Points_homo_2d(1,2);
                                leftmarker_point2_y = leftMarker4Points_homo_2d(2,2);
                                leftmarker_point3_x = leftMarker4Points_homo_2d(1,3);
                                leftmarker_point3_y = leftMarker4Points_homo_2d(2,3);
                                leftmarker_point4_x = leftMarker4Points_homo_2d(1,4);
                                leftmarker_point4_y = leftMarker4Points_homo_2d(2,4);
                                point4_min_x = point3_x + x_axis_minimum_distance/2;
                                point4_min_y = point2_y + y_axis_minimum_distance/2;
                                
                                point1_x_dif = abs( leftmarker_point1_x - point1_x );
                                point1_y_dif = abs( leftmarker_point1_y - point1_y );
                                point2_x_dif = abs( leftmarker_point2_x - point2_x );
                                point2_y_dif = abs( leftmarker_point2_y - point2_y );
                                point3_x_dif = abs( leftmarker_point3_x - point3_x );
                                point3_y_dif = abs( leftmarker_point3_y - point3_y );
                                p3p_accuracy = 0;
                                if( point1_x_dif<p3p_dif_threshold && point1_y_dif<p3p_dif_threshold && point2_x_dif<p3p_dif_threshold && ...
                                    point2_y_dif<p3p_dif_threshold && point3_x_dif<p3p_dif_threshold && point3_y_dif<p3p_dif_threshold )
                                    p3p_accuracy = 1;
                                end

                                if( p3p_accuracy==1 && leftmarker_point4_x>point4_min_x && leftmarker_point4_x<640 && leftmarker_point4_y>point4_min_y && leftmarker_point4_y<480 )     
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    %%%%%%%% Now check the reprojection area of left side Marker %%%%%%%
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    side_length_1 = sqrt( (leftmarker_point1_x-leftmarker_point2_x)^2 + (leftmarker_point1_y-leftmarker_point2_y)^2 );
                                    side_length_2 = sqrt( (leftmarker_point1_x-leftmarker_point3_x)^2 + (leftmarker_point1_y-leftmarker_point3_y)^2 );
                                    side_length_diag = sqrt( (leftmarker_point2_x-leftmarker_point3_x)^2 + (leftmarker_point2_y-leftmarker_point3_y)^2 );
                                    half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                    area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                    
                                    side_length_3 = sqrt( (leftmarker_point3_x-leftmarker_point4_x)^2 + (leftmarker_point3_y-leftmarker_point4_y)^2 );
                                    side_length_4 = sqrt( (leftmarker_point4_x-leftmarker_point2_x)^2 + (leftmarker_point4_y-leftmarker_point2_y)^2 );
                                    half_perim = ( side_length_3 + side_length_4 + side_length_diag )/2;
                                    area_2 = sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag));
                                    area_leftMarker = area_1 + area_2;
                                    
                                    projectedArea_leftmarker_ratio = area_leftMarker/full_area;
                                    
                                    if( projectedArea_leftmarker_ratio>projection_area_thred )
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        %%%%%% Project the other side %%%%%
                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        within_projected_rightmarker = 0;
                                        T_rightcam_rightmarker = T_rightcam_leftcam_true * T_leftcam_leftmarker * T_leftmarker_rightmarker_true;
                                        z_depth_rightcam_rightmarker = T_rightcam_rightmarker(3,4);
                                        
                                        if( z_depth_rightcam_rightmarker>0 )
                                            rightMarker4Points_homo_3d = P_rightcam * T_rightcam_rightmarker * RightSideMarker4CornerPoints_homog;
                                            normalizer = rightMarker4Points_homo_3d(3, :);
                                            rightMarker4Points_homo_2d = rightMarker4Points_homo_3d(1:2, :);
                                            rightMarker4Points_homo_2d = rightMarker4Points_homo_2d./normalizer;
                                            rightmarker_point1_x = rightMarker4Points_homo_2d(1,1);
                                            rightmarker_point1_y = rightMarker4Points_homo_2d(2,1);
                                            rightmarker_point2_x = rightMarker4Points_homo_2d(1,2);
                                            rightmarker_point2_y = rightMarker4Points_homo_2d(2,2);
                                            rightmarker_point3_x = rightMarker4Points_homo_2d(1,3);
                                            rightmarker_point3_y = rightMarker4Points_homo_2d(2,3);
                                            rightmarker_point4_x = rightMarker4Points_homo_2d(1,4);
                                            rightmarker_point4_y = rightMarker4Points_homo_2d(2,4);
                                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            %%%% Check the projected points %%%
                                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            if (    rightmarker_point1_x>0 && rightmarker_point1_x<640 && rightmarker_point1_y>0 && rightmarker_point1_y<480 && ...
                                                    rightmarker_point2_x>0 && rightmarker_point2_x<640 && rightmarker_point2_y>0 && rightmarker_point2_y<480 && ...
                                                    rightmarker_point3_x>0 && rightmarker_point3_x<640 && rightmarker_point3_y>0 && rightmarker_point3_y<480 && ...
                                                    rightmarker_point4_x>0 && rightmarker_point4_x<640 && rightmarker_point4_y>0 && rightmarker_point4_y<480 )
                                                
                                                within_projected_rightmarker = 1;
                                                if( within_projected_rightmarker )
                                                    
                                                    side_length_1 = sqrt( (rightmarker_point1_x - rightmarker_point2_x)^2 + ( rightmarker_point1_y - rightmarker_point2_y )^2 );
                                                    side_length_2 = sqrt( (rightmarker_point1_x - rightmarker_point3_x)^2 + ( rightmarker_point1_y - rightmarker_point3_y )^2 );
                                                    side_length_diag = sqrt( ( rightmarker_point2_x - rightmarker_point3_x )^2 + ( rightmarker_point2_y - rightmarker_point3_y)^2 );
                                                    half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                                    area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                                    side_length_3 = sqrt( ( rightmarker_point3_x - rightmarker_point4_x )^2 + ( rightmarker_point3_y - rightmarker_point4_y )^2 );
                                                    side_length_4 = sqrt( ( rightmarker_point4_x - rightmarker_point2_x )^2 + ( rightmarker_point4_y - rightmarker_point2_y )^2 );
                                                    half_perim = ( side_length_3 + side_length_4 + side_length_diag )/2;
                                                    area_2 = sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag));
                                                    area_rightMarker = area_1 + area_2;
                                                    projectedArea_rightmarker_ratio = area_rightMarker/full_area;
                                                    
                                                    if ( projectedArea_rightmarker_ratio>projection_area_thred )
                                                        
                                                        if( good_pose_number==0 )
                                                            good_pose_number = good_pose_number + 1;
                                                            
                                                            r_leftcam_leftmarker = T_leftcam_leftmarker(1:3, 1:3);
                                                            q_leftcam_leftmarker = rotm2quat( r_leftcam_leftmarker );
                                                            t_leftcam_leftmarker = T_leftcam_leftmarker(1:3,4);
                                                            t_leftcam_leftmarker = transpose( t_leftcam_leftmarker );
                                                            AreaPoseSet_leftcam_leftmarker = [ AreaPoseSet_leftcam_leftmarker; projectedArea_leftmarker_ratio, q_leftcam_leftmarker, t_leftcam_leftmarker ];
                                                            
                                                            r_rightcam_rightmarker = T_rightcam_rightmarker(1:3, 1:3);
                                                            q_rightcam_rightmarker = rotm2quat( r_rightcam_rightmarker );
                                                            t_rightcam_rightmarker = T_rightcam_rightmarker(1:3,4);
                                                            t_rightcam_rightmarker = transpose( t_rightcam_rightmarker );
                                                            AreaPoseSet_rightcam_rightmarker = [AreaPoseSet_rightcam_rightmarker; projectedArea_rightmarker_ratio, q_rightcam_rightmarker t_rightcam_rightmarker ];
                                                        end
                                                        
                                                        if( good_pose_number>0 )
                                                            [set_size, columns] = size(AreaPoseSet_leftcam_leftmarker);
                                                            pose_difference_index_1 = 1;
                                                            pose_difference_index_2 = 1;
                                                            
                                                            rotm1_to_add = T_leftcam_leftmarker(1:3,1:3);
                                                            quaternion1_to_add = rotm2quat(rotm1_to_add);
                                                            translation1_to_add = T_leftcam_leftmarker(1:3,4);
                                                            translation1_to_add = transpose( translation1_to_add );
                                                            
                                                            rotm2_to_add = T_rightcam_rightmarker(1:3,1:3);
                                                            quaternion2_to_add = rotm2quat(rotm2_to_add);
                                                            translation2_to_add = T_rightcam_rightmarker(1:3,4);
                                                            translation2_to_add = transpose( translation2_to_add );
                                                            
                                                            for index = 1:set_size
                                                                
                                                                quat1_to_compare = AreaPoseSet_leftcam_leftmarker(index, 2:5);
                                                                trans1_to_compare = AreaPoseSet_leftcam_leftmarker(index, 6:8);
                                                                quaternion1_difference_value = compare_quaternion_difference( quaternion1_to_add, quat1_to_compare );
                                                                translation1_difference_value = compare_sum_of_3dim_difference( translation1_to_add, trans1_to_compare);
                                                                
                                                                if ( translation1_difference_value<minimum_translation_thred && quaternion1_difference_value<minimum_rotation_thred )
                                                                    pose_difference_index_1 = 0;
                                                                    break;
                                                                end
                                                                
                                                                quat2_to_compare = AreaPoseSet_rightcam_rightmarker(index, 2:5);
                                                                trans2_to_compare = AreaPoseSet_rightcam_rightmarker(index, 6:8);
                                                                quaternion2_difference_value = compare_quaternion_difference( quaternion2_to_add, quat2_to_compare );
                                                                translation2_difference_value = compare_sum_of_3dim_difference( translation2_to_add, trans2_to_compare);
                                                                
                                                                if ( translation2_difference_value<minimum_translation_thred && quaternion2_difference_value<minimum_rotation_thred )
                                                                    pose_difference_index_2 = 0;
                                                                    break;
                                                                end
                                                            end
                                                            
                                                            if ( pose_difference_index_1==1 && pose_difference_index_2==1 ) 
                                                                good_pose_number = good_pose_number + 1;
                                                                AreaPoseSet_leftcam_leftmarker = [AreaPoseSet_leftcam_leftmarker; projectedArea_leftmarker_ratio, quaternion1_to_add, translation1_to_add ];
                                                                AreaPoseSet_rightcam_rightmarker = [AreaPoseSet_rightcam_rightmarker; projectedArea_rightmarker_ratio, quaternion2_to_add, translation2_to_add];
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
end

h = msgbox('Operation Completed.');
