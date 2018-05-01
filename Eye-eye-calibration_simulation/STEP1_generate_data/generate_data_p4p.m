
% clear the screen
clear all;
close all;
clc;


% Define the 3-D coordinates of the marker features. In this simulation, to simply the process, the calibration boards mounted on both sides of the rig are the same, which means the
% coordinates represented in each marker's frame are the same. In this measurement generation process, we use 9 feature points while for the optimization we use 25 points.

LeftSideMarker4CornerPoints_3D = [-0.2    0.2    0;
                                   0.2    0.2    0;
                                  -0.2   -0.2    0;
                                   0.2   -0.2    0];
                               
RightSideMarker4CornerPoints_3D = [-0.2    0.2    0;
                                    0.2    0.2    0;
                                   -0.2   -0.2    0;
                                    0.2   -0.2    0];                               
RightSideMarker4CornerPoints_4D = [-0.2    0.2    0   1;
                                    0.2    0.2    0   1;
                                   -0.2   -0.2    0   1;
                                    0.2   -0.2    0   1];
focal_length = [600 600];                                
principalPoint = [320 240];
imageSize = [640 480];

% In this simulation we assume that the two cameras used in the experiment have the following intrinsic parameters.

K_leftcam = cameraIntrinsics( focal_length, principalPoint, imageSize );
K_rightcam = cameraIntrinsics( focal_length, principalPoint, imageSize );
% Corresponding projection matrix:
K_leftcam_intrinsic = [600       0        320;
                       0         600      240;
                       0         0        1];
                   
P_leftcam = [600       0      320     0;
             0       600      240     0;
             0         0       1      0];

K_rightcam_intrinsic = [600      0       320;
                       0        600      240;
                       0        0        1];
                       
% Corresponding projection matrix:
P_rightcam = [600      0     320     0;
                0    600     240     0;
                0      0      1      0];

LeftCamParams = cameraParameters('IntrinsicMatrix', K_leftcam.IntrinsicMatrix, 'WorldUnits', 'm', 'ImageSize', [640,480]);
RightCamParams = cameraParameters('IntrinsicMatrix', K_rightcam.IntrinsicMatrix, 'WorldUnits', 'm','ImageSize', [640,480]);

% Since the car is not moving, choose the frame of the leftside camera mounted on the car as the world reference frame.
T_W_leftcam = [1   0   0   0;
               0   1   0   0;
               0   0   1   0;
               0   0   0   1]; % W represents the world frame, leftcam means the left camera frame, in this experiment, they are aligned.

% The right camera board mounted on the right side of the car also has its own coordinate frame.
T_leftcam_rightcam_true = [     0.5          0          sqrt(3)/2      0.6*sqrt(3);
                                 0           1            0               0;
                               -sqrt(3)/2    0            0.5             0.6;
                                 0           0            0               1];
T_W_rightcam = T_leftcam_rightcam_true;      % W represents the world frame, rightcam means the right camera frame, T_W_rightcam is the ground truth of unknown X.
T_rightcam_leftcam_true = inv(T_leftcam_rightcam_true);

half_projection_points_2d = [ 160  80; 480 80; 160 400; 480 400];
[ R_world_leftcam, t_world_leftcam ] = estimateWorldCameraPose( half_projection_points_2d, LeftSideMarker4CornerPoints_3D, LeftCamParams);
T_leftmarker_leftcam_centerprojected = [R_world_leftcam, transpose(t_world_leftcam);
                                        0        0        0       1];

[ R_world_rightcam, t_world_rightcam ] = estimateWorldCameraPose( half_projection_points_2d, RightSideMarker4CornerPoints_3D, RightCamParams);
T_rightmarker_rightcam_centerprojected = [R_world_rightcam, transpose(t_world_rightcam);
                                          0        0       0       1];
               
T_leftmarker_rightmarker_true = T_leftmarker_leftcam_centerprojected * T_leftcam_rightcam_true * inv(T_rightmarker_rightcam_centerprojected);                              
T_rightmarker_leftmarker_true = inv( T_leftmarker_rightmarker_true );


x_axis_minimum_distance = 100;
y_axis_minimum_distance = 80;

axis_increasement = 5;
larger_axis_increasement = 10;
x_axis_maximum = 640;
y_axis_maximum = 480;

full_area = x_axis_maximum*y_axis_maximum;
projection_area_thred = 0.1;

minimum_translation_thred = 0.05;
minimum_rotation_thred = 0.04;

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
point4_x_start = 0 + 0;   point4_x_end = 620;
point4_y_start = 0 + 0;   point4_y_end = 460;

point1_x_to_minus = mod( point1_x_start, larger_axis_increasement );
point1_x_start = point1_x_start - point1_x_to_minus;
point1_x_to_minus = mod( point1_x_end, larger_axis_increasement );
point1_x_end = point1_x_end - point1_x_to_minus;

point1_y_to_minus = mod( point1_y_start, larger_axis_increasement );
point1_y_start = point1_y_start - point1_y_to_minus;
point1_y_to_minus = mod( point1_y_end, larger_axis_increasement );
point1_y_end = point1_y_end - point1_y_to_minus;

point2_x_to_minus = mod( point2_x_end, axis_increasement );
point2_x_end = point2_x_end - point2_x_to_minus;

point2_y_to_minus = mod( point2_y_start, axis_increasement );
point2_y_start = point2_y_start - point2_y_to_minus;
point2_y_to_minus = mod( point2_y_end, axis_increasement );
point2_y_end = point2_y_end - point2_y_to_minus;

point3_x_to_minus = mod( point3_x_start, axis_increasement );
point3_x_start = point3_x_start - point3_x_to_minus;
point3_x_to_minus = mod( point3_x_end, axis_increasement );
point3_x_end = point3_x_end - point3_x_to_minus;
        
point3_y_to_minus = mod( point3_y_end, axis_increasement );
point3_y_end = point3_y_end - point3_y_to_minus;

good_pose_number = 0;
Poseset_leftcam_leftmarker = [];  %% Quaternion and Translation set of transform T_leftcam_leftmarker;
Areaset_leftcam_leftmarker = [];
Poseset_rightcam_rightmarker = [];  %% Quaternion and Translation set of transform T_rightcam_rightmarker;
Areaset_rightcam_rightmarker = [];

for point1_x = point1_x_start:larger_axis_increasement:point1_x_end
    for point1_y = point1_y_start:larger_axis_increasement:point1_y_end
        for point2_y = point2_y_start:axis_increasement:point2_y_end
            point2_x_start = point1_x + x_axis_minimum_distance;
            point2_x_to_minus = mod( point2_x_start, axis_increasement );
            point2_x_start = point2_x_start - point2_x_to_minus;
            for point2_x = point2_x_start:axis_increasement:point2_x_end
                point3_y_start = point1_y + y_axis_minimum_distance;
                point3_y_to_minus = mod( point3_y_start, axis_increasement );
                point3_y_start = point3_y_start - point3_y_to_minus;
                for point3_y = point3_y_start:axis_increasement:point3_y_end
                    for point3_x = point3_x_start:axis_increasement:point3_x_end
                        point4_x_start = point3_x + x_axis_minimum_distance;
                        point4_x_to_minus = mod( point4_x_start, axis_increasement );
                        point4_x_start = point4_x_start - point4_x_to_minus;
                        point4_y_start = point2_y + y_axis_minimum_distance;
                        point4_y_to_minus = mod( point4_y_start, axis_increasement );
                        point4_y_start = point4_y_start - point4_y_to_minus;
                        for point4_x = point4_x_start:axis_increasement:point4_x_end
                            for point4_y = point4_y_start:axis_increasement:point4_y_end
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                %%%%%%%%%%%%%%%%%%%%       P4P      %%%%%%%%%%%%%%%%%%%%%%%%
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                corners_projection_points_2d = [ point1_x  point1_y; point2_x point2_y; point3_x point3_y; point4_x point4_y];
                                [ R_world_leftcam, t_world_leftcam, inlierIdx, status_code ] = estimateWorldCameraPose( corners_projection_points_2d, LeftSideMarker4CornerPoints_3D, LeftCamParams);
                                
                                if( status_code==0 )
                                    T_leftmarker_leftcam = [R_world_leftcam, transpose(t_world_leftcam);
                                        0        0        0       1];
                                    T_leftcam_leftmarker = inv( T_leftmarker_leftcam );
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    %%%%%%%% Now check the reprojection area of left side Marker %%%%%%%
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                    side_length_1 = sqrt( (point1_x-point2_x)^2 + (point1_y-point2_y)^2 );
                                    side_length_2 = sqrt( (point1_x-point3_x)^2 + (point1_y-point3_y)^2 );
                                    side_length_diag = sqrt( (point2_x-point3_x)^2 + (point2_y-point3_y)^2 );
                                    half_perim = ( side_length_1 + side_length_2 + side_length_diag )/2;
                                    area_1 = sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag));
                                    side_length_3 = sqrt( (point3_x-point4_x)^2 + (point3_y-point4_y)^2 );
                                    side_length_4 = sqrt( (point4_x-point2_x)^2 + (point4_y-point2_y)^2 );
                                    side_length_diag = sqrt( (point2_x-point3_x)^2 + (point2_y-point3_y)^2 );
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
                                        z_depth = T_rightcam_rightmarker(3,4);
                                        
                                        if( z_depth>0 )
                                            rightMarker4Points_homo_3d = P_rightcam * T_rightcam_rightmarker * RightSideMarker4CornerPoints_4D;
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
                                            if (    rightmarker_point1_x>0 && rightmarker_point1_x<6400 && rightmarker_point1_y>0 && rightmarker_point1_y<480 && ...
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
                                                    side_length_diag = sqrt( ( rightmarker_point2_x- rightmarker_point3_x)^2 + ( rightmarker_point2_y - rightmarker_point3_y)^2 );
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
                                                            q_t_leftcam_leftmarker = [q_leftcam_leftmarker t_leftcam_leftmarker];
                                                            Poseset_leftcam_leftmarker = [Poseset_leftcam_leftmarker; q_t_leftcam_leftmarker];
                                                            Areaset_leftcam_leftmarker = [Areaset_leftcam_leftmarker; projectedArea_leftmarker_ratio];
                                                            
                                                            r_rightcam_rightmarker = T_rightcam_rightmarker(1:3, 1:3);
                                                            q_rightcam_rightmarker = rotm2quat( r_rightcam_rightmarker );
                                                            t_rightcam_rightmarker = T_rightcam_rightmarker(1:3,4);
                                                            t_rightcam_rightmarker = transpose( t_rightcam_rightmarker );
                                                            q_t_rightcam_rightmarker = [q_rightcam_rightmarker t_rightcam_rightmarker];
                                                            Poseset_rightcam_rightmarker = [Poseset_rightcam_rightmarker; q_t_rightcam_rightmarker];
                                                            Areaset_rightcam_rightmarker = [Areaset_rightcam_rightmarker; projectedArea_rightmarker_ratio];
                                                        end
                                                        
                                                        if( good_pose_number>0 )
                                                            [set_size, columns] = size(Poseset_leftcam_leftmarker);
                                                            pose_difference_index_1 = 1;
                                                            pose_difference_index_2 = 1;
                                                            
                                                            rotm1_to_add = T_leftcam_leftmarker(1:3,1:3);
                                                            quat1_to_add = rotm2quat(rotm1_to_add);
                                                            trans1_to_add = T_leftcam_leftmarker(1:3,4);
                                                            
                                                            rotm2_to_add = T_rightcam_rightmarker(1:3,1:3);
                                                            quat2_to_add = rotm2quat(rotm2_to_add);
                                                            trans2_to_add = T_rightcam_rightmarker(1:3,4);
                                                            
                                                            for index = 1:set_size
                                                                quat1_to_compare = Poseset_leftcam_leftmarker(index, 1:4);
                                                                trans1_to_compare = Poseset_leftcam_leftmarker(index, 5:7);
                                                                trans1_to_compare = transpose(trans1_to_compare);
                                                                quaternion1_difference_value = compare_quaternion_difference( quat1_to_add, quat1_to_compare );
                                                                translation1_difference_value = compare_sum_of_3dim_difference( trans1_to_add, trans1_to_compare);
                                                                
                                                                if ( translation1_difference_value<minimum_translation_thred && quaternion1_difference_value<minimum_rotation_thred )
                                                                    pose_difference_index_1 = 0;
                                                                    break;
                                                                end
                                                                
                                                                quat2_to_compare = Poseset_rightcam_rightmarker(index, 1:4);
                                                                trans2_to_compare = Poseset_rightcam_rightmarker(index, 5:7);
                                                                trans2_to_compare = transpose(trans2_to_compare);
                                                                quaternion2_difference_value = compare_quaternion_difference( quat2_to_add, quat2_to_compare );
                                                                translation2_difference_value = compare_sum_of_3dim_difference( trans2_to_add, trans2_to_compare);
                                                                
                                                                if ( translation2_difference_value<minimum_translation_thred && quaternion2_difference_value<minimum_rotation_thred )
                                                                    pose_difference_index_2 = 0;
                                                                    break;
                                                                end  
                                                            end
                                                            
                                                            if ( pose_difference_index_1==1 && pose_difference_index_2==1 )
                                                                
                                                                good_pose_number = good_pose_number + 1;
                                                                
                                                                r_leftcam_leftmarker = T_leftcam_leftmarker(1:3, 1:3);
                                                                q_leftcam_leftmarker = rotm2quat( r_leftcam_leftmarker );
                                                                t_leftcam_leftmarker = T_leftcam_leftmarker(1:3,4);
                                                                t_leftcam_leftmarker = transpose( t_leftcam_leftmarker );
                                                                q_t_leftcam_leftmarker = [q_leftcam_leftmarker t_leftcam_leftmarker];
                                                                Poseset_leftcam_leftmarker = [Poseset_leftcam_leftmarker; q_t_leftcam_leftmarker];
                                                                Areaset_leftcam_leftmarker = [Areaset_leftcam_leftmarker; projectedArea_leftmarker_ratio];
                                                                
                                                                r_rightcam_rightmarker = T_rightcam_rightmarker(1:3, 1:3);
                                                                q_rightcam_rightmarker = rotm2quat( r_rightcam_rightmarker );
                                                                t_rightcam_rightmarker = T_rightcam_rightmarker(1:3,4);
                                                                t_rightcam_rightmarker = transpose( t_rightcam_rightmarker );
                                                                q_t_rightcam_rightmarker = [q_rightcam_rightmarker t_rightcam_rightmarker];
                                                                Poseset_rightcam_rightmarker = [Poseset_rightcam_rightmarker; q_t_rightcam_rightmarker];
                                                                Areaset_rightcam_rightmarker = [Areaset_rightcam_rightmarker; projectedArea_rightmarker_ratio]; 
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
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %%%%%%%%%%%%% Three Points Iteration P3P %%%%%%%%%%%
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    end
                end
            end
        end
    end
end

h = msgbox('Operation Completed.');