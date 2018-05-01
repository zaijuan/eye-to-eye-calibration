%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%     The initial data used in this program are generated based on the following    %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


 load true_pose_bank/AreaPoseSet_leftcam_leftmarker.mat;
 load true_pose_bank/AreaPoseSet_rightcam_rightmarker.mat;
 Areas_Q_Ts_set = [];
 [bigrows, bigcolumns] = size( AreaPoseSet_leftcam_leftmarker );
 to_minus = mod( bigrows,2);
one_half_bigrows = (bigrows-to_minus)/2;

 for m = 1:bigrows
     Areas_Q_Ts_set = [Areas_Q_Ts_set; AreaPoseSet_leftcam_leftmarker(m,1) AreaPoseSet_rightcam_rightmarker(m,1) AreaPoseSet_leftcam_leftmarker(m, 2:8) AreaPoseSet_rightcam_rightmarker(m, 2:8)];
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Sort the area of Rightside Marker (ASCEND ORDER)%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Area_RightsideMarker_ascend = sortrows(Areas_Q_Ts_set, 2, 'ascend');
[rows, columns] = size(Areas_Q_Ts_set);
to_minus = mod( rows,4);
third_fourth_rows = ( (rows-to_minus)/4 )*3;
Area_RightsideMarker_first_sorted = Area_RightsideMarker_ascend(1:third_fourth_rows, :);  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Sort the area of Leftside Marker based on Rightside Marker (ASCEND ORDER)%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Area_LeftsideMarker_ascend = sortrows(Area_RightsideMarker_first_sorted, 1, 'ascend');
%Small_areas_sorted_set = Area_LeftsideMarker_ascend(1:one_half_bigrows, :);
Small_areas_sorted_set = Area_LeftsideMarker_ascend(2001:8000, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Sort the area of Leftside Marker (DESCEND ORDER)%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Area_LeftsideMarker_descend = sortrows(Areas_Q_Ts_set, 1, 'descend');
[rows, columns] = size(Areas_Q_Ts_set);
to_minus = mod( rows,4);
third_fourth_rows = ( (rows-to_minus)/4 )*3;
Area_LeftsideMarker_first_sorted = Area_LeftsideMarker_descend(1:third_fourth_rows, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Sort the area of Leftside Marker based on Rightside Marker (DESCEND ORDER)%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Area_RightsideMarker_second_sorted = sortrows(Area_LeftsideMarker_first_sorted, 2, 'descend');
%Big_areas_sorted_set = Area_RightsideMarker_second_sorted(1:one_half_bigrows, :);
Big_areas_sorted_set = Area_RightsideMarker_second_sorted(1:6000, :);

h = msgbox('Operation Completed.');