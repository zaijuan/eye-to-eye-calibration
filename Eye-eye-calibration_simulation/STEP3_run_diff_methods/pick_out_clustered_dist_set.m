function [AA, BB] = pick_out_clustered_dist_set( Initial_set, set_number)
[bigrows, columns] = size(Initial_set);
bank_size = bigrows/4;
Final_set = [];
Chosen_Id_set = [];


for i=1:set_number
    
    if i==1
        random_seed = randi(bank_size);
        Final_set = [Final_set; Initial_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
    
    if i>1
        [currentrows, currentcolumns] = size( Final_set );
        current_size = currentrows/4;
        translation_dist_nearest = 1.0;
        rotation_smallest = 2.0;
        
        for j=1:bank_size
            already_in_set = 0;
            for k=1:current_size
                if( j==Chosen_Id_set(k) )
                    already_in_set = 1;
                    break;
                end
            end
            
            if( already_in_set )
                continue;
            else
                T_leftcam_leftmarker_to_add = Initial_set( (4*j-3):4*j,1:4);
                quat_leftcam_leftmarker_to_add = rotm2quat(T_leftcam_leftmarker_to_add(1:3, 1:3));
                trans_leftcam_leftmarker_to_add = T_leftcam_leftmarker_to_add(1:3, 4);
                for k=1:current_size
                    T_leftcam_leftmarker_to_compare = Final_set( (4*k-3):4*k,1:4);
                    quat_leftcam_leftmarker_to_compare = rotm2quat(T_leftcam_leftmarker_to_compare(1:3, 1:3));
                    trans_leftcam_leftmarker_to_compare = T_leftcam_leftmarker_to_compare(1:3, 4);
                    
                    translation_diff = compare_sum_of_3dim_difference( trans_leftcam_leftmarker_to_add, trans_leftcam_leftmarker_to_compare );
                    rotation_diff = compare_quaternion_difference( quat_leftcam_leftmarker_to_add, quat_leftcam_leftmarker_to_compare );
                    
                    if( translation_diff<=translation_dist_nearest && rotation_diff<=rotation_smallest )
                        translation_dist_nearest = translation_diff;
                        rotation_smallest = rotation_diff;
                        Chosen_Id = j;
                    end
                end
            end
        end
        Final_set = [Final_set; Initial_set( (4*Chosen_Id-3):4*Chosen_Id, :)];
        Chosen_Id_set = [Chosen_Id_set; Chosen_Id];
    end
end

AA=[];
BB=[];
for i=1:set_number
    AA = [AA Final_set( (4*i-3):4*i, 1:4)];
    BB = [BB Final_set( (4*i-3):4*i, 5:8)];
end

end
