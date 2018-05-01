function [AA, BB] = pick_out_scattered_dist_set( Initial_set, set_number)

[bigrows, columns] = size(Initial_set);
bank_size = bigrows/4;

minimum_translation_thred = 0.02;
% min_rotconditioned_translation = 0.09;
% max_rotconditioned_translation = 0.2;

minimum_rotation_thred = 0.04;
% min_transconditioned_rot = 0.4;
% max_transconditioned_rot = 1.0;

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
        currentsize = currentrows/4;
        need_to_regenerate = 1;
        while( need_to_regenerate )
            random_seed = randi(bank_size);
            need_to_regenerate = 0;
            for j=1:currentsize
                if( random_seed==Chosen_Id_set(j) )
                    need_to_regenerate = 1;
                end
            end
            
            T_leftcam_leftmarker_to_add = Initial_set( (4*random_seed-3):4*random_seed,1:4);
            quat_leftcam_leftmarker_to_add = rotm2quat(T_leftcam_leftmarker_to_add(1:3, 1:3));
            trans_leftcam_leftmarker_to_add = T_leftcam_leftmarker_to_add(1:3, 4);
            
            for j=1:currentsize
                T_leftcam_leftmarker_to_compare = Final_set( (4*j-3):4*j,1:4);
                quat_leftcam_leftmarker_to_compare = rotm2quat(T_leftcam_leftmarker_to_compare(1:3, 1:3));
                trans_leftcam_leftmarker_to_compare = T_leftcam_leftmarker_to_compare(1:3, 4);
                
                trans_difference = compare_translation_difference( trans_leftcam_leftmarker_to_add, trans_leftcam_leftmarker_to_compare );
                quat_difference = compare_quaternion_difference( quat_leftcam_leftmarker_to_add, quat_leftcam_leftmarker_to_compare );
                
                if( trans_difference<minimum_translation_thred && quat_difference<minimum_rotation_thred )
                    need_to_regenerate = 1;
                    break;
                end 
%                 if( trans_difference<min_rotconditioned_translation && quat_difference<max_transconditioned_rot && quat_difference>minimum_rotation_thred )
%                     need_to_regenerate = 1;
%                     break;
%                 end
%                 if( quat_difference<min_transconditioned_rot && trans_difference<max_rotconditioned_translation && trans_difference>minimum_translation_thred )
%                     need_to_regenerate = 1;
%                     break;
%                 end
            end
        end
        Final_set = [ Final_set; Initial_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
end

AA=[];
BB=[];
for i=1:set_number
    AA = [AA Final_set( (4*i-3):4*i, 1:4)];
    BB = [BB Final_set( (4*i-3):4*i, 5:8)];
end

end
