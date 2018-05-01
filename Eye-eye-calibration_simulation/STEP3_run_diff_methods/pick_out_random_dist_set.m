function [AA, BB] = pick_out_random_dist_set( Big_projection_size_set, Small_projection_size_set, setnumber)

to_minus = mod(setnumber,2);
if(to_minus==0)
    onehalf = setnumber/2;
    otherhalf  = onehalf;
else
    onehalf = (setnumber+1)/2;
    otherhalf = setnumber - onehalf;
end

Final_set = [];
Chosen_Id_set = [];

[bigrows, columns] = size(Big_projection_size_set);
bank_size = bigrows/4;
for i=1:onehalf
    if i==1
        random_seed = randi(bank_size);
        Final_set = [Final_set; Big_projection_size_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
    if i>1
        [currentrows, columns] = size( Final_set );
        currentrows = currentrows/4;
        need_to_regenerate = 1;
        while( need_to_regenerate )
            random_seed = randi(bank_size);
            need_to_regenerate = 0;
            for j=1:currentrows
                if( random_seed==Chosen_Id_set(j) )
                    need_to_regenerate = 1;
                end
            end
        end
        Final_set = [ Final_set; Big_projection_size_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
end

Chosen_Id_set = [];
[bigrows, columns] = size(Small_projection_size_set);
bank_size = bigrows/4;
for i=1:otherhalf
    if i==1
        random_seed = randi(bank_size);
        Final_set = [Final_set; Small_projection_size_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
    if i>1
        [currentrows, columns] = size( Final_set );
        currentrows = currentrows/4 - onehalf;
        need_to_regenerate = 1;
        while( need_to_regenerate )
            random_seed = randi(bank_size);
            need_to_regenerate = 0;
            for j=1:currentrows
                if( random_seed==Chosen_Id_set(j) )
                    need_to_regenerate = 1;
                end
            end
        end
        Final_set = [ Final_set; Small_projection_size_set( (4*random_seed-3):4*random_seed, :) ];
        Chosen_Id_set = [Chosen_Id_set; random_seed];
    end
end 

AA=[];
BB=[];
for i=1:setnumber
    AA = [AA Final_set( (4*i-3):4*i, 1:4)];
    BB = [BB Final_set( (4*i-3):4*i, 5:8)];
end
    
end
