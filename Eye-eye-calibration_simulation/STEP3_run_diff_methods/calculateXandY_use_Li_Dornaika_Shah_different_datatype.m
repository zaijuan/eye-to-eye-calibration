function [ ERROR_ALL_RX_Li, ERROR_ALL_TX_Li, ERROR_ALL_RY_Li, ERROR_ALL_TY_Li,...
           ERROR_ALL_RX_Dornaika, ERROR_ALL_TX_Dornaika, ERROR_ALL_RY_Dornaika, ERROR_ALL_TY_Dornaika,...
           ERROR_ALL_RX_Shah, ERROR_ALL_TX_Shah, ERROR_ALL_RY_Shah, ERROR_ALL_TY_Shah] = calculateXandY_use_Li_Dornaika_Shah_different_datatype(  )

T_leftcam_rightcam_true = [   0.5,                 0,     0.866025403784439,   1.03923048454133;
                              0,                   1,     0,                   0;
                             -0.866025403784439,   0,     0.500000000000000,  -0.6;
                              0,                   0,     0,                   1];
                            
T_leftmarker_rightmarker_true = [0.499999999999288,    0,       -0.866025403784850,   1.58049636190572;
                                 0,                    1,        0,                   0;
                                 0.866025403784851,    0,        0.499999999999287,   0.912500000001394;
                                 0,                    0,        0,                   1];
                             
X_true = T_leftmarker_rightmarker_true;
Y_true = T_leftcam_rightcam_true;

ERROR_ALL_RX_Li = zeros(100, 4);
ERROR_ALL_TX_Li = zeros(100, 4);
ERROR_ALL_RY_Li = zeros(100, 4);
ERROR_ALL_TY_Li = zeros(100, 4);
ERROR_ALL_RX_Dornaika = zeros(100, 4);
ERROR_ALL_TX_Dornaika = zeros(100, 4);
ERROR_ALL_RY_Dornaika = zeros(100, 4);
ERROR_ALL_TY_Dornaika = zeros(100, 4);
ERROR_ALL_RX_Shah = zeros(100, 4);
ERROR_ALL_TX_Shah = zeros(100, 4);
ERROR_ALL_RY_Shah = zeros(100, 4);
ERROR_ALL_TY_Shah = zeros(100, 4);

load data_bank/Ai_Bi_Bigsize_Level4.txt;
load data_bank/Ai_Bi_Smallsize_Level4.txt;

for iteration_run = 1:1:100   
        measurement_number_column = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [AA, BB] = pick_out_scattered_dist_set( Ai_Bi_Bigsize_Level5, 50);
        measurement_number_column = measurement_number_column +1;

        [X_li1995, Y_li1995] = li1995(AA, BB);
        [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
        [X_shah2013, Y_shah2013] = shah2013(AA, BB);
        
        [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
        [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
        [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
        
        ERROR_ALL_RX_Li(iteration_run,measurement_number_column) = error_RX_li1995;
        ERROR_ALL_TX_Li(iteration_run,measurement_number_column) = error_TX_li1995;
        ERROR_ALL_RY_Li(iteration_run,measurement_number_column) = error_RY_li1995;
        ERROR_ALL_TY_Li(iteration_run,measurement_number_column) = error_TY_li1995;
        ERROR_ALL_RX_Dornaika(iteration_run,measurement_number_column) = error_RX_dornaika1998;
        ERROR_ALL_TX_Dornaika(iteration_run,measurement_number_column) = error_TX_dornaika1998;
        ERROR_ALL_RY_Dornaika(iteration_run,measurement_number_column) = error_RY_dornaika1998;
        ERROR_ALL_TY_Dornaika(iteration_run,measurement_number_column) = error_TY_dornaika1998;
        ERROR_ALL_RX_Shah(iteration_run,measurement_number_column) = error_RX_shah2013;
        ERROR_ALL_TX_Shah(iteration_run,measurement_number_column) = error_TX_shah2013;
        ERROR_ALL_RY_Shah(iteration_run,measurement_number_column) = error_RY_shah2013;
        ERROR_ALL_TY_Shah(iteration_run,measurement_number_column) = error_TY_shah2013;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [AA, BB] = pick_out_clustered_dist_set( Ai_Bi_Bigsize_Level4, 50);
        measurement_number_column = measurement_number_column +1;

        [X_li1995, Y_li1995] = li1995(AA, BB);
        [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
        [X_shah2013, Y_shah2013] = shah2013(AA, BB);
        
        [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
        [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
        [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
        
        ERROR_ALL_RX_Li(iteration_run,measurement_number_column) = error_RX_li1995;
        ERROR_ALL_TX_Li(iteration_run,measurement_number_column) = error_TX_li1995;
        ERROR_ALL_RY_Li(iteration_run,measurement_number_column) = error_RY_li1995;
        ERROR_ALL_TY_Li(iteration_run,measurement_number_column) = error_TY_li1995;
        ERROR_ALL_RX_Dornaika(iteration_run,measurement_number_column) = error_RX_dornaika1998;
        ERROR_ALL_TX_Dornaika(iteration_run,measurement_number_column) = error_TX_dornaika1998;
        ERROR_ALL_RY_Dornaika(iteration_run,measurement_number_column) = error_RY_dornaika1998;
        ERROR_ALL_TY_Dornaika(iteration_run,measurement_number_column) = error_TY_dornaika1998;
        ERROR_ALL_RX_Shah(iteration_run,measurement_number_column) = error_RX_shah2013;
        ERROR_ALL_TX_Shah(iteration_run,measurement_number_column) = error_TX_shah2013;
        ERROR_ALL_RY_Shah(iteration_run,measurement_number_column) = error_RY_shah2013;
        ERROR_ALL_TY_Shah(iteration_run,measurement_number_column) = error_TY_shah2013;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [AA, BB] = pick_out_scattered_dist_set( Ai_Bi_Smallsize_Level4, 50);
        measurement_number_column = measurement_number_column +1;

        [X_li1995, Y_li1995] = li1995(AA, BB);
        [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
        [X_shah2013, Y_shah2013] = shah2013(AA, BB);
        
        [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
        [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
        [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
        
        ERROR_ALL_RX_Li(iteration_run,measurement_number_column) = error_RX_li1995;
        ERROR_ALL_TX_Li(iteration_run,measurement_number_column) = error_TX_li1995;
        ERROR_ALL_RY_Li(iteration_run,measurement_number_column) = error_RY_li1995;
        ERROR_ALL_TY_Li(iteration_run,measurement_number_column) = error_TY_li1995;
        ERROR_ALL_RX_Dornaika(iteration_run,measurement_number_column) = error_RX_dornaika1998;
        ERROR_ALL_TX_Dornaika(iteration_run,measurement_number_column) = error_TX_dornaika1998;
        ERROR_ALL_RY_Dornaika(iteration_run,measurement_number_column) = error_RY_dornaika1998;
        ERROR_ALL_TY_Dornaika(iteration_run,measurement_number_column) = error_TY_dornaika1998;
        ERROR_ALL_RX_Shah(iteration_run,measurement_number_column) = error_RX_shah2013;
        ERROR_ALL_TX_Shah(iteration_run,measurement_number_column) = error_TX_shah2013;
        ERROR_ALL_RY_Shah(iteration_run,measurement_number_column) = error_RY_shah2013;
        ERROR_ALL_TY_Shah(iteration_run,measurement_number_column) = error_TY_shah2013;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [AA, BB] = pick_out_clustered_dist_set( Ai_Bi_Smallsize_Level4, 50);
        measurement_number_column = measurement_number_column +1;

        [X_li1995, Y_li1995] = li1995(AA, BB);
        [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
        [X_shah2013, Y_shah2013] = shah2013(AA, BB);
        
        [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
        [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
        [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
        
        ERROR_ALL_RX_Li(iteration_run,measurement_number_column) = error_RX_li1995;
        ERROR_ALL_TX_Li(iteration_run,measurement_number_column) = error_TX_li1995;
        ERROR_ALL_RY_Li(iteration_run,measurement_number_column) = error_RY_li1995;
        ERROR_ALL_TY_Li(iteration_run,measurement_number_column) = error_TY_li1995;
        ERROR_ALL_RX_Dornaika(iteration_run,measurement_number_column) = error_RX_dornaika1998;
        ERROR_ALL_TX_Dornaika(iteration_run,measurement_number_column) = error_TX_dornaika1998;
        ERROR_ALL_RY_Dornaika(iteration_run,measurement_number_column) = error_RY_dornaika1998;
        ERROR_ALL_TY_Dornaika(iteration_run,measurement_number_column) = error_TY_dornaika1998;
        ERROR_ALL_RX_Shah(iteration_run,measurement_number_column) = error_RX_shah2013;
        ERROR_ALL_TX_Shah(iteration_run,measurement_number_column) = error_TX_shah2013;
        ERROR_ALL_RY_Shah(iteration_run,measurement_number_column) = error_RY_shah2013;
        ERROR_ALL_TY_Shah(iteration_run,measurement_number_column) = error_TY_shah2013;
    
end

end
