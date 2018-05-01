function [Error_All_RX_Li, Error_All_TX_Li, Error_All_RY_Li, Error_All_TY_Li,...
          Error_All_RX_Dornaika, Error_All_TX_Dornaika, Error_All_RY_Dornaika, Error_All_TY_Dornaika,...
          Error_All_RX_Shah, Error_All_TX_Shah, Error_All_RY_Shah, Error_All_TY_Shah] = calculateXandY_use_Li_Dornaika_Shah_different_noise_level()
      
T_leftcam_rightcam_true = [   0.5,                 0,    0.866025403784439,   1.03923048454133;
                              0,                   1,    0,                   0;
                             -0.866025403784439,   0,    0.500000000000000,  -0.6;
                              0,                   0,    0,                   1];
                            
T_leftmarker_rightmarker_true = [0.499999999999288,    0,       -0.866025403784850,   1.58049636190572;
                                 0,                    1,        0,                   0;
                                 0.866025403784851,    0,        0.499999999999287,   0.912500000001394;
                                 0,                    0,        0,                   1];
                             
X_true = T_leftmarker_rightmarker_true;
Y_true = T_leftcam_rightcam_true;

 Error_All_RX_Li = zeros(100, 8);
 Error_All_TX_Li = zeros(100, 8);
 Error_All_RY_Li = zeros(100, 8);
 Error_All_TY_Li = zeros(100, 8);
 Error_All_RX_Dornaika = zeros(100, 8);
 Error_All_TX_Dornaika = zeros(100, 8);
 Error_All_RY_Dornaika = zeros(100, 8);
 Error_All_TY_Dornaika = zeros(100, 8);
 Error_All_RX_Shah = zeros(100, 8);
 Error_All_TX_Shah = zeros(100, 8);
 Error_All_RY_Shah = zeros(100, 8);
 Error_All_TY_Shah = zeros(100, 8);
 measurement_noise_column = 0;
 
load data_bank/Ai_Bi_Bigsize_Level0.txt;
load data_bank/Ai_Bi_Smallsize_Level0.txt;
measurement_noise_column = measurement_noise_column +1;

for iteration_run = 1:1:100
     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level0, Ai_Bi_Smallsize_Level0, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end


load data_bank/Ai_Bi_Bigsize_Level1.txt;
load data_bank/Ai_Bi_Smallsize_Level1.txt;
 measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level1, Ai_Bi_Smallsize_Level1, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level2.txt;
load data_bank/Ai_Bi_Smallsize_Level2.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level2, Ai_Bi_Smallsize_Level2, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level3.txt;
load data_bank/Ai_Bi_Smallsize_Level3.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level3, Ai_Bi_Smallsize_Level3, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level4.txt;
load data_bank/Ai_Bi_Smallsize_Level4.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level4, Ai_Bi_Smallsize_Level4, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level5.txt;
load data_bank/Ai_Bi_Smallsize_Level5.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level5, Ai_Bi_Smallsize_Level5, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level6.txt;
load data_bank/Ai_Bi_Smallsize_Level6.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level6, Ai_Bi_Smallsize_Level6, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

load data_bank/Ai_Bi_Bigsize_Level7.txt;
load data_bank/Ai_Bi_Smallsize_Level7.txt;
measurement_noise_column = measurement_noise_column +1; 
for iteration_run = 1:1:100     
     [AA, BB] = pick_out_random_dist_set( Ai_Bi_Bigsize_Level7, Ai_Bi_Smallsize_Level7, 60);
     
     [X_li1995, Y_li1995] = li1995(AA,BB);
     [X_dornaika1998, Y_dornaika1998] = dornaika1998(AA, BB);
     [X_shah2013, Y_shah2013] = shah2013(AA, BB);
     
     [error_RX_li1995, error_TX_li1995, error_RY_li1995, error_TY_li1995] = compare_to_ground_truth(X_true, Y_true, X_li1995, Y_li1995);
     [error_RX_dornaika1998, error_TX_dornaika1998, error_RY_dornaika1998, error_TY_dornaika1998] = compare_to_ground_truth(X_true, Y_true, X_dornaika1998, Y_dornaika1998);
     [error_RX_shah2013, error_TX_shah2013, error_RY_shah2013, error_TY_shah2013] = compare_to_ground_truth(X_true, Y_true, X_shah2013, Y_shah2013);
  
     Error_All_RX_Li(iteration_run,measurement_noise_column) = error_RX_li1995;
     Error_All_TX_Li(iteration_run,measurement_noise_column) = error_TX_li1995;
     Error_All_RY_Li(iteration_run,measurement_noise_column) = error_RY_li1995;
     Error_All_TY_Li(iteration_run,measurement_noise_column) = error_TY_li1995;
     Error_All_RX_Dornaika(iteration_run,measurement_noise_column) = error_RX_dornaika1998;
     Error_All_TX_Dornaika(iteration_run,measurement_noise_column) = error_TX_dornaika1998;
     Error_All_RY_Dornaika(iteration_run,measurement_noise_column) = error_RY_dornaika1998;
     Error_All_TY_Dornaika(iteration_run,measurement_noise_column) = error_TY_dornaika1998;
     Error_All_RX_Shah(iteration_run,measurement_noise_column) = error_RX_shah2013;
     Error_All_TX_Shah(iteration_run,measurement_noise_column) = error_TX_shah2013;
     Error_All_RY_Shah(iteration_run,measurement_noise_column) = error_RY_shah2013;
     Error_All_TY_Shah(iteration_run,measurement_noise_column) = error_TY_shah2013;
end

end