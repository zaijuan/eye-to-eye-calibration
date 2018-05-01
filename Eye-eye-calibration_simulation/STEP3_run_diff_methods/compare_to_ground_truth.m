function [quat_dif_arccosX, error_TX, quat_dif_arccosY, error_TY] = compare_to_ground_truth(X_true, Y_true, X_est, Y_est)
%%% This function compares the error between the estimated X, Y and the corresponding ground truth.

R_X_true = X_true(1:3, 1:3);
quat_X_true = rotm2quat(R_X_true);

R_X_est = X_est(1:3, 1:3);
quat_X_est = rotm2quat(R_X_est);

quat_dotproduct_X = quat_X_true(1,1) * quat_X_est(1,1) + quat_X_true(1,2) * quat_X_est(1,2) + ...
                    quat_X_true(1,3) * quat_X_est(1,3) + quat_X_true(1,4) * quat_X_est(1,4);
quat_dif_arccosX_init = acos( quat_dotproduct_X );
quat_dif_arccosX = min( quat_dif_arccosX_init, pi-quat_dif_arccosX_init);

TX = X_true(1:3,4) - X_est(1:3,4);
error_TX = sqrt( TX(1,1)*TX(1,1) + TX(2,1)*TX(2,1) + TX(3,1)*TX(3,1) );
       



R_Y_true = Y_true(1:3, 1:3);
quat_Y_true = rotm2quat(R_Y_true);

R_Y_est = Y_est(1:3, 1:3);
quat_Y_est = rotm2quat(R_Y_est);

quat_dotproduct_Y = quat_Y_true(1,1) * quat_Y_est(1,1) + quat_Y_true(1,2) * quat_Y_est(1,2) + ...
                    quat_Y_true(1,3) * quat_Y_est(1,3) + quat_Y_true(1,4) * quat_Y_est(1,4);
quat_dif_arccosY_init = acos( quat_dotproduct_Y );
quat_dif_arccosY = min( quat_dif_arccosY_init, pi - quat_dif_arccosY_init);

TY = Y_true(1:3,4) - Y_est(1:3,4);
error_TY = sqrt( TY(1,1)*TY(1,1) + TY(2,1)*TY(2,1) + TY(3,1)*TY(3,1) );



%%%% The following is not used in this function, it was put here just as a back-up.%%%%%%             
RX = X_true(1:3,1:3) - X_est(1:3,1:3);
error_RX = sqrt( RX(1,1)*RX(1,1) + RX(1,2)*RX(1,2) + RX(1,3)*RX(1,3) + ...
                 RX(2,1)*RX(2,1) + RX(2,2)*RX(2,2) + RX(2,3)*RX(2,3) + ...
                 RX(3,1)*RX(3,1) + RX(3,2)*RX(3,2) + RX(3,3)*RX(3,3) );
             
RY = Y_true(1:3,1:3) - Y_est(1:3,1:3);
error_RY = sqrt( RY(1,1)*RY(1,1) + RY(1,2)*RY(1,2) + RY(1,3)*RY(1,3) + ...
                 RY(2,1)*RY(2,1) + RY(2,2)*RY(2,2) + RY(2,3)*RY(2,3) + ...
                 RY(3,1)*RY(3,1) + RY(3,2)*RY(3,2) + RY(3,3)*RY(3,3) );