
function [quat_difference] = compare_quaternion_difference(quat_X_true, quat_X_est)

quat_dotproduct = quat_X_true(1) * quat_X_est(1) + quat_X_true(2) * quat_X_est(2) + ...
                    quat_X_true(3) * quat_X_est(3) + quat_X_true(4) * quat_X_est(4);
quat_dif_arccosX_init = acos( quat_dotproduct );
quat_difference = min( quat_dif_arccosX_init, pi-quat_dif_arccosX_init);

end