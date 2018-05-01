function [error_TX] = compare_sum_of_3dim_difference(X_true, X_est)

TX = X_true - X_est;
error_TX = abs(TX(1)) + abs(TX(2)) + abs(TX(3));
end