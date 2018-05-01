function [error_TX] = compare_translation_difference(X_true, X_est)

TX = X_true - X_est;
error_TX = sqrt( TX(1)*TX(1) + TX(2)*TX(2) + TX(3)*TX(3) );

end