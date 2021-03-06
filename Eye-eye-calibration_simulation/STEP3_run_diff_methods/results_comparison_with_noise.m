clc;    % Clear the command window. 

load calibration_results/Error_All_BiEdge1_noise.txt;
Error_All_RX_BiEdge1 = Error_All_BiEdge1_noise(1:100, :);
Error_All_TX_BiEdge1 = Error_All_BiEdge1_noise(101:200, :);
Error_All_RY_BiEdge1 = Error_All_BiEdge1_noise(201:300, :);
Error_All_TY_BiEdge1 = Error_All_BiEdge1_noise(301:400, :);
Error_Avrg_RX_BiEdge1 = sum(Error_All_RX_BiEdge1)./100;                  
Error_Avrg_RX_BiEdge1 = Error_Avrg_RX_BiEdge1*180/pi;
Error_Avrg_TX_BiEdge1 = sum(Error_All_TX_BiEdge1)./100;
Error_Avrg_RY_BiEdge1 = sum(Error_All_RY_BiEdge1)./100;                  
Error_Avrg_RY_BiEdge1 = Error_Avrg_RY_BiEdge1*180/pi;
Error_Avrg_TY_BiEdge1 = sum(Error_All_TY_BiEdge1)./100;

load calibration_results/Error_All_BiEdge2_noise.txt;
Error_All_RX_BiEdge2 = Error_All_BiEdge2_noise(1:100, :);
Error_All_TX_BiEdge2 = Error_All_BiEdge2_noise(101:200, :);
Error_All_RY_BiEdge2 = Error_All_BiEdge2_noise(201:300, :);
Error_All_TY_BiEdge2 = Error_All_BiEdge2_noise(301:400, :);
Error_Avrg_RX_BiEdge2 = sum(Error_All_RX_BiEdge2)./100;                
Error_Avrg_RX_BiEdge2 = Error_Avrg_RX_BiEdge2*180/pi;
Error_Avrg_TX_BiEdge2 = sum(Error_All_TX_BiEdge2)./100;
Error_Avrg_RY_BiEdge2 = sum(Error_All_RY_BiEdge2)./100;                
Error_Avrg_RY_BiEdge2 = Error_Avrg_RY_BiEdge2*180/pi;
Error_Avrg_TY_BiEdge2 = sum(Error_All_TY_BiEdge2)./100;

load calibration_results/Error_All_RightEdge1_noise.txt;
Error_All_RX_RightEdge1 = Error_All_RightEdge1_noise(1:100, :);
Error_All_TX_RightEdge1 = Error_All_RightEdge1_noise(101:200, :);
Error_All_RY_RightEdge1 = Error_All_RightEdge1_noise(201:300, :);
Error_All_TY_RightEdge1 = Error_All_RightEdge1_noise(301:400, :);
Error_Avrg_RX_RightEdge1 = sum(Error_All_RX_RightEdge1)./100;          
Error_Avrg_RX_RightEdge1 = Error_Avrg_RX_RightEdge1*180/pi;
Error_Avrg_TX_RightEdge1 = sum(Error_All_TX_RightEdge1)./100;
Error_Avrg_RY_RightEdge1 = sum(Error_All_RY_RightEdge1)./100;          
Error_Avrg_RY_RightEdge1 = Error_Avrg_RY_RightEdge1*180/pi;
Error_Avrg_TY_RightEdge1 = sum(Error_All_TY_RightEdge1)./100;


load calibration_results/Error_All_RightEdge2_noise.txt;
Error_All_RX_RightEdge2 = Error_All_RightEdge2_noise(1:100, :);
Error_All_TX_RightEdge2 = Error_All_RightEdge2_noise(101:200, :);
Error_All_RY_RightEdge2 = Error_All_RightEdge2_noise(201:300, :);
Error_All_TY_RightEdge2 = Error_All_RightEdge2_noise(301:400, :);
Error_Avrg_RX_RightEdge2 = sum(Error_All_RX_RightEdge2)./100;        
Error_Avrg_RX_RightEdge2 = Error_Avrg_RX_RightEdge2*180/pi;
Error_Avrg_TX_RightEdge2 = sum(Error_All_TX_RightEdge2)./100;
Error_Avrg_RY_RightEdge2 = sum(Error_All_RY_RightEdge2)./100;        
Error_Avrg_RY_RightEdge2 = Error_Avrg_RY_RightEdge2*180/pi;
Error_Avrg_TY_RightEdge2 = sum(Error_All_TY_RightEdge2)./100;


load calibration_results/Error_All_Wang_noise.txt;
Error_All_RX_Wang = Error_All_Wang_noise(1:100, :);        
Error_All_TX_Wang = Error_All_Wang_noise(101:200, :);
Error_All_RY_Wang = Error_All_Wang_noise(201:300, :);      
Error_All_TY_Wang = Error_All_Wang_noise(301:400, :);
Error_Avrg_RX_Wang = sum(Error_All_RX_Wang)./100;                   
Error_Avrg_RX_Wang = Error_Avrg_RX_Wang*180/pi;
Error_Avrg_TX_Wang = sum(Error_All_TX_Wang)./100;
Error_Avrg_RY_Wang = sum(Error_All_RY_Wang)./100;                   
Error_Avrg_RY_Wang = Error_Avrg_RY_Wang*180/pi;
Error_Avrg_TY_Wang = sum(Error_All_TY_Wang)./100;

Error_Avrg_Wang = [Error_Avrg_RX_Wang; Error_Avrg_TX_Wang; Error_Avrg_RY_Wang; Error_Avrg_TY_Wang];
Error_Avrg_BiEdge1 = [Error_Avrg_RX_BiEdge1; Error_Avrg_TX_BiEdge1; Error_Avrg_RY_BiEdge1; Error_Avrg_TY_BiEdge1];
Error_Avrg_BiEdge2 = [Error_Avrg_RX_BiEdge2; Error_Avrg_TX_BiEdge2; Error_Avrg_RY_BiEdge2; Error_Avrg_TY_BiEdge2];
Error_Avrg_RightEdge1 = [Error_Avrg_RX_RightEdge1; Error_Avrg_TX_RightEdge1; Error_Avrg_RY_RightEdge1; Error_Avrg_TY_RightEdge1];
Error_Avrg_RightEdge2 = [Error_Avrg_RX_RightEdge2; Error_Avrg_TX_RightEdge2; Error_Avrg_RY_RightEdge2; Error_Avrg_TY_RightEdge2];
 
Error_Avrg_Wang = Error_Avrg_Wang(:, 1:8);
Error_Avrg_BiEdge1 = Error_Avrg_BiEdge1(:, 1:8);
Error_Avrg_BiEdge2 = Error_Avrg_BiEdge2(:, 1:8);
Error_Avrg_RightEdge1 = Error_Avrg_RightEdge1(:, 1:8);
Error_Avrg_RightEdge2 = Error_Avrg_RightEdge2(:, 1:8);

Error_Avrg_MonoEdge1 = Error_Avrg_RightEdge1;
Error_Avrg_MonoEdge2 = Error_Avrg_RightEdge2; 

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%    Plot the X's rotation error of all methods   %%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 figure(1);
 X = [0 0.2 0.4 0.6 0.8 1.0 1.2 1.4];
  plot(X, Error_Avrg_Wang(1,:), ':*k', 'LineWidth', 1, 'MarkerSize',10); hold on;
  plot(X, Error_Avrg_MonoEdge1(1,:), '-or', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_MonoEdge2(1,:), '-ob', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge1(1,:), '-sr', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge2(1,:), '-sb', 'LineWidth', 1, 'MarkerSize',12); hold on;
   
 legend( 'Dornaika', 'Shah', 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu', 'Location', 'NorthEastOutside' );
 legend(  'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu', 'Location', 'NorthEastOutside' );
 title('X Rotation Error');
 xlabel('Measurement Noise(in pixel)');
 ylabel({'Average Rotation Error', '(in degrees)'});
 
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  %%%%%%%%%%%%%   Plot the X's Translation error of all methods    %%%%%%%%%%%%%%%
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 figure(2);
 X = [0 0.2 0.4 0.6 0.8 1.0 1.2 1.4];
  plot(X, Error_Avrg_Wang(2,:), ':*k', 'LineWidth', 1, 'MarkerSize',10); hold on;
  plot(X, Error_Avrg_MonoEdge1(2,:), '-or', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_MonoEdge2(2,:), '-ob', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge1(2,:), '-sr', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge2(2,:), '-sb', 'LineWidth', 1, 'MarkerSize',12); hold on;

 legend(  'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu', 'Location', 'NorthEastOutside' );
 title('X Translation Error');
 xlabel('Measurement Noise(in pixel)');
 ylabel({'Average Error', '(in meters)'}); 
  
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%      Plot the Y's Rotation error of all methods     %%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 figure(3);
 X = [0 0.2 0.4 0.6 0.8 1.0 1.2 1.4];
  plot(X, Error_Avrg_Wang(3,:),':*k', 'LineWidth', 1, 'MarkerSize',10); hold on;
  plot(X, Error_Avrg_MonoEdge1(3,:),'-or', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_MonoEdge2(3,:),'-ob', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge1(3,:),'-sr', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge2(3,:),'-sb', 'LineWidth', 1, 'MarkerSize',12); hold on;

 legend(  'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu', 'Location', 'NorthEastOutside' );
 title('Y Rotation Error');
 xlabel('Measurement Noise(in pixel)');
 ylabel({'Average Rotation Error', '(in degrees)'});
  
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%    Plot the Y's Translation error of all methods   %%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 figure(4);
 X = [0 0.2 0.4 0.6 0.8 1.0 1.2 1.4];
  plot(X, Error_Avrg_Wang(4,:),':*k', 'LineWidth',1, 'MarkerSize',10); hold on;
  plot(X, Error_Avrg_MonoEdge1(4,:),'-or', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_MonoEdge2(4,:),'-ob', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge1(4,:),'-sr', 'LineWidth', 1, 'MarkerSize',12); hold on;
  plot(X, Error_Avrg_BiEdge2(4,:),'-sb', 'LineWidth', 1, 'MarkerSize',12); hold on;
  
 legend(  'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu', 'Location', 'NorthEastOutside' );
 title('Y Translation Error');
 xlabel('Measurement Noise(in pixel)');
 ylabel({'Average Error', '(in meters)'});
