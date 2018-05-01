Error_Avrg_Wang = zeros(4,4);
Error_Avrg_BiEdge1 = zeros(4,4);
Error_Avrg_BiEdge2 = zeros(4,4);
Error_Avrg_RightEdge1 = zeros(4,4);
Error_Avrg_RightEdge2 = zeros(4,4);

load calibration_results/Error_All_BiEdge1_datatype.txt;
Error_All_RX_BiEdge1 = Error_All_BiEdge1_datatype(1:100, :);
Error_All_TX_BiEdge1 = Error_All_BiEdge1_datatype(101:200, :);
Error_All_RY_BiEdge1 = Error_All_BiEdge1_datatype(201:300, :);
Error_All_TY_BiEdge1 = Error_All_BiEdge1_datatype(301:400, :);
Error_Avrg_RX_BiEdge1 = sum(Error_All_RX_BiEdge1)./100;  
Error_Avrg_RX_BiEdge1 = Error_Avrg_RX_BiEdge1*180/pi;
Error_Avrg_TX_BiEdge1 = sum(Error_All_TX_BiEdge1)./100;
Error_Avrg_RY_BiEdge1 = sum(Error_All_RY_BiEdge1)./100;  
Error_Avrg_RY_BiEdge1 = Error_Avrg_RY_BiEdge1*180/pi;
Error_Avrg_TY_BiEdge1 = sum(Error_All_TY_BiEdge1)./100;

load calibration_results/Error_All_BiEdge2_datatype.txt;
Error_All_RX_BiEdge2 = Error_All_BiEdge2_datatype(1:100, :);
Error_All_TX_BiEdge2 = Error_All_BiEdge2_datatype(101:200, :);
Error_All_RY_BiEdge2 = Error_All_BiEdge2_datatype(201:300, :);
Error_All_TY_BiEdge2 = Error_All_BiEdge2_datatype(301:400, :);
Error_Avrg_RX_BiEdge2 = sum(Error_All_RX_BiEdge2)./100;  
Error_Avrg_RX_BiEdge2 = Error_Avrg_RX_BiEdge2*180/pi;
Error_Avrg_TX_BiEdge2 = sum(Error_All_TX_BiEdge2)./100;
Error_Avrg_RY_BiEdge2 = sum(Error_All_RY_BiEdge2)./100;  
Error_Avrg_RY_BiEdge2 = Error_Avrg_RY_BiEdge2*180/pi;
Error_Avrg_TY_BiEdge2 = sum(Error_All_TY_BiEdge2)./100;

load calibration_results/Error_All_RightEdge1_datatype.txt;
Error_All_RX_RightEdge1 = Error_All_RightEdge1_datatype(1:100, :);
Error_All_TX_RightEdge1 = Error_All_RightEdge1_datatype(101:200, :);
Error_All_RY_RightEdge1 = Error_All_RightEdge1_datatype(201:300, :);
Error_All_TY_RightEdge1 = Error_All_RightEdge1_datatype(301:400, :);
Error_Avrg_RX_RightEdge1 = sum(Error_All_RX_RightEdge1)./100;  
Error_Avrg_RX_RightEdge1 = Error_Avrg_RX_RightEdge1*180/pi;
Error_Avrg_TX_RightEdge1 = sum(Error_All_TX_RightEdge1)./100;
Error_Avrg_RY_RightEdge1 = sum(Error_All_RY_RightEdge1)./100;  
Error_Avrg_RY_RightEdge1 = Error_Avrg_RY_RightEdge1*180/pi;
Error_Avrg_TY_RightEdge1 = sum(Error_All_TY_RightEdge1)./100;

load calibration_results/Error_All_RightEdge2_datatype.txt;
Error_All_RX_RightEdge2 = Error_All_RightEdge2_datatype(1:100, :);
Error_All_TX_RightEdge2 = Error_All_RightEdge2_datatype(101:200, :);
Error_All_RY_RightEdge2 = Error_All_RightEdge2_datatype(201:300, :);
Error_All_TY_RightEdge2 = Error_All_RightEdge2_datatype(301:400, :);
Error_Avrg_RX_RightEdge2 = sum(Error_All_RX_RightEdge2)./100;  
Error_Avrg_RX_RightEdge2 = Error_Avrg_RX_RightEdge2*180/pi;
Error_Avrg_TX_RightEdge2 = sum(Error_All_TX_RightEdge2)./100;
Error_Avrg_RY_RightEdge2 = sum(Error_All_RY_RightEdge2)./100;  
Error_Avrg_RY_RightEdge2 = Error_Avrg_RY_RightEdge2*180/pi;
Error_Avrg_TY_RightEdge2 = sum(Error_All_TY_RightEdge2)./100;

load calibration_results/Error_All_Wang_datatype.txt;
Error_All_RX_Wang = Error_All_Wang_datatype(1:100, :);
Error_All_TX_Wang = Error_All_Wang_datatype(101:200, :);
Error_All_RY_Wang = Error_All_Wang_datatype(201:300, :);
Error_All_TY_Wang = Error_All_Wang_datatype(301:400, :);
Error_Avrg_RX_Wang = sum(Error_All_RX_Wang)./100;  Error_Avrg_RX_Wang = Error_Avrg_RX_Wang*180/pi;
Error_Avrg_TX_Wang = sum(Error_All_TX_Wang)./100;
Error_Avrg_RY_Wang = sum(Error_All_RY_Wang)./100;  Error_Avrg_RY_Wang = Error_Avrg_RY_Wang*180/pi;
Error_Avrg_TY_Wang = sum(Error_All_TY_Wang)./100;

Error_Avrg_Wang = [Error_Avrg_RX_Wang; Error_Avrg_TX_Wang; Error_Avrg_RY_Wang; Error_Avrg_TY_Wang];
Error_Avrg_BiEdge1 = [Error_Avrg_RX_BiEdge1; Error_Avrg_TX_BiEdge1; Error_Avrg_RY_BiEdge1; Error_Avrg_TY_BiEdge1];
Error_Avrg_BiEdge2 = [Error_Avrg_RX_BiEdge2; Error_Avrg_TX_BiEdge2; Error_Avrg_RY_BiEdge2; Error_Avrg_TY_BiEdge2];
Error_Avrg_RightEdge1 = [Error_Avrg_RX_RightEdge1; Error_Avrg_TX_RightEdge1; Error_Avrg_RY_RightEdge1; Error_Avrg_TY_RightEdge1];
Error_Avrg_RightEdge2 = [Error_Avrg_RX_RightEdge2; Error_Avrg_TX_RightEdge2; Error_Avrg_RY_RightEdge2; Error_Avrg_TY_RightEdge2];

Error_Avrg_RX_Wang = transpose(Error_Avrg_RX_Wang);
Error_Avrg_RX_BiEdge1 = transpose(Error_Avrg_RX_BiEdge1);
Error_Avrg_RX_BiEdge2 = transpose(Error_Avrg_RX_BiEdge2);
Error_Avrg_RX_RightEdge1 = transpose(Error_Avrg_RX_RightEdge1);
Error_Avrg_RX_RightEdge2 = transpose(Error_Avrg_RX_RightEdge2);

Error_Avrg_TX_Wang = transpose(Error_Avrg_TX_Wang);
Error_Avrg_TX_BiEdge1 = transpose(Error_Avrg_TX_BiEdge1);
Error_Avrg_TX_BiEdge2 = transpose(Error_Avrg_TX_BiEdge2);
Error_Avrg_TX_RightEdge1 = transpose(Error_Avrg_TX_RightEdge1);
Error_Avrg_TX_RightEdge2 = transpose(Error_Avrg_TX_RightEdge2);

Error_Avrg_RY_Wang = transpose(Error_Avrg_RY_Wang);
Error_Avrg_RY_BiEdge1 = transpose(Error_Avrg_RY_BiEdge1);
Error_Avrg_RY_BiEdge2 = transpose(Error_Avrg_RY_BiEdge2);
Error_Avrg_RY_RightEdge1 = transpose(Error_Avrg_RY_RightEdge1);
Error_Avrg_RY_RightEdge2 = transpose(Error_Avrg_RY_RightEdge2);

Error_Avrg_TY_Wang = transpose(Error_Avrg_TY_Wang);
Error_Avrg_TY_BiEdge1 = transpose(Error_Avrg_TY_BiEdge1);
Error_Avrg_TY_BiEdge2 = transpose(Error_Avrg_TY_BiEdge2);
Error_Avrg_TY_RightEdge1 = transpose(Error_Avrg_TY_RightEdge1);
Error_Avrg_TY_RightEdge2 = transpose(Error_Avrg_TY_RightEdge2);

RX_datatype_display = [ % Error_Avrg_Li_different_datatype(:,1), Error_Avrg_Dornaika_different_datatype(:,1), Error_Avrg_Shah_different_datatype(:,1), ...
    Error_Avrg_RX_Wang, Error_Avrg_RX_RightEdge1, Error_Avrg_RX_RightEdge2, Error_Avrg_RX_BiEdge1, Error_Avrg_RX_BiEdge2 ];
TX_datatype_display = [ % Error_Avrg_Li_different_datatype(:,2), Error_Avrg_Dornaika_different_datatype(:,2), Error_Avrg_Shah_different_datatype(:,2),...
    Error_Avrg_TX_Wang, Error_Avrg_TX_RightEdge1, Error_Avrg_TX_RightEdge2, Error_Avrg_TX_BiEdge1, Error_Avrg_TX_BiEdge2,];
RY_datatype_display = [ %Error_Avrg_Li_different_datatype(:,3), Error_Avrg_Dornaika_different_datatype(:,3), Error_Avrg_Shah_different_datatype(:,3),...
    Error_Avrg_RY_Wang, Error_Avrg_RY_RightEdge1, Error_Avrg_RY_RightEdge2, Error_Avrg_RY_BiEdge1, Error_Avrg_RY_BiEdge2];
TY_datatype_display = [ %Error_Avrg_Li_different_datatype(:,4), Error_Avrg_Dornaika_different_datatype(:,4), Error_Avrg_Shah_different_datatype(:,4),...
    Error_Avrg_TY_Wang, Error_Avrg_TY_RightEdge1, Error_Avrg_TY_RightEdge2, Error_Avrg_TY_BiEdge1, Error_Avrg_TY_BiEdge2];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot the Y's rotation error of different data types %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
X=[ 1 2 3 4 5 ];
xticklabels({})
plot( X, RY_datatype_display(1,:), '*r','markersize', 10 ); hold on
plot( X, RY_datatype_display(2,:), '+g','markersize', 10 ); hold on
plot( X, RY_datatype_display(3,:), 'ob','markersize', 10 ); hold on
plot( X, RY_datatype_display(4,:), 'xm','markersize', 10 ); hold on
plot( X, RY_datatype_display(1,:), 'r');
plot( X, RY_datatype_display(2,:), 'g');
plot( X, RY_datatype_display(3,:), 'b');
plot( X, RY_datatype_display(4,:), 'm');

set( gca,'xtick', 1:5);
set( gca,'XtickLabel',{ 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu' });
xlabel( 'Different methods' );
ylabel({'Rotation Error(Y)';'(in degrees)'});
title( 'Y Rotation Error' );
legend( 'Scattered large', 'Clustered large', 'Scattered small', 'Clustered small', 'Location', 'NorthEastOutside');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot the Y's translation error of different data types %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2);
X=[ 1 2 3 4 5 ];
xticklabels({})
plot( X, TY_datatype_display(1,:), '*r','markersize',10); hold on
plot( X, TY_datatype_display(2,:), '+g','markersize',10); hold on
plot( X, TY_datatype_display(3,:), 'ob','markersize',10); hold on
plot( X, TY_datatype_display(4,:), 'xm','markersize',10); hold on
plot( X, TY_datatype_display(1,:), 'r');
plot( X, TY_datatype_display(2,:), 'g');
plot( X, TY_datatype_display(3,:), 'b');
plot( X, TY_datatype_display(4,:), 'm');

set( gca,'xtick', 1:5);
set( gca,'XtickLabel',{ 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu' });
xlabel( 'Different methods');
ylabel( {'Translation Error(Y)';'(in meters)'});
title( 'Y Translation Error');
legend( 'Scattered large', 'Clustered large', 'Scattered small', 'Clustered small', 'Location','NorthEastOutside');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot the X's rotation error of different data types %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
X=[ 1 2 3 4 5 ];
xticklabels({})
plot( X, RX_datatype_display(1,:), '*r','markersize', 10 ); hold on
plot( X, RX_datatype_display(2,:), '+g','markersize', 10 ); hold on
plot( X, RX_datatype_display(3,:), 'ob','markersize', 10 ); hold on
plot( X, RX_datatype_display(4,:), 'xm','markersize', 10 ); hold on
plot( X, RX_datatype_display(1,:), 'r');
plot( X, RX_datatype_display(2,:), 'g');
plot( X, RX_datatype_display(3,:), 'b');
plot( X, RX_datatype_display(4,:), 'm');

set( gca,'xtick', 1:5);
set( gca,'XtickLabel',{ 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu' });
xlabel( 'Different methods' );
ylabel({'Rotation Error(X)';'(in degrees)'});
title( 'X Rotation Error' );
legend( 'Scattered large', 'Clustered large', 'Scattered small', 'Clustered small', 'Location', 'NorthEastOutside');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot the X's translation error of different data types %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4);
X=[ 1 2 3 4 5 ];
xticklabels({})
plot( X, TX_datatype_display(1,:), '*r','markersize',10); hold on
plot( X, TX_datatype_display(2,:), '+g','markersize',10); hold on
plot( X, TX_datatype_display(3,:), 'ob','markersize',10); hold on
plot( X, TX_datatype_display(4,:), 'xm','markersize',10); hold on
plot( X, TX_datatype_display(1,:), 'r');
plot( X, TX_datatype_display(2,:), 'g');
plot( X, TX_datatype_display(3,:), 'b');
plot( X, TX_datatype_display(4,:), 'm');

set( gca,'xtick', 1:5);
%set( gca,'XtickLabel',{ 'Dornaika', 'Shah', 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu' });
set( gca,'XtickLabel',{ 'Wang', 'NonWght-1', 'Wght-1', 'Liu', 'Wght-Liu' });
xlabel( 'Different methods');
ylabel( {'Translation Error(X)';'(in meters)'});
title( 'X Translation Error');
legend( 'Scattered large', 'Clustered large', 'Scattered small', 'Clustered small', 'Location','NorthEastOutside');
