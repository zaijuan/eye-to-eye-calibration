quat_leftcam_leftmarker = [0.0747764237533558,-0.978172365435691,-0.0643274021440987,0.182891484990403];
rot_leftcam_leftmarker = quat2rotm(quat_leftcam_leftmarker);
trans_leftcam_leftmarker = [-0.439600613702652; -0.232116549127759; 1.00720170671073];
quat_rightcam_rightmarker = [0.0183209525410194,0.978172365435882,0.0969219836467715,-0.182891484989148];
rot_rightcam_rightmarker = quat2rotm(quat_rightcam_rightmarker);
trans_rightcam_rightmarker = [-0.121524448515421; -0.0371650680678677; 0.532882839822978];
T_leftcam_leftmarker = [rot_leftcam_leftmarker, trans_leftcam_leftmarker;
    0 0 0 1];
T_rightcam_rightmarker = [rot_rightcam_rightmarker, trans_rightcam_rightmarker;
    0 0 0 1];
T_leftcam_rightcam_true = [     0.5          0          sqrt(3)/2      0.6*sqrt(3);
                                 0           1            0               0;
                               -sqrt(3)/2    0            0.5             0.6;
                                 0           0            0               1];
T_leftmarker_rightmarker_true = [0.500000000000194,-5.65214196070398e-13,-0.866025403784327,1.78049636190688;
    5.83868653766371e-13,1,-3.15748708347219e-13,-3.20410364906820e-13;
    0.866025403784326,-3.47674584326588e-13,0.500000000000194,-0.633910161513293;
    0,0,0,1];                             

T_leftside = inv(T_leftcam_leftmarker) * T_leftcam_rightcam_true * T_rightcam_rightmarker;
T_equal = T_leftside - T_leftmarker_rightmarker_true;
a = 0;