
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "types_edge_extension.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "cv2_eigen.hpp"

#include <g2o/core/sparse_optimizer_terminate_action.h>


class PixelOpt{

private:
    g2o::SparseOptimizer optimizer_;
    int optimization_round;

    // In this simulation, there are two vertex, which are represented as X_vertex and Y_vertex.
    int X_vertexID;  // vertex index of X_vertex, which represents the transformation from right marker frame to left marker frame.
    int Y_vertexID;  // vertex index of Y_vertex, which represents the transformation from right camera frame to left camera frame.
    int B_vertexID;  // for building left edge, it needs to be initialized during the adding of the vertex.
    int A_vertexID;  // for building right edge, it needs to be initialized during the adding of the vertex.

    tf::Matrix3x3 r_leftmarker_rightmarker_true;
    tf::Vector3 t_leftmarker_rightmarker;
    tf::Transform T_leftmarker_rightmarker_true; // ground truth of X.

    tf::Matrix3x3 r_leftcam_rightcam;
    tf::Vector3 t_leftcam_rightcam;
    tf::Transform T_leftcam_rightcam_true;      // ground truth of Y.

    double focal_length;
    double leftcameraparam[3][3];
    double rightcameraparam[3][3];
    Eigen::Matrix3d KMat_leftcam;   // This redundant definition of camera parameter is for g2o optimization.
    Eigen::Matrix3d KMat_rightcam;  // This redundant definition of camera parameter is for g2o optimization.

    cv::Mat distCoeff;
    cv::Mat leftCamParam;
    cv::Mat rightCamParam;  // The camera parameter.

    tf::Transform T_leftmarker_rightmarker_est_3d;   // the estimated X from AAXX = YYBB;
    tf::Transform T_leftcam_rightcam_est_3d;        // the estimated Y from AAXX = YYBB;

    tf::Transform T_leftmarker_rightmarker_est_biEdge_1;  // the estimated X from final nonlinear pixel-level optimization without building adaptive information..
    tf::Transform T_leftcam_rightcam_est_biEdge_1;        // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.

    tf::Transform T_leftmarker_rightmarker_est_biEdge_2;  // the estimated X from final nonlinear pixel-level optimization with building adaptive information.
    tf::Transform T_leftcam_rightcam_est_biEdge_2;        // the estimated Y from final nonlinear pixel-level optimization with building adaptive information.

    tf::Transform T_leftmarker_rightmarker_est_left_1;    // the estimated X from final nonlinear pixel-level optimization without building adaptive information.
    tf::Transform T_leftcam_rightcam_est_left_1;          // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.

    tf::Transform T_leftmarker_rightmarker_est_left_2;    // the estimated X from final nonlinear pixel-level optimization with building adaptive information.
    tf::Transform T_leftcam_rightcam_est_left_2;          // the estimated Y from final nonlinear pixel-level optimization with building adaptive information.

    tf::Transform T_leftmarker_rightmarker_est_right_1;    // the estimated X from final nonlinear pixel-level optimization without building adaptive information.
    tf::Transform T_leftcam_rightcam_est_right_1;          // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.

    tf::Transform T_leftmarker_rightmarker_est_right_2;    // the estimated X from final nonlinear pixel-level optimization with building adaptive information.
    tf::Transform T_leftcam_rightcam_est_right_2;          // the estimated Y from final nonlinear pixel-level optimization with building adaptive information.

    tf::Transform T_leftcam_rightcam_init;        // for the Y vertex input initialization.
    tf::Transform T_leftmarker_rightmarker_init;  // for the X vertex input initialization.

    std::vector<cv::Point3f> objPoints; // Marker feature 3d points. In this experiment the two marker boards used are the same.

    std::vector< std::vector<cv::Point2f> > leftsideMarkerimg2d_largeproj_bank;
    std::vector< std::vector<cv::Point2f> > rightsideMarkerimg2d_largeproj_bank; // Those two VECTOR of VECTORs store the noise-corrupted 2-d image coordinates of the marker features.

    std::vector<double> leftsideMarker_largeproj_area_bank;
    std::vector<double> rightsideMarker_largeproj_area_bank;

    std::vector< std::vector<cv::Point2f> > leftsideMarkerimg2d_smallproj_bank;
    std::vector< std::vector<cv::Point2f> > rightsideMarkerimg2d_smallproj_bank; // Those two VECTOR of VECTORs store the noise-corrupted 2-d image coordinates of the marker features.

    std::vector<double> leftsideMarker_smallproj_area_bank;
    std::vector<double> rightsideMarker_smallproj_area_bank;

    std::vector< std::vector<cv::Point2f> > leftsideMarkerimg2d_set;
    std::vector< std::vector<cv::Point2f> > rightsideMarkerimg2d_set; // Those two VECTOR of VECTORs store the noise-corrupted 2-d image coordinates of the marker features.

    std::vector<double> leftsideMarker_proj_area_set;
    std::vector<double> rightsideMarker_proj_area_set;

    int posepairnumber;      // This is not fixed.
    int cornerPointNum;  // This is determined by the design.

    double leftsideMarkerSet_smallest_area, rightsideMarkerSet_smallest_area;
    double normalizor_largest_left_edge, normalizor_largest_right_edge;

    std::vector<cv::Mat> rod_leftcam_leftmarker_largeproj_bank;
    std::vector<cv::Mat> t_leftcam_leftmarker_largeproj_bank;
    std::vector<cv::Mat> rod_rightcam_rightmarker_largeproj_bank;
    std::vector<cv::Mat> t_rightcam_rightmarker_largeproj_bank;

    std::vector<tf::Transform> T_leftcam_leftmarker_largeproj_est_bank;
    std::vector<tf::Transform> T_rightcam_rightmarker_largeproj_est_bank;

    std::vector<cv::Mat> rod_leftcam_leftmarker_smallproj_bank;
    std::vector<cv::Mat> t_leftcam_leftmarker_smallproj_bank;
    std::vector<cv::Mat> rod_rightcam_rightmarker_smallproj_bank;
    std::vector<cv::Mat> t_rightcam_rightmarker_smallproj_bank;

    std::vector<tf::Transform> T_leftcam_leftmarker_smallproj_est_bank;
    std::vector<tf::Transform> T_rightcam_rightmarker_smallproj_est_bank;

    std::vector<cv::Mat> rod_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> t_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> rod_rightcam_rightmarker_est_set;
    std::vector<cv::Mat> t_rightcam_rightmarker_est_set;

    std::vector<tf::Transform> T_leftcam_leftmarker_est_set;
    std::vector<tf::Transform> T_rightcam_rightmarker_est_set;

    std::vector<cv::Mat> rvecsAA_set; // the set of rotation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsAA_set; // the set of translation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> rvecsBB_set; // the set of rotation vectors of BB for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsBB_set; // the set of translation vectors of BB for the function (AAXX = YYBB) optimization

    std::vector<cv::Mat> rvecsAA_subset; // the subset of rotation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsAA_subset; // the subset of translation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> rvecsBB_subset; // the subset of rotation vectors of BB for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsBB_subset; // the subset of translation vectors of BB for the function (AAXX = YYBB) optimization

    double Error_RX_BiEdge1_number[100][10];
    double Error_TX_BiEdge1_number[100][10];
    double Error_RY_BiEdge1_number[100][10];
    double Error_TY_BiEdge1_number[100][10];

    double Error_RX_BiEdge2_number[100][10];
    double Error_TX_BiEdge2_number[100][10];
    double Error_RY_BiEdge2_number[100][10];
    double Error_TY_BiEdge2_number[100][10];

    double Error_RX_RightEdge1_number[100][10];
    double Error_TX_RightEdge1_number[100][10];
    double Error_RY_RightEdge1_number[100][10];
    double Error_TY_RightEdge1_number[100][10];

    double Error_RX_RightEdge2_number[100][10];
    double Error_TX_RightEdge2_number[100][10];
    double Error_RY_RightEdge2_number[100][10];
    double Error_TY_RightEdge2_number[100][10];

    double Error_RX_Wang_number[100][10];
    double Error_TX_Wang_number[100][10];
    double Error_RY_Wang_number[100][10];
    double Error_TY_Wang_number[100][10];

    double Error_RX_BiEdge1_noise[100][8];
    double Error_TX_BiEdge1_noise[100][8];
    double Error_RY_BiEdge1_noise[100][8];
    double Error_TY_BiEdge1_noise[100][8];

    double Error_RX_BiEdge2_noise[100][8];
    double Error_TX_BiEdge2_noise[100][8];
    double Error_RY_BiEdge2_noise[100][8];
    double Error_TY_BiEdge2_noise[100][8];

    double Error_RX_RightEdge1_noise[100][8];
    double Error_TX_RightEdge1_noise[100][8];
    double Error_RY_RightEdge1_noise[100][8];
    double Error_TY_RightEdge1_noise[100][8];

    double Error_RX_RightEdge2_noise[100][8];
    double Error_TX_RightEdge2_noise[100][8];
    double Error_RY_RightEdge2_noise[100][8];
    double Error_TY_RightEdge2_noise[100][8];

    double Error_RX_Wang_noise[100][8];
    double Error_TX_Wang_noise[100][8];
    double Error_RY_Wang_noise[100][8];
    double Error_TY_Wang_noise[100][8];

    double Error_RX_BiEdge1_datatype[100][4];
    double Error_TX_BiEdge1_datatype[100][4];
    double Error_RY_BiEdge1_datatype[100][4];
    double Error_TY_BiEdge1_datatype[100][4];

    double Error_RX_BiEdge2_datatype[100][4];
    double Error_TX_BiEdge2_datatype[100][4];
    double Error_RY_BiEdge2_datatype[100][4];
    double Error_TY_BiEdge2_datatype[100][4];

    double Error_RX_RightEdge1_datatype[100][4];
    double Error_TX_RightEdge1_datatype[100][4];
    double Error_RY_RightEdge1_datatype[100][4];
    double Error_TY_RightEdge1_datatype[100][4];

    double Error_RX_RightEdge2_datatype[100][4];
    double Error_TX_RightEdge2_datatype[100][4];
    double Error_RY_RightEdge2_datatype[100][4];
    double Error_TY_RightEdge2_datatype[100][4];

    double Error_RX_Wang_datatype[100][4];
    double Error_TX_Wang_datatype[100][4];
    double Error_RY_Wang_datatype[100][4];
    double Error_TY_Wang_datatype[100][4];

    double Jacobian_upper_left_A[2][6];
    double Jacobian_upper_right_A[2][6];
    double Jacobian_lower_left_A[2][6];
    double Jacobian_lower_right_A[2][6];

    double Jacobian_upper_left_B[2][6];
    double Jacobian_upper_right_B[2][6];
    double Jacobian_lower_left_B[2][6];
    double Jacobian_lower_right_B[2][6];

    tf::Vector3 get_R_multiply_T( tf::Matrix3x3 R, tf::Vector3 T );
    void clearMeasurementBank( );
    void clearDataAndVectorContainer( );

    void optimizeAAXXequalsYYBB( int numberOfImagesToUse, cv::Mat& rotationMatrixX, cv::Mat& rotationMatrixY, cv::Mat& tx, cv::Mat& ty,
                             const std::vector<cv::Mat>& rvecsAA_set, const std::vector<cv::Mat>& tvecsAA_set,
                             const std::vector<cv::Mat>& rvecsBB_set, const std::vector<cv::Mat>& tvecsBB_set);

    void addGaussianNoise( std::vector<cv::Point2f>& imgPoint, double noise_level );
    cv::Mat makeMatrixFromVector( cv::Mat rotationVector );

    void calculate4CornerJacobianMatrix( tf::Transform A_est, tf::Transform B_est  );
    void calculateJacobian( tf::Transform T, cv::Point3f X, double jacobian[][6] );

    double calculateRotationError( tf::Transform X_est, tf::Transform X_true );
    double calculateTranslationError( tf::Transform X_est, tf::Transform X_true );
    double calculateSum_3dim_trans_error( tf::Transform X_est, tf::Transform X_true );

    //void sort_results_number( double Error_RX[][10], double Error_TX[][10], double Error_RY[][10], double Error_TY[][10] );
    //void sort_results_noise( double Error_RX[][8], double Error_TX[][8], double Error_RY[][8], double Error_TY[][8] );
    //void sort_results_datatype( double Error_RX[][4], double Error_TX[][4], double Error_RY[][4], double Error_TY[][4] );

    void store_error_results_number_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][10], double Error_TX[][10], double Error_RY[][10], double Error_TY[][10]);
    void store_error_results_noise_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][8], double Error_TX[][8], double Error_RY[][8], double Error_TY[][8]);
    void store_error_results_datatype_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][4], double Error_TX[][4], double Error_RY[][4], double Error_TY[][4]);

    g2o::OptimizableGraph::Vertex* buildVertex( const int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed);
    g2o::OptimizableGraph::Edge* buildLeftEdge( const int markerRigId, const int carCamId, const int Bi_id, const int measurementid, const int featureid, const double area_ratio=1);
    g2o::OptimizableGraph::Edge* buildRightEdge( const int markerRigId, const int carCamId, const int Ai_id, const int measurementid, const int featureid, const double area_ratio=1);

public:
    PixelOpt();
    void ReadMeasurementsFromFile( std::string largeprojection_filename, std::string smallprojection_filename );
    void GenerateDifferentNoiseLevelMeasurements( double noise_level );
    void GenerateSetNumberofMeasurements( int setnumber, std::string distribute_flag );
    void GenerateSetNumberofMeasurements( int setnumber, double projection_area_thred );   // Fuction overload.
    void PickOutSubset( );
    void CalculateAAiandBBi( std::string bank_or_set );
    void SolveAAXXequalsYYBB( bool use_whole_set );

    void StartPixelLevelOptimization_with_adaptive_informat( );
    void StartPixelLevelOptimization_without_adaptive_informat( );

    void StartPixelLevelOptimization_right_with_adaptive_informat( );
    void StartPixelLevelOptimization_right_without_adaptive_informat( );

    void StoreResultsfromDifferentMeasurements( );
    void StoreCurrentRoundErrorResults_number( int iteration_round, int i );
    void StoreCurrentRoundErrorResults_noise( int iteration_round, int i );
    void StoreCurrentRoundErrorResults_datatype( int iteration_round, int i );
    void StoreAllErrorResultsToFile_number( std::string store_path );
    void StoreAllErrorResultsToFile_noise( std::string store_path );
    void StoreAllErrorResultsToFile_datatype( std::string store_path );
    void StoreAAiandBBiDiffrtNoiseLevel( std::string largeproj_store_path, std::string smallproj_store_path );
};



PixelOpt::PixelOpt():optimization_round(1000), X_vertexID(0), Y_vertexID(1), B_vertexID(1), A_vertexID(1),
    focal_length(500), leftsideMarkerSet_smallest_area(0), rightsideMarkerSet_smallest_area(0), normalizor_largest_left_edge(0), normalizor_largest_right_edge(0),
    leftcameraparam{ {focal_length, 0, 320}, {0, focal_length, 240}, {0, 0, 1} },
    rightcameraparam{ {focal_length, 0, 320}, {0, focal_length, 240}, {0, 0, 1} },
    Jacobian_upper_left_A{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_upper_right_A{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_lower_left_A{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_lower_right_A{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_upper_left_B{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_upper_right_B{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_lower_left_B{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    Jacobian_lower_right_B{ {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0} },
    posepairnumber(0), cornerPointNum(25){

    //SETUP the 3D features of the marker board.//
    cv::Point3f obj_3dPoint1( -0.2,  0.2, 0 );     objPoints.push_back( obj_3dPoint1 );
    cv::Point3f obj_3dPoint2( -0.1,  0.2, 0 );     objPoints.push_back( obj_3dPoint2 );
    cv::Point3f obj_3dPoint3(    0,  0.2, 0 );     objPoints.push_back( obj_3dPoint3 );
    cv::Point3f obj_3dPoint4(  0.1,  0.2, 0 );     objPoints.push_back( obj_3dPoint4 );
    cv::Point3f obj_3dPoint5(  0.2,  0.2, 0 );     objPoints.push_back( obj_3dPoint5 );
    cv::Point3f obj_3dPoint6( -0.2,  0.1, 0 );     objPoints.push_back( obj_3dPoint6 );
    cv::Point3f obj_3dPoint7( -0.1,  0.1, 0 );     objPoints.push_back( obj_3dPoint7 );
    cv::Point3f obj_3dPoint8(    0,  0.1, 0 );     objPoints.push_back( obj_3dPoint8 );
    cv::Point3f obj_3dPoint9(  0.1,  0.1, 0 );     objPoints.push_back( obj_3dPoint9 );
    cv::Point3f obj_3dPoint10( 0.2,  0.1, 0 );     objPoints.push_back( obj_3dPoint10 );
    cv::Point3f obj_3dPoint11(-0.2,    0, 0 );     objPoints.push_back( obj_3dPoint11 );
    cv::Point3f obj_3dPoint12(-0.1,    0, 0 );     objPoints.push_back( obj_3dPoint12 );
    cv::Point3f obj_3dPoint13(   0,    0, 0 );     objPoints.push_back( obj_3dPoint13 );
    cv::Point3f obj_3dPoint14( 0.1,    0, 0 );     objPoints.push_back( obj_3dPoint14 );
    cv::Point3f obj_3dPoint15( 0.2,    0, 0 );     objPoints.push_back( obj_3dPoint15 );
    cv::Point3f obj_3dPoint16(-0.2, -0.1, 0 );     objPoints.push_back( obj_3dPoint16 );
    cv::Point3f obj_3dPoint17(-0.1, -0.1, 0 );     objPoints.push_back( obj_3dPoint17 );
    cv::Point3f obj_3dPoint18(   0, -0.1, 0 );     objPoints.push_back( obj_3dPoint18 );
    cv::Point3f obj_3dPoint19( 0.1, -0.1, 0 );     objPoints.push_back( obj_3dPoint19 );
    cv::Point3f obj_3dPoint20( 0.2, -0.1, 0 );     objPoints.push_back( obj_3dPoint20 );
    cv::Point3f obj_3dPoint21(-0.2, -0.2, 0 );     objPoints.push_back( obj_3dPoint21 );
    cv::Point3f obj_3dPoint22(-0.1, -0.2, 0 );     objPoints.push_back( obj_3dPoint22 );
    cv::Point3f obj_3dPoint23(   0, -0.2, 0 );     objPoints.push_back( obj_3dPoint23 );
    cv::Point3f obj_3dPoint24( 0.1, -0.2, 0 );     objPoints.push_back( obj_3dPoint24 );
    cv::Point3f obj_3dPoint25( 0.2, -0.2, 0 );     objPoints.push_back( obj_3dPoint25 );


    r_leftmarker_rightmarker_true = tf::Matrix3x3 (0.499999999996154,    0,    -0.866025403786660,
                                                   0,                    1,     0,
                                                   0.866025403786660,    0,     0.499999999996154);
    t_leftmarker_rightmarker = tf::Vector3(        1.58049636190652,     0,     0.912499999999470);
    T_leftmarker_rightmarker_true = tf::Transform(r_leftmarker_rightmarker_true, t_leftmarker_rightmarker); // ground truth of Y.

    r_leftcam_rightcam = tf::Matrix3x3(0.5,                0,        0.866025403784439,
                                       0,                  1,        0,
                                      -0.866025403784439,  0,        0.5);
    t_leftcam_rightcam = tf::Vector3(  1.03923048454133,   0,       -0.6);
    T_leftcam_rightcam_true = tf::Transform(r_leftcam_rightcam, t_leftcam_rightcam); // ground truth of Y.

    KMat_leftcam(0,0) = leftcameraparam[0][0];   KMat_leftcam(0,1) = 0;                        KMat_leftcam(0,2) = leftcameraparam[0][2];
    KMat_leftcam(1,0) = 0;                       KMat_leftcam(1,1) = leftcameraparam[1][1];    KMat_leftcam(1,2) = leftcameraparam[1][2];
    KMat_leftcam(2,0) = 0;                       KMat_leftcam(2,1) = 0;                        KMat_leftcam(2,2) = 1;

    KMat_rightcam(0,0) = rightcameraparam[0][0];  KMat_rightcam(0,1) =0;                           KMat_rightcam(0,2) = rightcameraparam[0][2];
    KMat_rightcam(1,0) = 0;                       KMat_rightcam(1,1) = rightcameraparam[1][1];;    KMat_rightcam(1,2) = rightcameraparam[1][2];
    KMat_rightcam(2,0) = 0;                       KMat_rightcam(2,1) = 0;                          KMat_rightcam(2,2) = 1;

    distCoeff = cv::Mat::zeros(4, 1, CV_32FC1 );
    leftCamParam = cv::Mat(3, 3, CV_64F, leftcameraparam);
    rightCamParam = cv::Mat(3, 3, CV_64F, rightcameraparam);  // The camera parameter.

    T_leftmarker_rightmarker_est_biEdge_1.setIdentity();
    T_leftcam_rightcam_est_biEdge_1.setIdentity();

    T_leftmarker_rightmarker_est_biEdge_2.setIdentity();
    T_leftcam_rightcam_est_biEdge_2.setIdentity();

    T_leftmarker_rightmarker_est_left_1 .setIdentity();
    T_leftcam_rightcam_est_left_1.setIdentity();

    T_leftmarker_rightmarker_est_left_2.setIdentity();
    T_leftcam_rightcam_est_left_2.setIdentity();

    T_leftmarker_rightmarker_est_right_1 .setIdentity();
    T_leftcam_rightcam_est_right_1.setIdentity();

    T_leftmarker_rightmarker_est_right_2.setIdentity();
    T_leftcam_rightcam_est_right_2.setIdentity();

    // ********************SETUP G2O************************* //
    typedef g2o::BlockSolver< g2o::BlockSolverTraits< 6, 2 > > Block_;
    Block_::LinearSolverType* linearSolver = new g2o::LinearSolverPCG<Block_::PoseMatrixType>();
    Block_* solver_ptr = new Block_( linearSolver );

    // Choosing the method to use by the optimizer
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    optimizer_.setAlgorithm( solver );
    optimizer_.setVerbose( false );

    // Set terminate thresholds
//    double gainTerminateThreshold_ = 1e-6;

    // Set Convergence Criterion
//    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction();
//    terminateAction->setGainThreshold( gainTerminateThreshold_ );
//    optimizer_.addPostIterationAction( terminateAction );
}



void PixelOpt::clearMeasurementBank(){

    leftsideMarkerimg2d_largeproj_bank.clear();
    rightsideMarkerimg2d_largeproj_bank.clear();

    T_leftcam_leftmarker_largeproj_est_bank.clear();
    T_rightcam_rightmarker_largeproj_est_bank.clear();

    leftsideMarkerimg2d_smallproj_bank.clear();
    rightsideMarkerimg2d_smallproj_bank.clear();

    T_leftcam_leftmarker_smallproj_est_bank.clear();
    T_rightcam_rightmarker_smallproj_est_bank.clear();
}



void PixelOpt::clearDataAndVectorContainer( ){

    leftsideMarkerimg2d_set.clear();
    rightsideMarkerimg2d_set.clear();

    leftsideMarker_proj_area_set.clear();
    rightsideMarker_proj_area_set.clear();

    rod_leftcam_leftmarker_est_set.clear();
    t_leftcam_leftmarker_est_set.clear();

    rod_rightcam_rightmarker_est_set.clear();
    t_rightcam_rightmarker_est_set.clear();

    T_leftcam_leftmarker_est_set.clear();
    T_rightcam_rightmarker_est_set.clear();

    rvecsAA_set.clear();  // the set of rotation vectors of AA for the function(AAXX = YYBB) optimization
    tvecsAA_set.clear();  // the set of translation vectors of AA for the function(AAXX = YYBB) optimization
    rvecsBB_set.clear();  // the set of rotation vectors of BB for the function(AAXX = YYBB) optimization
    tvecsBB_set.clear();  // the set of translation vectors of BB for the function(AAXX = YYBB) optimization

    rvecsAA_subset.clear();  // the set of rotation vectors of AA for the function(AAXX = YYBB) optimization
    tvecsAA_subset.clear();  // the set of translation vectors of AA for the function(AAXX = YYBB) optimization
    rvecsBB_subset.clear();  // the set of rotation vectors of BB for the function(AAXX = YYBB) optimization
    tvecsBB_subset.clear();  // the set of translation vectors of BB for the function(AAXX = YYBB) optimization
}



void PixelOpt::addGaussianNoise( std::vector<cv::Point2f> &imgPoint, double noise_level ){
    cv::Mat NOISE = cv::Mat(imgPoint.size(), 2, CV_32F);
    cv::randn(NOISE, 0, noise_level);
    for (size_t i=0; i<imgPoint.size();i++){
        imgPoint[i].x += NOISE.at<float>(i,0);
        imgPoint[i].y += NOISE.at<float>(i,1);
    }
}



tf::Vector3 PixelOpt::get_R_multiply_T(tf::Matrix3x3 R, tf::Vector3 T){
    tf::Vector3 row_unit, T_vec;
    double RT1, RT2, RT3;
    row_unit = R.getRow(0)*T;
    RT1 = row_unit.getX() + row_unit.getY() + row_unit.getZ();

    row_unit = R.getRow(1)*T;
    RT2 = row_unit.getX() + row_unit.getY() + row_unit.getZ();

    row_unit = R.getRow(2)*T;
    RT3 = row_unit.getX() + row_unit.getY() + row_unit.getZ();

    T_vec = tf::Vector3(RT1, RT2, RT3);
    return T_vec;
}



void PixelOpt::calculate4CornerJacobianMatrix( tf::Transform A_est, tf::Transform B_est ){
    calculateJacobian( A_est, objPoints.at(0), Jacobian_upper_left_A );
    calculateJacobian( A_est, objPoints.at(4), Jacobian_upper_right_A );
    calculateJacobian( A_est, objPoints.at(20), Jacobian_lower_left_A );
    calculateJacobian( A_est, objPoints.at(24), Jacobian_lower_right_A );
    calculateJacobian( B_est, objPoints.at(0), Jacobian_upper_left_B );
    calculateJacobian( B_est, objPoints.at(4), Jacobian_upper_right_B );
    calculateJacobian( B_est, objPoints.at(20), Jacobian_lower_left_B );
    calculateJacobian( B_est, objPoints.at(24), Jacobian_lower_right_B );
}



void PixelOpt::calculateJacobian( tf::Transform T, cv::Point3f X, double jacobian[][6] ){
    tf::Matrix3x3 R_ = T.getBasis();
    tf::Vector3 t_ = T.getOrigin();
    tf::Vector3 position_w( X.x, X.y, X.z );
    tf::Vector3 position_c = R_ * position_w + t_;
    double X_ = position_c.getX();
    double Y_ = position_c.getY();
    double Z_ = position_c.getZ();
    double XX_ = X_*X_;
    double YY_ = Y_*Y_;
    double ZZ_ = Z_*Z_;

    jacobian[0][0] = -focal_length/Z_;
    jacobian[0][1] = 0;
    jacobian[0][2] = focal_length*X_/ZZ_;
    jacobian[0][3] = focal_length*X_*Y_/ZZ_;
    jacobian[0][4] = -( focal_length+(focal_length*XX_/ZZ_) );
    jacobian[0][5] = focal_length*Y_/Z_,
    jacobian[1][0] = 0;
    jacobian[1][1] = -focal_length/Z_;
    jacobian[1][2] = focal_length*Y_/ZZ_;
    jacobian[1][3] = focal_length+(focal_length*YY_/ZZ_);
    jacobian[1][4] = -focal_length*X_*Y_/ZZ_;
    jacobian[1][5] = -focal_length*X_/Z_;
}



double PixelOpt::calculateRotationError( tf::Transform X_est, tf::Transform X_true ){
    const double PI = 3.141592653589793;
    tf::Quaternion q_est = X_est.getRotation();
    tf::Quaternion q_true = X_true.getRotation();
    double quat_dif = q_est.getX()*q_true.getX() + q_est.getY()*q_true.getY() + q_est.getZ()*q_true.getZ() + q_est.getW()*q_true.getW();
    double radians_quat_dif = std::acos(quat_dif);
    double minradians_quat_dif = std::min( radians_quat_dif, PI-radians_quat_dif );
    return minradians_quat_dif;
}



double PixelOpt::calculateTranslationError( tf::Transform X_est, tf::Transform X_true ){
    double x_dif = X_est.getOrigin().getX() - X_true.getOrigin().getX();
    double y_dif = X_est.getOrigin().getY() - X_true.getOrigin().getY();
    double z_dif = X_est.getOrigin().getZ() - X_true.getOrigin().getZ();
    double dist_dif = std::sqrt( x_dif*x_dif + y_dif*y_dif + z_dif*z_dif );
    return dist_dif;
}



double PixelOpt::calculateSum_3dim_trans_error( tf::Transform X_est, tf::Transform X_true ){
    double x_dif = X_est.getOrigin().getX() - X_true.getOrigin().getX();
    double y_dif = X_est.getOrigin().getY() - X_true.getOrigin().getY();
    double z_dif = X_est.getOrigin().getZ() - X_true.getOrigin().getZ();
    double dist_dif = std::abs( x_dif ) + std::abs( y_dif ) + std::abs( z_dif );
    return dist_dif;
}



void PixelOpt::ReadMeasurementsFromFile( std::string largeprojection_filename, std::string smallprojection_filename ){

    double area_leftmarker, area_rightmarker, a_1, a_2, a_3, a_4, a_5, a_6, a_7, b_1, b_2, b_3, b_4, b_5, b_6, b_7;
    double q_x, q_y, q_z, q_w, t_x, t_y, t_z;

    tf::Vector3 tf_translation_vector;

    tf::Transform T_leftcam_leftmarker;
    tf::Quaternion quat_leftcam_leftmarker;
    cv::Mat mat_leftcam_leftmarker;
    cv::Mat rod_leftcam_leftmarker_vec;
    cv::Mat t_leftcam_leftmarker_vec;

    tf::Transform T_rightcam_rightmarker;
    cv::Mat mat_rightcam_rightmarker;
    cv::Mat rod_rightcam_rightmarker_vec;
    cv::Mat t_rightcam_rightmarker_vec;

    std::ifstream infile( largeprojection_filename );

    while ( infile>>area_leftmarker>>area_rightmarker>>a_1>>a_2>>a_3>>a_4>>a_5>>a_6>>a_7>>b_1 >>b_2>>b_3>>b_4>>b_5>>b_6>>b_7 ){
        q_w = a_1; q_x = a_2; q_y = a_3; q_z = a_4;
        t_x = a_5; t_y = a_6; t_z = a_7;
        quat_leftcam_leftmarker = tf::Quaternion( q_x, q_y, q_z, q_w);
        tf_translation_vector = tf::Vector3( t_x, t_y, t_z );
        T_leftcam_leftmarker = tf::Transform( quat_leftcam_leftmarker, tf_translation_vector );
        mat_leftcam_leftmarker = ( cv::Mat_<double>(3,3)<<T_leftcam_leftmarker.getBasis().getRow(0).getX(), T_leftcam_leftmarker.getBasis().getRow(0).getY(), T_leftcam_leftmarker.getBasis().getRow(0).getZ(),
                      T_leftcam_leftmarker.getBasis().getRow(1).getX(), T_leftcam_leftmarker.getBasis().getRow(1).getY(), T_leftcam_leftmarker.getBasis().getRow(1).getZ(),
                      T_leftcam_leftmarker.getBasis().getRow(2).getX(), T_leftcam_leftmarker.getBasis().getRow(2).getY(), T_leftcam_leftmarker.getBasis().getRow(2).getZ() );
        rod_leftcam_leftmarker_vec = (cv::Mat_<double>(3,1)<<0, 0, 0);
        cv::Rodrigues( mat_leftcam_leftmarker, rod_leftcam_leftmarker_vec );
        t_leftcam_leftmarker_vec = ( cv::Mat_<double>(3,1)<<t_x, t_y, t_z );
        rod_leftcam_leftmarker_largeproj_bank.push_back( rod_leftcam_leftmarker_vec );  t_leftcam_leftmarker_largeproj_bank.push_back( t_leftcam_leftmarker_vec );
        leftsideMarker_largeproj_area_bank.push_back( area_leftmarker );

        T_rightcam_rightmarker =  T_leftcam_rightcam_true.inverse() * T_leftcam_leftmarker * T_leftmarker_rightmarker_true;
        mat_rightcam_rightmarker = (cv::Mat_<double>(3,3)<<T_rightcam_rightmarker.getBasis().getRow(0).getX(), T_rightcam_rightmarker.getBasis().getRow(0).getY(), T_rightcam_rightmarker.getBasis().getRow(0).getZ(),
                     T_rightcam_rightmarker.getBasis().getRow(1).getX(), T_rightcam_rightmarker.getBasis().getRow(1).getY(), T_rightcam_rightmarker.getBasis().getRow(1).getZ(),
                     T_rightcam_rightmarker.getBasis().getRow(2).getX(), T_rightcam_rightmarker.getBasis().getRow(2).getY(), T_rightcam_rightmarker.getBasis().getRow(2).getZ() );
        rod_rightcam_rightmarker_vec = ( cv::Mat_<double>(3,1)<<0, 0, 0 );
        cv::Rodrigues( mat_rightcam_rightmarker, rod_rightcam_rightmarker_vec );
        t_rightcam_rightmarker_vec = ( cv::Mat_<double>(3,1)<<T_rightcam_rightmarker.getOrigin().getX(), T_rightcam_rightmarker.getOrigin().getY(), T_rightcam_rightmarker.getOrigin().getZ());
        rod_rightcam_rightmarker_largeproj_bank.push_back( rod_rightcam_rightmarker_vec ); t_rightcam_rightmarker_largeproj_bank.push_back( t_rightcam_rightmarker_vec );
        rightsideMarker_largeproj_area_bank.push_back( area_rightmarker );

        if( infile.eof() )  break;
    } // the end of reading data file.
    infile.close();

    infile.open( smallprojection_filename );
    while ( infile>>area_leftmarker>>area_rightmarker>>a_1>>a_2>>a_3>>a_4>>a_5>>a_6>>a_7>>b_1 >>b_2>>b_3>>b_4>>b_5>>b_6>>b_7 ){
        q_w = a_1; q_x = a_2; q_y = a_3; q_z = a_4;
        t_x = a_5; t_y = a_6; t_z = a_7;
        quat_leftcam_leftmarker = tf::Quaternion( q_x, q_y, q_z, q_w);
        tf_translation_vector = tf::Vector3( t_x, t_y, t_z );
        T_leftcam_leftmarker = tf::Transform( quat_leftcam_leftmarker, tf_translation_vector );
        mat_leftcam_leftmarker = ( cv::Mat_<double>(3,3)<<T_leftcam_leftmarker.getBasis().getRow(0).getX(), T_leftcam_leftmarker.getBasis().getRow(0).getY(), T_leftcam_leftmarker.getBasis().getRow(0).getZ(),
                      T_leftcam_leftmarker.getBasis().getRow(1).getX(), T_leftcam_leftmarker.getBasis().getRow(1).getY(), T_leftcam_leftmarker.getBasis().getRow(1).getZ(),
                      T_leftcam_leftmarker.getBasis().getRow(2).getX(), T_leftcam_leftmarker.getBasis().getRow(2).getY(), T_leftcam_leftmarker.getBasis().getRow(2).getZ() );
        rod_leftcam_leftmarker_vec = (cv::Mat_<double>(3,1)<<0, 0, 0);
        cv::Rodrigues( mat_leftcam_leftmarker, rod_leftcam_leftmarker_vec );
        t_leftcam_leftmarker_vec = ( cv::Mat_<double>(3,1)<<t_x, t_y, t_z );
        rod_leftcam_leftmarker_smallproj_bank.push_back( rod_leftcam_leftmarker_vec );
        t_leftcam_leftmarker_smallproj_bank.push_back( t_leftcam_leftmarker_vec );
        leftsideMarker_smallproj_area_bank.push_back( area_leftmarker );

        T_rightcam_rightmarker =  T_leftcam_rightcam_true.inverse() * T_leftcam_leftmarker * T_leftmarker_rightmarker_true;
        mat_rightcam_rightmarker = (cv::Mat_<double>(3,3)<<T_rightcam_rightmarker.getBasis().getRow(0).getX(), T_rightcam_rightmarker.getBasis().getRow(0).getY(), T_rightcam_rightmarker.getBasis().getRow(0).getZ(),
                     T_rightcam_rightmarker.getBasis().getRow(1).getX(), T_rightcam_rightmarker.getBasis().getRow(1).getY(), T_rightcam_rightmarker.getBasis().getRow(1).getZ(),
                     T_rightcam_rightmarker.getBasis().getRow(2).getX(), T_rightcam_rightmarker.getBasis().getRow(2).getY(), T_rightcam_rightmarker.getBasis().getRow(2).getZ() );
        rod_rightcam_rightmarker_vec = ( cv::Mat_<double>(3,1)<<0, 0, 0 );
        cv::Rodrigues( mat_rightcam_rightmarker, rod_rightcam_rightmarker_vec );
        t_rightcam_rightmarker_vec = ( cv::Mat_<double>(3,1)<<T_rightcam_rightmarker.getOrigin().getX(), T_rightcam_rightmarker.getOrigin().getY(), T_rightcam_rightmarker.getOrigin().getZ());
        rod_rightcam_rightmarker_smallproj_bank.push_back( rod_rightcam_rightmarker_vec );
        t_rightcam_rightmarker_smallproj_bank.push_back( t_rightcam_rightmarker_vec );
        rightsideMarker_smallproj_area_bank.push_back( area_rightmarker );

        if( infile.eof() )  break;
    } // the end of reading data file.
}



void PixelOpt::GenerateDifferentNoiseLevelMeasurements( double noise_level ){

    clearMeasurementBank(); //Since this program is going to run under different measurement set, we need first to clear the bank container.

    std::vector<cv::Point2f> markerimg2d;

    for( size_t i=0; i<rod_leftcam_leftmarker_largeproj_bank.size(); i++ ){
        // Project 3D marker features to the 2d plane based on the known camera parameters.
        cv::projectPoints(objPoints, rod_leftcam_leftmarker_largeproj_bank.at(i), t_leftcam_leftmarker_largeproj_bank.at(i), leftCamParam, distCoeff, markerimg2d);
        addGaussianNoise( markerimg2d, noise_level );  // Add Gaussian noise to each projection.

        leftsideMarkerimg2d_largeproj_bank.push_back( markerimg2d );

        // Project 3D marker features to the 2d plane based on the known camera parameters.
        cv::projectPoints( objPoints, rod_rightcam_rightmarker_largeproj_bank.at(i), t_rightcam_rightmarker_largeproj_bank.at(i), rightCamParam, distCoeff, markerimg2d);
        addGaussianNoise( markerimg2d, noise_level );  // Add Gaussian noise to each projection.

        rightsideMarkerimg2d_largeproj_bank.push_back( markerimg2d );
    }

    for( size_t i=0; i<rod_leftcam_leftmarker_smallproj_bank.size(); i++ ){
        // Project 3D marker features to the 2d plane based on the known camera parameters.
        cv::projectPoints(objPoints, rod_leftcam_leftmarker_smallproj_bank.at(i), t_leftcam_leftmarker_smallproj_bank.at(i), leftCamParam, distCoeff, markerimg2d);
        addGaussianNoise( markerimg2d, noise_level );  // Add Gaussian noise to each projection.

        leftsideMarkerimg2d_smallproj_bank.push_back( markerimg2d );

        // Project 3D marker features to the 2d plane based on the known camera parameters.
        cv::projectPoints( objPoints, rod_rightcam_rightmarker_smallproj_bank.at(i), t_rightcam_rightmarker_smallproj_bank.at(i), rightCamParam, distCoeff, markerimg2d);
        addGaussianNoise( markerimg2d, noise_level );  // Add Gaussian noise to each projection.

        rightsideMarkerimg2d_smallproj_bank.push_back( markerimg2d );
    }
}



void PixelOpt::GenerateSetNumberofMeasurements( int setnumber, std::string distribute_flag ){

    clearDataAndVectorContainer();

    std::vector<cv::Point2f> leftmarkerimg2d;
    std::vector<cv::Point2f> rightmarkerimg2d;

    cv::Mat rod_leftcam_leftmarker;
    cv::Mat t_leftcam_leftmarker;
    tf::Matrix3x3 tfrot_leftcam_leftmarker;
    tf::Vector3 tftrans_leftcam_leftmarker;
    tf::Transform T_leftcam_leftmarker;
    cv::Mat cvrot_leftcam_leftmarker;

    cv::Mat rod_rightcam_rightmarker;
    cv::Mat t_rightcam_rightmarker;


    cv::Mat rod_leftcam_leftmarker_toadd;
    cv::Mat t_leftcam_leftmarker_toadd;
    tf::Matrix3x3 tfrot_leftcam_leftmarker_toadd;
    tf::Vector3 tftrans_leftcam_leftmarker_toadd;
    tf::Transform T_leftcam_leftmarker_toadd;
    cv::Mat cvrot_leftcam_leftmarker_toadd;
    double leftmarker_area_toadd, rightmarker_area_to_add;

    std::vector<int> chosenIdset;

    int currentsize;
    bool need_regenerate;
    bool already_in_set;
    int banksize;
    int random_seed;
    int chosenId;
    double translation_diff, rotation_diff;
    double minimum_trans_thred, minimum_rot_thred;
    double translation_smallest;
    double rotation_smallest;

    /******************************************
             large_projection_scattered
    *******************************************/
    if( distribute_flag=="large_projection_scattered"){

        minimum_trans_thred = 0.12;
        minimum_rot_thred = 0.08;

        banksize = rod_leftcam_leftmarker_largeproj_bank.size();
        random_seed = ( std::rand() )%banksize;
        for( int i=1; i<=setnumber; i++){
            if( i==1 )
            {
                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size();

                need_regenerate = true;
                while ( need_regenerate ){
                    random_seed = ( std::rand() )%banksize;

                    need_regenerate = false;
                    for( int j=0; j<currentsize; j++ ){
                        if( random_seed==chosenIdset.at(j) )
                            need_regenerate = true;
                    }   // check whether the generated random_seed id is alreaday in set.

                    rod_leftcam_leftmarker_toadd = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                    t_leftcam_leftmarker_toadd = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                    cv::Rodrigues( rod_leftcam_leftmarker_toadd, cvrot_leftcam_leftmarker_toadd );
                    tfrot_leftcam_leftmarker_toadd = tf::Matrix3x3( cvrot_leftcam_leftmarker_toadd.at<double>(0,0),cvrot_leftcam_leftmarker_toadd.at<double>(0,1),cvrot_leftcam_leftmarker_toadd.at<double>(0,2),
                                                                    cvrot_leftcam_leftmarker_toadd.at<double>(1,0),cvrot_leftcam_leftmarker_toadd.at<double>(1,1),cvrot_leftcam_leftmarker_toadd.at<double>(1,2),
                                                                    cvrot_leftcam_leftmarker_toadd.at<double>(2,0),cvrot_leftcam_leftmarker_toadd.at<double>(2,1),cvrot_leftcam_leftmarker_toadd.at<double>(2,2) );
                    tftrans_leftcam_leftmarker_toadd = tf::Vector3( t_leftcam_leftmarker_toadd.at<double>(0,0), t_leftcam_leftmarker_toadd.at<double>(1,0), t_leftcam_leftmarker_toadd.at<double>(2,0) );
                    T_leftcam_leftmarker_toadd = tf::Transform( tfrot_leftcam_leftmarker_toadd, tftrans_leftcam_leftmarker_toadd );

                    for( int j=0; j<currentsize; j++ ){

                        rod_leftcam_leftmarker = rod_leftcam_leftmarker_est_set.at( j );
                        t_leftcam_leftmarker = t_leftcam_leftmarker_est_set.at( j );
                        cv::Rodrigues( rod_leftcam_leftmarker, cvrot_leftcam_leftmarker );
                        tfrot_leftcam_leftmarker = tf::Matrix3x3( cvrot_leftcam_leftmarker.at<double>(0,0),cvrot_leftcam_leftmarker.at<double>(0,1),cvrot_leftcam_leftmarker.at<double>(0,2),
                                                                  cvrot_leftcam_leftmarker.at<double>(1,0),cvrot_leftcam_leftmarker.at<double>(1,1),cvrot_leftcam_leftmarker.at<double>(1,2),
                                                                  cvrot_leftcam_leftmarker.at<double>(2,0),cvrot_leftcam_leftmarker.at<double>(2,1),cvrot_leftcam_leftmarker.at<double>(2,2) );
                        tftrans_leftcam_leftmarker = tf::Vector3( t_leftcam_leftmarker.at<double>(0,0), t_leftcam_leftmarker.at<double>(1,0), t_leftcam_leftmarker.at<double>(2,0) );
                        T_leftcam_leftmarker = tf::Transform( tfrot_leftcam_leftmarker, tftrans_leftcam_leftmarker );

                        translation_diff = calculateTranslationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );
                        rotation_diff = calculateRotationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );

                        if( (translation_diff<minimum_trans_thred) && (rotation_diff<minimum_rot_thred) ){
                            need_regenerate = true;
                            break;
                        }
                    }
                }  // check whether the generated pose is too close to the poses in the set.

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );

                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
        }
    }  /**************************
         large_projection_scattered
        ***************************/

    if( distribute_flag=="large_projection_clustered" ){

        banksize = rod_leftcam_leftmarker_largeproj_bank.size();

        for( int i=1; i<=setnumber; i++){
            if( i==1 ){
                random_seed = ( std::rand() )%banksize;

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size();
                translation_smallest = 1.0;
                rotation_smallest = 2.0;

                for( int j=0; j<banksize; j++ ){

                    already_in_set = false;

                    for( int k=0; k<currentsize; k++){
                        if( j==chosenIdset.at(k) ){
                            already_in_set = true;
                            break;
                        }
                    }

                    if( already_in_set==true )
                        continue;
                    else{
                        for( int k=0; k<currentsize; k++ ){
                            rod_leftcam_leftmarker = rod_leftcam_leftmarker_est_set.at( k );
                            t_leftcam_leftmarker = t_leftcam_leftmarker_est_set.at( k );

                            cv::Rodrigues( rod_leftcam_leftmarker, cvrot_leftcam_leftmarker );
                            tfrot_leftcam_leftmarker = tf::Matrix3x3( cvrot_leftcam_leftmarker.at<double>(0,0),cvrot_leftcam_leftmarker.at<double>(0,1),cvrot_leftcam_leftmarker.at<double>(0,2),
                                                                      cvrot_leftcam_leftmarker.at<double>(1,0),cvrot_leftcam_leftmarker.at<double>(1,1),cvrot_leftcam_leftmarker.at<double>(1,2),
                                                                      cvrot_leftcam_leftmarker.at<double>(2,0),cvrot_leftcam_leftmarker.at<double>(2,1),cvrot_leftcam_leftmarker.at<double>(2,2) );
                            tftrans_leftcam_leftmarker = tf::Vector3( t_leftcam_leftmarker.at<double>(0,0), t_leftcam_leftmarker.at<double>(1,0), t_leftcam_leftmarker.at<double>(2,0) );
                            T_leftcam_leftmarker = tf::Transform( tfrot_leftcam_leftmarker, tftrans_leftcam_leftmarker );

                            rod_leftcam_leftmarker_toadd = rod_leftcam_leftmarker_largeproj_bank.at( j );
                            t_leftcam_leftmarker_toadd = t_leftcam_leftmarker_largeproj_bank.at( j );
                            cv::Rodrigues( rod_leftcam_leftmarker_toadd, cvrot_leftcam_leftmarker_toadd );
                            tfrot_leftcam_leftmarker_toadd = tf::Matrix3x3( cvrot_leftcam_leftmarker_toadd.at<double>(0,0),cvrot_leftcam_leftmarker_toadd.at<double>(0,1),cvrot_leftcam_leftmarker_toadd.at<double>(0,2),
                                                                            cvrot_leftcam_leftmarker_toadd.at<double>(1,0),cvrot_leftcam_leftmarker_toadd.at<double>(1,1),cvrot_leftcam_leftmarker_toadd.at<double>(1,2),
                                                                            cvrot_leftcam_leftmarker_toadd.at<double>(2,0),cvrot_leftcam_leftmarker_toadd.at<double>(2,1),cvrot_leftcam_leftmarker_toadd.at<double>(2,2) );
                            tftrans_leftcam_leftmarker_toadd = tf::Vector3( t_leftcam_leftmarker_toadd.at<double>(0,0), t_leftcam_leftmarker_toadd.at<double>(1,0), t_leftcam_leftmarker_toadd.at<double>(2,0) );
                            T_leftcam_leftmarker_toadd = tf::Transform( tfrot_leftcam_leftmarker_toadd, tftrans_leftcam_leftmarker_toadd );

                            translation_diff = calculateSum_3dim_trans_error( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );
                            rotation_diff = calculateRotationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );

                            if( translation_diff<=translation_smallest && rotation_diff<=rotation_smallest ){
                                translation_smallest = translation_diff;
                                rotation_smallest = rotation_diff;
                                chosenId = j;
                            }
                        }
                    }
                }
                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( chosenId );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( chosenId );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( chosenId );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( chosenId);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( chosenId );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( chosenId );
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( chosenId );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( chosenId );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( chosenId );
            }
        }
    }  /********************************
            large_projection_clustered
        ********************************/


    /******************************************
             small_projection_scattered
    ******************************************/
    if( distribute_flag=="small_projection_scattered" ){

        minimum_trans_thred = 0.12;
        minimum_rot_thred = 0.08;
        banksize = rod_leftcam_leftmarker_smallproj_bank.size();

        for( int i=1; i<=setnumber; i++){
            if( i==1 )
            {
                random_seed = ( std::rand() )%banksize;

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size();

                need_regenerate = true;
                while ( need_regenerate ){
                    random_seed = ( std::rand() )%banksize;

                    need_regenerate = false;
                    for( int j=0; j<currentsize; j++ ){
                        if( random_seed==chosenIdset.at(j) )
                            need_regenerate = true;
                    }   // check whether the generated random_seed id is alreaday in set.

                    rod_leftcam_leftmarker_toadd = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                    t_leftcam_leftmarker_toadd = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                    cv::Rodrigues( rod_leftcam_leftmarker_toadd, cvrot_leftcam_leftmarker_toadd );
                    tfrot_leftcam_leftmarker_toadd = tf::Matrix3x3( cvrot_leftcam_leftmarker_toadd.at<double>(0,0),cvrot_leftcam_leftmarker_toadd.at<double>(0,1),cvrot_leftcam_leftmarker_toadd.at<double>(0,2),
                                                                    cvrot_leftcam_leftmarker_toadd.at<double>(1,0),cvrot_leftcam_leftmarker_toadd.at<double>(1,1),cvrot_leftcam_leftmarker_toadd.at<double>(1,2),
                                                                    cvrot_leftcam_leftmarker_toadd.at<double>(2,0),cvrot_leftcam_leftmarker_toadd.at<double>(2,1),cvrot_leftcam_leftmarker_toadd.at<double>(2,2) );
                    tftrans_leftcam_leftmarker_toadd = tf::Vector3( t_leftcam_leftmarker_toadd.at<double>(0,0), t_leftcam_leftmarker_toadd.at<double>(1,0), t_leftcam_leftmarker_toadd.at<double>(2,0) );
                    T_leftcam_leftmarker_toadd = tf::Transform( tfrot_leftcam_leftmarker_toadd, tftrans_leftcam_leftmarker_toadd );

                    for( int j=0; j<currentsize; j++ ){

                        rod_leftcam_leftmarker = rod_leftcam_leftmarker_est_set.at( j );
                        t_leftcam_leftmarker = t_leftcam_leftmarker_est_set.at( j );
                        cv::Rodrigues( rod_leftcam_leftmarker, cvrot_leftcam_leftmarker );
                        tfrot_leftcam_leftmarker = tf::Matrix3x3( cvrot_leftcam_leftmarker.at<double>(0,0),cvrot_leftcam_leftmarker.at<double>(0,1),cvrot_leftcam_leftmarker.at<double>(0,2),
                                                                  cvrot_leftcam_leftmarker.at<double>(1,0),cvrot_leftcam_leftmarker.at<double>(1,1),cvrot_leftcam_leftmarker.at<double>(1,2),
                                                                  cvrot_leftcam_leftmarker.at<double>(2,0),cvrot_leftcam_leftmarker.at<double>(2,1),cvrot_leftcam_leftmarker.at<double>(2,2) );
                        tftrans_leftcam_leftmarker = tf::Vector3( t_leftcam_leftmarker.at<double>(0,0), t_leftcam_leftmarker.at<double>(1,0), t_leftcam_leftmarker.at<double>(2,0) );
                        T_leftcam_leftmarker = tf::Transform( tfrot_leftcam_leftmarker, tftrans_leftcam_leftmarker );


                        translation_diff = calculateTranslationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );
                        rotation_diff = calculateRotationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );

                        if( (translation_diff<minimum_trans_thred) && (rotation_diff<minimum_rot_thred) ){
                            need_regenerate = true;
                            break;
                        }
                    }
                }  // check whether the generated pose is too close to the poses in the set.

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
        }
    }   /***********************************
              small_projection_scattered
        ************************************/

    /**************************************************
                  small_projection_clustered
    ***************************************************/
    if( distribute_flag=="small_projection_clustered" ){

        banksize = rod_leftcam_leftmarker_smallproj_bank.size();

        for( int i=1; i<=setnumber; i++){
            if( i==1 ){
                random_seed = ( std::rand() )%banksize;

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size();
                translation_smallest = 1.0;
                rotation_smallest = 2.0;

                for( int j=0; j<banksize; j++ ){

                    already_in_set = false;

                    for( int k=0; k<currentsize; k++){
                        if( j==chosenIdset.at(k) ){
                            already_in_set = true;
                            break;
                        }
                    }

                    if( already_in_set==true )
                        continue;
                    else{
                        for( int k=0; k<currentsize; k++ ){
                            rod_leftcam_leftmarker = rod_leftcam_leftmarker_est_set.at( k );
                            t_leftcam_leftmarker = t_leftcam_leftmarker_est_set.at( k );

                            cv::Rodrigues( rod_leftcam_leftmarker, cvrot_leftcam_leftmarker );
                            tfrot_leftcam_leftmarker = tf::Matrix3x3( cvrot_leftcam_leftmarker.at<double>(0,0),cvrot_leftcam_leftmarker.at<double>(0,1),cvrot_leftcam_leftmarker.at<double>(0,2),
                                                                      cvrot_leftcam_leftmarker.at<double>(1,0),cvrot_leftcam_leftmarker.at<double>(1,1),cvrot_leftcam_leftmarker.at<double>(1,2),
                                                                      cvrot_leftcam_leftmarker.at<double>(2,0),cvrot_leftcam_leftmarker.at<double>(2,1),cvrot_leftcam_leftmarker.at<double>(2,2) );
                            tftrans_leftcam_leftmarker = tf::Vector3( t_leftcam_leftmarker.at<double>(0,0), t_leftcam_leftmarker.at<double>(1,0), t_leftcam_leftmarker.at<double>(2,0) );
                            T_leftcam_leftmarker = tf::Transform( tfrot_leftcam_leftmarker, tftrans_leftcam_leftmarker );

                            rod_leftcam_leftmarker_toadd = rod_leftcam_leftmarker_smallproj_bank.at( j );
                            t_leftcam_leftmarker_toadd = t_leftcam_leftmarker_smallproj_bank.at( j );
                            cv::Rodrigues( rod_leftcam_leftmarker_toadd, cvrot_leftcam_leftmarker_toadd );
                            tfrot_leftcam_leftmarker_toadd = tf::Matrix3x3( cvrot_leftcam_leftmarker_toadd.at<double>(0,0),cvrot_leftcam_leftmarker_toadd.at<double>(0,1),cvrot_leftcam_leftmarker_toadd.at<double>(0,2),
                                                                            cvrot_leftcam_leftmarker_toadd.at<double>(1,0),cvrot_leftcam_leftmarker_toadd.at<double>(1,1),cvrot_leftcam_leftmarker_toadd.at<double>(1,2),
                                                                            cvrot_leftcam_leftmarker_toadd.at<double>(2,0),cvrot_leftcam_leftmarker_toadd.at<double>(2,1),cvrot_leftcam_leftmarker_toadd.at<double>(2,2) );
                            tftrans_leftcam_leftmarker_toadd = tf::Vector3( t_leftcam_leftmarker_toadd.at<double>(0,0), t_leftcam_leftmarker_toadd.at<double>(1,0), t_leftcam_leftmarker_toadd.at<double>(2,0) );
                            T_leftcam_leftmarker_toadd = tf::Transform( tfrot_leftcam_leftmarker_toadd, tftrans_leftcam_leftmarker_toadd );

                            translation_diff = calculateSum_3dim_trans_error( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );
                            rotation_diff = calculateRotationError( T_leftcam_leftmarker, T_leftcam_leftmarker_toadd );

                            if( translation_diff<=translation_smallest && rotation_diff<=rotation_smallest ){
                                translation_smallest = translation_diff;
                                rotation_smallest = rotation_diff;
                                chosenId = j;
                            }
                        }
                    }
                }
                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( chosenId );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( chosenId );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( chosenId );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( chosenId);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( chosenId );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( chosenId );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( chosenId );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( chosenId );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( chosenId );
            }
        }

    }   /**************************************
              small_projection_clustered
        ***************************************/

    /**************************************
                 random_normal
     ***************************************/
    if( distribute_flag=="random_normal" ){
        int onehalf_setnumber, otherhalf_setnumber;

        if( setnumber%2==0 )
            onehalf_setnumber = otherhalf_setnumber = setnumber/2;
        else{
            onehalf_setnumber = ( setnumber + 1 )/2;
            otherhalf_setnumber = setnumber - onehalf_setnumber;
        }

        banksize = rod_leftcam_leftmarker_largeproj_bank.size();

        for( int i=1; i<=onehalf_setnumber; i++){
            if( i==1 )
            {
                random_seed = ( std::rand() )%banksize;
                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size();

                need_regenerate = true;
                while ( need_regenerate ){
                    random_seed = ( std::rand() )%banksize;

                    need_regenerate = false;
                    for( int j=0; j<currentsize; j++ ){
                        if( random_seed==chosenIdset.at(j) )
                            need_regenerate = true;
                    }   // check whether the generated random_seed id is alreaday in set.
                }

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );

                chosenIdset.push_back( random_seed );
            }
        }

        chosenIdset.clear();
        banksize = rod_leftcam_leftmarker_smallproj_bank.size();
        for( int i=1; i<=otherhalf_setnumber; i++){
            if( i==1 )
            {
                random_seed = ( std::rand() )%banksize;
                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }

            if( i>1 ){
                currentsize = rod_leftcam_leftmarker_est_set.size() - onehalf_setnumber;

                need_regenerate = true;
                while ( need_regenerate ){
                    random_seed = ( std::rand() )%banksize;

                    need_regenerate = false;
                    for( int j=0; j<currentsize; j++ ){
                        if( random_seed==chosenIdset.at(j) )
                            need_regenerate = true;
                    }   // check whether the generated random_seed id is alreaday in set.
                }

                rod_leftcam_leftmarker = rod_leftcam_leftmarker_smallproj_bank.at( random_seed );
                t_leftcam_leftmarker = t_leftcam_leftmarker_smallproj_bank.at( random_seed );
                rod_rightcam_rightmarker = rod_rightcam_rightmarker_smallproj_bank.at( random_seed );
                t_rightcam_rightmarker = t_rightcam_rightmarker_smallproj_bank.at( random_seed);

                leftmarkerimg2d = leftsideMarkerimg2d_smallproj_bank.at( random_seed );
                rightmarkerimg2d = rightsideMarkerimg2d_smallproj_bank.at( random_seed );
                leftmarker_area_toadd = leftsideMarker_smallproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_smallproj_area_bank.at( random_seed );

                rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
                t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
                rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
                t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
                leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
                rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
                leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
                rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );
                chosenIdset.push_back( random_seed );
            }
        }
    }   /**************************
              random_normal
        ***************************/
    posepairnumber = rod_rightcam_rightmarker_est_set.size();
    leftsideMarkerSet_smallest_area = 1.0;
    rightsideMarkerSet_smallest_area = 1.0;
    normalizor_largest_left_edge = 0;
    normalizor_largest_right_edge = 0;
    double left_area, right_area;

    for( int i=0; i<leftsideMarker_proj_area_set.size(); i++){
        left_area = leftsideMarker_proj_area_set.at(i);
        right_area = rightsideMarker_proj_area_set.at(i);
        if ( left_area<leftsideMarkerSet_smallest_area )
            leftsideMarkerSet_smallest_area = left_area;
        if ( right_area<rightsideMarkerSet_smallest_area )
            rightsideMarkerSet_smallest_area = right_area;
        if( (left_area*left_area/right_area)>normalizor_largest_right_edge )
            normalizor_largest_right_edge = left_area*left_area/right_area;
        if( (right_area*right_area/left_area)>normalizor_largest_left_edge )
            normalizor_largest_left_edge = right_area*right_area/left_area;
    }
}


void PixelOpt::GenerateSetNumberofMeasurements( int setnumber, double projection_area_thred ){
    clearDataAndVectorContainer();

    std::vector<cv::Point2f> leftmarkerimg2d;
    std::vector<cv::Point2f> rightmarkerimg2d;

    cv::Mat rod_leftcam_leftmarker;
    cv::Mat t_leftcam_leftmarker;

    cv::Mat rod_rightcam_rightmarker;
    cv::Mat t_rightcam_rightmarker;

    double leftmarker_area_toadd, rightmarker_area_to_add;

    std::vector<int> chosenIdset;

    int currentsize;
    bool need_regenerate;
    int banksize;
    int random_seed;

    banksize = rod_leftcam_leftmarker_largeproj_bank.size();

    for( int i=1; i<=setnumber; i++){
        currentsize = rod_leftcam_leftmarker_est_set.size();

        need_regenerate = true;
        while ( need_regenerate ){
            random_seed = ( std::rand() )%banksize;

            need_regenerate = false;
            for( int j=0; j<currentsize; j++ ){
                if( random_seed==chosenIdset.at(j) )
                    need_regenerate = true;
                leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
                rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );
                if( leftmarker_area_toadd<projection_area_thred || rightmarker_area_to_add<projection_area_thred )
                    need_regenerate = true;
            }   // check whether the generated random_seed id is alreaday in set.
        }

        rod_leftcam_leftmarker = rod_leftcam_leftmarker_largeproj_bank.at( random_seed );
        t_leftcam_leftmarker = t_leftcam_leftmarker_largeproj_bank.at( random_seed );
        rod_rightcam_rightmarker = rod_rightcam_rightmarker_largeproj_bank.at( random_seed );
        t_rightcam_rightmarker = t_rightcam_rightmarker_largeproj_bank.at( random_seed);

        leftmarkerimg2d = leftsideMarkerimg2d_largeproj_bank.at( random_seed );
        rightmarkerimg2d = rightsideMarkerimg2d_largeproj_bank.at( random_seed );
        leftmarker_area_toadd = leftsideMarker_largeproj_area_bank.at( random_seed );
        rightmarker_area_to_add = rightsideMarker_largeproj_area_bank.at( random_seed );

        rod_leftcam_leftmarker_est_set.push_back( rod_leftcam_leftmarker );
        t_leftcam_leftmarker_est_set.push_back( t_leftcam_leftmarker );
        rod_rightcam_rightmarker_est_set.push_back( rod_rightcam_rightmarker );
        t_rightcam_rightmarker_est_set.push_back( t_rightcam_rightmarker );
        leftsideMarkerimg2d_set.push_back( leftmarkerimg2d );
        rightsideMarkerimg2d_set.push_back( rightmarkerimg2d );
        leftsideMarker_proj_area_set.push_back( leftmarker_area_toadd );
        rightsideMarker_proj_area_set.push_back( rightmarker_area_to_add );

        chosenIdset.push_back( random_seed );
    }

    posepairnumber = rod_rightcam_rightmarker_est_set.size();
    leftsideMarkerSet_smallest_area = 1.0;
    rightsideMarkerSet_smallest_area = 1.0;
    normalizor_largest_left_edge = 0;
    normalizor_largest_right_edge = 0;
    double left_area, right_area;

    for( int i=0; i<leftsideMarker_proj_area_set.size(); i++){
        left_area = leftsideMarker_proj_area_set.at(i);
        right_area = rightsideMarker_proj_area_set.at(i);
        if ( left_area<leftsideMarkerSet_smallest_area )
            leftsideMarkerSet_smallest_area = left_area;
        if ( right_area<rightsideMarkerSet_smallest_area )
            rightsideMarkerSet_smallest_area = right_area;
        if( (left_area*left_area/right_area)>normalizor_largest_right_edge )
            normalizor_largest_right_edge = left_area*left_area/right_area;
        if( (right_area*right_area/left_area)>normalizor_largest_left_edge )
            normalizor_largest_left_edge = right_area*right_area/left_area;
    }
}



void PixelOpt::CalculateAAiandBBi( std::string bank_or_set ){
    cv::Mat rvec, tvec;
    tf::Transform T_cam_marker;

    cv::Mat cv_cam_marker;
    tf::Matrix3x3 tf_cam_marker;
    tf::Vector3 tf_trans;

    if( bank_or_set=="set" ){
        for(size_t i=0; i<rod_leftcam_leftmarker_est_set.size(); i++){
            cv::solvePnP( objPoints, leftsideMarkerimg2d_set.at(i), leftCamParam, distCoeff, rvec, tvec );
            rvecsAA_set.push_back( rvec.clone() );
            tvecsAA_set.push_back( tvec.clone() );
            //1. This is A in the experiment //2. the results from solvePnP //3. the AA to be the inputs of AAXX = YYBB

            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker,tf_trans );
            T_leftcam_leftmarker_est_set.push_back( T_cam_marker );


            cv::solvePnP( objPoints, rightsideMarkerimg2d_set.at(i), rightCamParam, distCoeff, rvec, tvec );
            rvecsBB_set.push_back( rvec.clone() );
            tvecsBB_set.push_back( tvec.clone() );

            // 1. This is B in the experiment //2. the results from solvePnP) //3.the BB to be the inputs of AAXX = YYBB

            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans);
            T_rightcam_rightmarker_est_set.push_back( T_cam_marker );
        }
    }

    if( bank_or_set=="bank" ){
        for(size_t i=0; i<rod_leftcam_leftmarker_largeproj_bank.size(); i++){

            cv::solvePnP( objPoints, leftsideMarkerimg2d_largeproj_bank.at(i), leftCamParam, distCoeff, rvec, tvec );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker,tf_trans );
            T_leftcam_leftmarker_largeproj_est_bank.push_back( T_cam_marker );


            cv::solvePnP( objPoints, rightsideMarkerimg2d_largeproj_bank.at(i), rightCamParam, distCoeff, rvec, tvec );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans);
            T_rightcam_rightmarker_largeproj_est_bank.push_back( T_cam_marker );
        }

        for(size_t i=0; i<rod_leftcam_leftmarker_smallproj_bank.size(); i++){

            cv::solvePnP( objPoints, leftsideMarkerimg2d_smallproj_bank.at(i), leftCamParam, distCoeff, rvec, tvec );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker,tf_trans );
            T_leftcam_leftmarker_smallproj_est_bank.push_back( T_cam_marker );


            cv::solvePnP( objPoints, rightsideMarkerimg2d_smallproj_bank.at(i), rightCamParam, distCoeff, rvec, tvec );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans);
            T_rightcam_rightmarker_smallproj_est_bank.push_back( T_cam_marker );
        }
    }

}



void PixelOpt::StoreAAiandBBiDiffrtNoiseLevel( std::string largeproj_store_path, std::string smallproj_store_path ){

    std::ofstream outfile;
    outfile.open( largeproj_store_path, std::fstream::app|std::fstream::out);

    for (size_t i=0; i<T_leftcam_leftmarker_largeproj_est_bank.size(); i++){
        outfile<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getX()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getY()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getZ()<<" "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getOrigin().getX()<<" "
               <<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getX()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getY()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(0).getZ()<<" "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getOrigin().getX()<<std::endl
               <<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getX()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getY()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getZ()<<" "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getOrigin().getY()<<" "
               <<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getX()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getY()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(1).getZ()<<" "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getOrigin().getY()<<std::endl
               <<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getX()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getY()<<"  "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getZ()<<" "<<T_leftcam_leftmarker_largeproj_est_bank.at(i).getOrigin().getZ()<<" "
               <<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getX()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getY()<<"  "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getBasis().getRow(2).getZ()<<" "<<T_rightcam_rightmarker_largeproj_est_bank.at(i).getOrigin().getZ()<<std::endl
               <<0<<" "<<0<<" "<<0<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;  // This correspondes to BBi.
    } //Ai and Bi are stored in parallel
    outfile.close();

    outfile.open( smallproj_store_path, std::fstream::app|std::fstream::out);

    for (size_t i=0; i<T_leftcam_leftmarker_smallproj_est_bank.size(); i++){
        outfile<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getX()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getY()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getZ()<<" "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getOrigin().getX()<<" "
               <<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getX()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getY()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(0).getZ()<<" "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getOrigin().getX()<<std::endl
               <<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getX()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getY()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getZ()<<" "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getOrigin().getY()<<" "
               <<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getX()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getY()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(1).getZ()<<" "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getOrigin().getY()<<std::endl
               <<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getX()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getY()<<"  "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getZ()<<" "<<T_leftcam_leftmarker_smallproj_est_bank.at(i).getOrigin().getZ()<<" "
               <<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getX()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getY()<<"  "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getBasis().getRow(2).getZ()<<" "<<T_rightcam_rightmarker_smallproj_est_bank.at(i).getOrigin().getZ()<<std::endl
               <<0<<" "<<0<<" "<<0<<" "<<1<<" "<<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;  // This correspondes to BBi.
    }
    outfile.close();
}



void PixelOpt::PickOutSubset( ){

    double max_trans = 0;
    double max_rot = 0;
    int row_id, column_id;
    int setnumber;
    std::vector<tf::Transform> T_leftcam_leftmarker_temp_set = T_leftcam_leftmarker_est_set;
    int setsize = rvecsAA_set.size();
    std::vector<int> to_delete_id_set;

    tf::Transform T_leftcam_leftmarker_row, T_leftcam_leftmarker_column;

    if ( setsize<=30 ){
        rvecsAA_subset = rvecsAA_set;
        tvecsAA_subset = tvecsAA_set;
        rvecsBB_subset = rvecsBB_set;
        tvecsBB_subset = tvecsBB_set;
    }

    else{
        rvecsAA_subset = rvecsAA_set;
        tvecsAA_subset = tvecsAA_set;
        rvecsBB_subset = rvecsBB_set;
        tvecsBB_subset = tvecsBB_set;

        setnumber = 30;

        double** rotation_distMat = new double* [setsize];
        double** translation_distMat = new double* [setsize];
        double** rot_trans_distMat = new double* [setsize];

        for( int i=0; i <setsize; i++ ){
            rotation_distMat[i] = new double[setsize];
            translation_distMat[i] = new double[setsize];
            rot_trans_distMat[i] = new double[setsize];
        }

        for( int row=0; row<setsize; row++){
            T_leftcam_leftmarker_row = T_leftcam_leftmarker_temp_set.at( row );

            for( int column=0; column<setsize; column++ ){
                T_leftcam_leftmarker_column = T_leftcam_leftmarker_temp_set.at( column );

                rotation_distMat[row][column] = calculateRotationError( T_leftcam_leftmarker_row, T_leftcam_leftmarker_column );
                translation_distMat[row][column] = calculateTranslationError( T_leftcam_leftmarker_row, T_leftcam_leftmarker_column );

                if( rotation_distMat[row][column]>max_rot ) max_rot = rotation_distMat[row][column];
                if( translation_distMat[row][column]>max_trans ) max_trans = translation_distMat[row][column];
            }
        }

        // Normalization all the distance to [0, 1].
        double rot, trans;
        for( int row=0; row<setsize; row++){
            for( int column=0; column<setsize; column++ ){
                rotation_distMat[row][column] = (rotation_distMat[row][column]) / max_rot;
                translation_distMat[row][column] = (translation_distMat[row][column]) / max_trans;
                rot = rotation_distMat[row][column];
                trans = translation_distMat[row][column];
                rot_trans_distMat[row][column] = std::sqrt( rot*rot + trans*trans );
            }
        }

        double rot_trans_dist_min;
        for( int i=0; i<(setsize-setnumber); i++ ){

            rot_trans_dist_min = 1.0;
            for( int row=0; row<(setsize-1); row++ ){
                for( int column=(row+1); column<setsize; column++){
                    if( rot_trans_distMat[row][column]<rot_trans_dist_min ){
                        row_id = row;
                        column_id = column;
                        rot_trans_dist_min = rot_trans_distMat[row][column];
                    }
                }
            }

            if( leftsideMarker_proj_area_set.at(row_id)>leftsideMarker_proj_area_set.at(column_id) ){
                //change the entry related to column_id, whose repro area is smaller.
                for( int i=0; i<setsize; i++ ){
                    rot_trans_distMat[column_id][i] = 1.0;
                    rot_trans_distMat[i][column_id] = 1.0;
                }
                to_delete_id_set.push_back( column_id );
            }
            else{ // change the entry related to row_id, whose repro area is smaller.
                for( int i=0; i<setsize; i++ ){
                    rot_trans_distMat[row_id][i] = 1.0;
                    rot_trans_distMat[i][row_id] = 1.0;
                }
                to_delete_id_set.push_back( row_id );
            }
        }

        int temp = 0; //Rearrange the content in to_delete_id_set, so the number are ranking from larger to smaller.
        for ( int i=0; i<( to_delete_id_set.size()-1 ); i++ ){
            for( int j=0; j<( to_delete_id_set.size()-(i+1) ); j++ ){
                if( to_delete_id_set.at(j)<to_delete_id_set.at(j+1) ){
                    temp = to_delete_id_set.at(j);
                    to_delete_id_set.at(j) = to_delete_id_set.at(j+1);
                    to_delete_id_set.at(j+1) = temp;
                }
            }
        }

        for ( int i=0; i<to_delete_id_set.size(); i++ ){
            rvecsAA_subset.erase( rvecsAA_subset.begin() + to_delete_id_set.at(i) );
            tvecsAA_subset.erase( tvecsAA_subset.begin() + to_delete_id_set.at(i) );
            rvecsBB_subset.erase( rvecsBB_subset.begin() + to_delete_id_set.at(i) );
            tvecsBB_subset.erase( tvecsBB_subset.begin() + to_delete_id_set.at(i) );
        }

        delete [] rotation_distMat;
        delete [] translation_distMat;
        delete [] rot_trans_distMat;
    }
}



cv::Mat PixelOpt::makeMatrixFromVector(cv::Mat rotationVector){   //Calculates the Cross Product Matrix
    double x = rotationVector.at<double>(0, 0);
    double y = rotationVector.at<double>(1, 0);
    double z = rotationVector.at<double>(2, 0);
    cv::Mat makeMatrix = (cv::Mat_<double>(3, 3) << 0, -z, y, z, 0, -x, -y, x, 0);
    return makeMatrix;
}



/*
 * Solve the matrix equation AX = YB
 * @param numberOfImagesToUse [in]: number of the measurements
 * @param rvecsAA_set [in]: the set of rotation vectors of A
 * @param tvecsAA_set [in]: the set of translation vectors of A
 * @param rvecsBB_set [in]: the set of rotation vectors of B
 * @param tvecsBB_set [in]: the set of translation vectors of B
 * @param rotationMatrixX [in] and [out]: initial guess and result of Rx, it will be corrected after the calculation
 * @param tx [out]: Tx
 * @param rotationMatrixY [in] and [out]: initial guess and result of Rx, it will be corrected after the calculation
 * @param ty [out]: Ty
 */
void PixelOpt::optimizeAAXXequalsYYBB( int numberOfImagesToUse, cv::Mat& rotationMatrixX, cv::Mat& rotationMatrixY, cv::Mat& tx, cv::Mat& ty,
                                   const std::vector<cv::Mat>& rvecsAA_set, const std::vector<cv::Mat>& tvecsAA_set,
                                   const std::vector<cv::Mat>& rvecsBB_set, const std::vector<cv::Mat>& tvecsBB_set){

    /***********************************  Calculation of Rotation  *******************************/
    cv::Mat FOverall = cv::Mat_<double>(9 * numberOfImagesToUse, 6);
    cv::Mat QOverall = cv::Mat_<double>(9 * numberOfImagesToUse, 1);

    for (int n = 0; n < numberOfImagesToUse; n++)
    {
        //cv::Mat F = cv::Mat(9, 6, CV_64F);
        cv::Mat F = cv::Mat_<double>(9, 6);
        F = FOverall(cv::Range(9 * n, 9 * n + 9), cv::Range(0, 6));

        cv::Mat r_in = rvecsAA_set.at(n);
        cv::Mat Ra = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Rodrigues(r_in, Ra);

        cv::Mat r_out = rvecsBB_set.at(n);
        cv::Mat Rb = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Rodrigues(r_out, Rb);

        for (int i = 0; i < 3; i++){
            cv::Mat Ax = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            cv::Mat Rxi = rotationMatrixX(cv::Range(0, 3), cv::Range(i, i + 1));
            Ax = makeMatrixFromVector(Rxi);

            cv::Mat F11 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            F11 = F(cv::Range(3 * i, 3 * i + 3), cv::Range(0, 3));
            F11 = -(Ra * Ax);}

        cv::Mat rightOverall;
        rightOverall = rotationMatrixY*Rb;

        for (int i = 0; i < 3; i++)
        {
            cv::Mat F12 = rightOverall(cv::Range(0, 3), cv::Range(i, i + 1));
            cv::Mat F12rod;
            F12rod = F(cv::Range(3 * i, 3 * i + 3), cv::Range(3, 6));
            F12rod = 1 * makeMatrixFromVector(F12);
        }

        //Built Q_Overall for m Images
        cv::Mat vectorQ = (cv::Mat_<double>(9, 1) << 0, 0, 0, 0, 0, 0, 0, 0, 0);;
        vectorQ = QOverall(cv::Range(9 * n, 9 * n + 9), cv::Range(0, 1));

        cv::Mat equationQ;
        equationQ = -Ra*rotationMatrixX + rotationMatrixY*Rb;

        for (int i = 0; i < 3; i++)
        {
            cv::Mat Qa = equationQ(cv::Range(0, 3), cv::Range(i, i + 1));
            cv::Mat QaCache;

            QaCache = vectorQ(cv::Range(3 * i, 3 * i + 3), cv::Range(0, 1));
            QaCache = Qa * 1;
        }
    }

    /**********************Overriding initialRotX and initialRotY***********************/
    cv::Mat FOverallTransp = FOverall.t();

    cv::Mat FOverallTranspMalFOverall = FOverallTransp*FOverall;
    cv::Mat invOfFOverallTranspMalFOverall = FOverallTranspMalFOverall.inv();
    cv::Mat deltaR = invOfFOverallTranspMalFOverall*FOverallTransp*QOverall;

    cv::Mat deltaRx = deltaR(cv::Range(0, 3), cv::Range(0, 1));
    cv::Mat DeltaRxRodrigues;
    cv::Rodrigues(deltaRx, DeltaRxRodrigues);

    cv::Mat deltaRy = deltaR(cv::Range(3, 6), cv::Range(0, 1));
    cv::Mat DeltaRyRodrigues;
    cv::Rodrigues(deltaRy, DeltaRyRodrigues);


    rotationMatrixX = DeltaRxRodrigues*rotationMatrixX;
    rotationMatrixY = DeltaRyRodrigues*rotationMatrixY;

    /*********************Calculation of Translation*************************/
    cv::Mat JOverall = cv::Mat_<double>(3 * numberOfImagesToUse, 6);
    cv::Mat qTranslOverall = cv::Mat_<double>(3 * numberOfImagesToUse, 1);
    cv::Mat tOverall;

    for (int n = 0; n < numberOfImagesToUse; n++)
    {
        cv::Mat J = cv::Mat_<double>(3, 6);
        J = JOverall(cv::Range(3 * n, 3 * n + 3), cv::Range(0, 6));

        cv::Mat qTransl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        qTransl = qTranslOverall(cv::Range(3 * n, 3 * n + 3), cv::Range(0, 1));

        cv::Mat r_in = rvecsAA_set.at(n);
        cv::Mat Ra = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat I = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

        Ra = J(cv::Range(0, 3), cv::Range(0, 3));
        Rodrigues(r_in, Ra);

        cv::Mat minusI;
        minusI = J(cv::Range(0, 3), cv::Range(3, 6));
        minusI = (-1)*I;

        cv::Mat t_A = tvecsAA_set.at(n);
        cv::Mat t_B = tvecsBB_set.at(n);

        cv::Mat qTranslCache;
        qTranslCache = qTransl(cv::Range(0, 3), cv::Range(0, 1));
        qTranslCache = rotationMatrixY*t_B - t_A;
    }

    /***************************** Calculation tOverall*************************************/
    cv::Mat JOverallTransp = JOverall.t();
    cv::Mat JOverallTranspMalJOverall = JOverallTransp*JOverall;
    cv::Mat invOfJOverallTranspMalJOverall = JOverallTranspMalJOverall.inv();
    tOverall = invOfJOverallTranspMalJOverall*JOverallTransp*qTranslOverall;

    tx = tOverall(cv::Range(0, 3), cv::Range(0, 1));
    ty = tOverall(cv::Range(3, 6), cv::Range(0, 1));
}



void PixelOpt::SolveAAXXequalsYYBB( bool use_whole_set ){

    if( use_whole_set){

        int numberOfImagesToUse = rvecsAA_set.size();  // number of the measurement
        int numberOfIterations = 100;      // number of iteration

        cv::Mat rotationMatrixX = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rotationMatrixY = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat tvecX = cv::Mat_<double>(3, 1);
        cv::Mat tvecY = cv::Mat_<double>(3, 1);

        /**************** START OPTIMIZATION: AAXX = YYBB *****************/
        for (int i = 0; i < numberOfIterations; i++)
            optimizeAAXXequalsYYBB(numberOfImagesToUse, rotationMatrixX, rotationMatrixY, tvecX, tvecY, rvecsAA_set, tvecsAA_set, rvecsBB_set, tvecsBB_set );

        tf::Matrix3x3 rot_Matrix;
        rot_Matrix = tf::Matrix3x3(rotationMatrixX.at<double>(0,0),rotationMatrixX.at<double>(0,1),rotationMatrixX.at<double>(0,2),
                                   rotationMatrixX.at<double>(1,0),rotationMatrixX.at<double>(1,1),rotationMatrixX.at<double>(1,2),
                                   rotationMatrixX.at<double>(2,0),rotationMatrixX.at<double>(2,1),rotationMatrixX.at<double>(2,2));
        tf::Vector3 tf_vecX = tf::Vector3(tvecX.at<double>(0,0),tvecX.at<double>(1,0),tvecX.at<double>(2,0));
        T_leftmarker_rightmarker_est_3d = tf::Transform(rot_Matrix, tf_vecX);  // the estimated X from AAXX = YYBB.


        rot_Matrix = tf::Matrix3x3(rotationMatrixY.at<double>(0,0),rotationMatrixY.at<double>(0,1),rotationMatrixY.at<double>(0,2),
                                   rotationMatrixY.at<double>(1,0),rotationMatrixY.at<double>(1,1),rotationMatrixY.at<double>(1,2),
                                   rotationMatrixY.at<double>(2,0),rotationMatrixY.at<double>(2,1),rotationMatrixY.at<double>(2,2));
        tf::Vector3 tf_vecY = tf::Vector3(tvecY.at<double>(0,0),tvecY.at<double>(1,0),tvecY.at<double>(2,0));
        T_leftcam_rightcam_est_3d= tf::Transform(rot_Matrix, tf_vecY);  // the estimated Y from AAXX = YYBB.
    }

    if( !use_whole_set ){

        int numberOfImagesToUse = rvecsAA_subset.size();  // number of the measurement
        int numberOfIterations = 100;      // number of iteration

        cv::Mat rotationMatrixX = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rotationMatrixY = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat tvecX = cv::Mat_<double>(3, 1);
        cv::Mat tvecY = cv::Mat_<double>(3, 1);

        /**************** START OPTIMIZATION: AAXX = YYBB *****************/
        for (int i = 0; i < numberOfIterations; i++)
            optimizeAAXXequalsYYBB(numberOfImagesToUse, rotationMatrixX, rotationMatrixY, tvecX, tvecY, rvecsAA_subset, tvecsAA_subset, rvecsBB_subset, tvecsBB_subset );

        tf::Matrix3x3 rot_Matrix;
        rot_Matrix = tf::Matrix3x3(rotationMatrixX.at<double>(0,0),rotationMatrixX.at<double>(0,1),rotationMatrixX.at<double>(0,2),
                                   rotationMatrixX.at<double>(1,0),rotationMatrixX.at<double>(1,1),rotationMatrixX.at<double>(1,2),
                                   rotationMatrixX.at<double>(2,0),rotationMatrixX.at<double>(2,1),rotationMatrixX.at<double>(2,2));
        tf::Vector3 tf_vecX = tf::Vector3(tvecX.at<double>(0,0),tvecX.at<double>(1,0),tvecX.at<double>(2,0));
        T_leftmarker_rightmarker_est_3d = tf::Transform(rot_Matrix, tf_vecX);  // the estimated X from AAXX = YYBB.


        rot_Matrix = tf::Matrix3x3(rotationMatrixY.at<double>(0,0),rotationMatrixY.at<double>(0,1),rotationMatrixY.at<double>(0,2),
                                   rotationMatrixY.at<double>(1,0),rotationMatrixY.at<double>(1,1),rotationMatrixY.at<double>(1,2),
                                   rotationMatrixY.at<double>(2,0),rotationMatrixY.at<double>(2,1),rotationMatrixY.at<double>(2,2));
        tf::Vector3 tf_vecY = tf::Vector3(tvecY.at<double>(0,0),tvecY.at<double>(1,0),tvecY.at<double>(2,0));
        T_leftcam_rightcam_est_3d= tf::Transform(rot_Matrix, tf_vecY);  // the estimated Y from AAXX = YYBB.
    }
}



void PixelOpt::StartPixelLevelOptimization_without_adaptive_informat( ){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );

    auto externMarkerVertex = buildVertex(X_vertexID, q_cam, t_cam, false);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = buildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    B_vertexID = Y_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.13;
    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<posepairnumber; i++ ){
        area_ratio_rightside_marker = rightsideMarker_proj_area_set.at(i);

        if( area_ratio_rightside_marker>area_ratio_threshold ){
            B_vertexID ++;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = buildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = buildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j );
                optimizer_.addEdge( edge_left );
            }
        }
    }


    A_vertexID = B_vertexID;
    for (int i=0; i<posepairnumber; i++){
        area_ratio_leftside_marker = leftsideMarker_proj_area_set.at(i);

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ());

            auto Ai_vertex = buildVertex( A_vertexID, q_cam, t_cam, true );
            // B_vertexID represents the inverse of B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = buildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j );
                optimizer_.addEdge( edge_right );
            }
        }
    }

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

//    for(int opt_round =0; opt_round<20; opt_round++){
//        externmarkervertex_pointer->setFixed(true);
//        carcamvertex_pointer->setFixed(false);
//        optimizer_.optimize( 10 );

//        externmarkervertex_pointer->setFixed(false);
//        carcamvertex_pointer->setFixed(true);
//        optimizer_.optimize( 10 );
//    }

    optimizer_.optimize( optimization_round );

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_biEdge_1 = tf::Transform(tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)));

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_biEdge_1 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));
}



void PixelOpt::StartPixelLevelOptimization_with_adaptive_informat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );

    auto externMarkerVertex = buildVertex(X_vertexID, q_cam, t_cam, false);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = buildVertex( Y_vertexID, q_cam, t_cam, false );
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    B_vertexID = Y_vertexID; //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    double area_ratio_rightside_marker, area_ratio_leftside_marker;
    double left_edge_infomat_weight, right_edge_infomat_weight;
    double area_ratio_threshold= 0.13;
    for (int i=0; i<posepairnumber; i++){
        area_ratio_rightside_marker = rightsideMarker_proj_area_set.at(i);
        //left_edge_infomat_weight = std::sqrt( area_ratio_rightside_marker/rightsideMarkerSet_smallest_area );
        left_edge_infomat_weight = std::sqrt( (rightsideMarker_proj_area_set.at(i)*rightsideMarker_proj_area_set.at(i)/leftsideMarker_proj_area_set.at(i))/normalizor_largest_left_edge );

        if( area_ratio_rightside_marker>area_ratio_threshold ){
            B_vertexID++;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = buildVertex(B_vertexID, q_cam, t_cam, true); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = buildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j, left_edge_infomat_weight );
                optimizer_.addEdge( edge_left );
            }
        }
    }


    A_vertexID = B_vertexID;
    for (int i=0; i<posepairnumber; i++){
        area_ratio_leftside_marker = leftsideMarker_proj_area_set.at(i);
        //right_edge_infomat_weight = std::sqrt( area_ratio_leftside_marker/leftsideMarkerSet_smallest_area );
        right_edge_infomat_weight = std::sqrt( (leftsideMarker_proj_area_set.at(i)*leftsideMarker_proj_area_set.at(i)/rightsideMarker_proj_area_set.at(i))/normalizor_largest_right_edge );

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;
            calculate4CornerJacobianMatrix( T_leftcam_leftmarker_est_set.at(i), T_rightcam_rightmarker_est_set.at(i) );

//            std::cout<<"A_area: "<<leftsideMarker_proj_area_set.at(i)<<std::endl;
//            std::cout<<"upper_left"<<std::endl;
//            std::cout<<Jacobian_upper_left_A[0][0]<<" "<<Jacobian_upper_left_A[0][1]<<" "<<Jacobian_upper_left_A[0][2]<<" "<<Jacobian_upper_left_A[0][3]<<" "<<Jacobian_upper_left_A[0][4]<<" "<<Jacobian_upper_left_A[0][5]<<std::endl;
//            std::cout<<Jacobian_upper_left_A[1][0]<<" "<<Jacobian_upper_left_A[1][1]<<" "<<Jacobian_upper_left_A[1][2]<<" "<<Jacobian_upper_left_A[1][3]<<" "<<Jacobian_upper_left_A[1][4]<<" "<<Jacobian_upper_left_A[1][5]<<std::endl;
//            std::cout<<"upper_right"<<std::endl;
//            std::cout<<Jacobian_upper_right_A[0][0]<<" "<<Jacobian_upper_right_A[0][1]<<" "<<Jacobian_upper_right_A[0][2]<<" "<<Jacobian_upper_right_A[0][3]<<" "<<Jacobian_upper_right_A[0][4]<<" "<<Jacobian_upper_right_A[0][5]<<std::endl;
//            std::cout<<Jacobian_upper_right_A[1][0]<<" "<<Jacobian_upper_right_A[1][1]<<" "<<Jacobian_upper_right_A[1][2]<<" "<<Jacobian_upper_right_A[1][3]<<" "<<Jacobian_upper_right_A[1][4]<<" "<<Jacobian_upper_right_A[1][5]<<std::endl;
//            std::cout<<"lower_left"<<std::endl;
//            std::cout<<Jacobian_lower_left_A[0][0]<<" "<<Jacobian_lower_left_A[0][1]<<" "<<Jacobian_lower_left_A[0][2]<<" "<<Jacobian_lower_left_A[0][3]<<" "<<Jacobian_lower_left_A[0][4]<<" "<<Jacobian_lower_left_A[0][5]<<std::endl;
//            std::cout<<Jacobian_lower_left_A[1][0]<<" "<<Jacobian_lower_left_A[1][1]<<" "<<Jacobian_lower_left_A[1][2]<<" "<<Jacobian_lower_left_A[1][3]<<" "<<Jacobian_lower_left_A[1][4]<<" "<<Jacobian_lower_left_A[1][5]<<std::endl;
//            std::cout<<"lower_right"<<std::endl;
//            std::cout<<Jacobian_lower_right_A[0][0]<<" "<<Jacobian_lower_right_A[0][1]<<" "<<Jacobian_lower_right_A[0][2]<<" "<<Jacobian_lower_right_A[0][3]<<" "<<Jacobian_lower_right_A[0][4]<<" "<<Jacobian_lower_right_A[0][5]<<std::endl;
//            std::cout<<Jacobian_lower_right_A[1][0]<<" "<<Jacobian_lower_right_A[1][1]<<" "<<Jacobian_lower_right_A[1][2]<<" "<<Jacobian_lower_right_A[1][3]<<" "<<Jacobian_lower_right_A[1][4]<<" "<<Jacobian_lower_right_A[1][5]<<std::endl;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d(T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ() );

            auto Ai_vertex = buildVertex( A_vertexID, q_cam, t_cam, true ); // B_vertexID represents the B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = buildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j, right_edge_infomat_weight );
                optimizer_.addEdge( edge_right );
            }
        }
    }

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

//    for(int opt_round =0; opt_round<20; opt_round++){
//        externmarkervertex_pointer->setFixed(true);
//        carcamvertex_pointer->setFixed(false);
//        optimizer_.optimize( 10 );

//        externmarkervertex_pointer->setFixed(false);
//        carcamvertex_pointer->setFixed(true);
//        optimizer_.optimize(10);
//    }

    optimizer_.optimize( optimization_round );

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_biEdge_2 = tf::Transform(tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)) );

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_biEdge_2 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)) );
}



void PixelOpt::StartPixelLevelOptimization_right_without_adaptive_informat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );

    auto externMarkerVertex = buildVertex(X_vertexID, q_cam, t_cam, false);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = buildVertex( Y_vertexID, q_cam, t_cam, false );
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    double area_ratio_leftside_marker;
    double area_ratio_threshold = 0.13;
    A_vertexID = Y_vertexID;

    for (int i=0; i<posepairnumber; i++){
        area_ratio_leftside_marker = leftsideMarker_proj_area_set.at(i);

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ() );
            auto Ai_vertex = buildVertex( A_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = buildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j );
                optimizer_.addEdge( edge_left );
            }
        }
    }

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

//    for(int opt_round =0; opt_round<20; opt_round++){
//        externmarkervertex_pointer->setFixed(true);
//        carcamvertex_pointer->setFixed(false);
//        optimizer_.optimize( 10 );

//        externmarkervertex_pointer->setFixed(false);
//        carcamvertex_pointer->setFixed(true);
//        optimizer_.optimize(10);
//    }

    optimizer_.optimize( optimization_round );

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_right_1 = tf::Transform( tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)) );


    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_right_1 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)) );
}


void PixelOpt::StartPixelLevelOptimization_right_with_adaptive_informat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );

    auto externMarkerVertex = buildVertex( X_vertexID, q_cam, t_cam, false );
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = buildVertex( Y_vertexID, q_cam, t_cam, false );
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    A_vertexID = Y_vertexID; //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    double area_ratio_leftside_marker, right_edge_infomat_weight;
    double area_ratio_threshold= 0.13;

    for (int i=0; i<posepairnumber; i++){
        area_ratio_leftside_marker = leftsideMarker_proj_area_set.at(i);
        //right_edge_infomat_weight = std::sqrt( area_ratio_leftside_marker/leftsideMarkerSet_smallest_area );
        right_edge_infomat_weight = std::sqrt( (leftsideMarker_proj_area_set.at(i)*leftsideMarker_proj_area_set.at(i)/rightsideMarker_proj_area_set.at(i))/normalizor_largest_right_edge );

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ() );

            auto Ai_vertex = buildVertex( A_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = buildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j, right_edge_infomat_weight );
                optimizer_.addEdge( edge_right );
            }

        }
    }
    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

//    for(int opt_round =0; opt_round<20; opt_round++){
//        externmarkervertex_pointer->setFixed(true);
//        carcamvertex_pointer->setFixed(false);
//        optimizer_.optimize( 10 );

//        externmarkervertex_pointer->setFixed(false);
//        carcamvertex_pointer->setFixed(true);
//        optimizer_.optimize(10);
//    }

    optimizer_.optimize( optimization_round );

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_right_2 = tf::Transform(tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)) );


    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_right_2 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)) );
}



g2o::OptimizableGraph::Vertex* PixelOpt::buildVertex( const int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed )
{
    double initdata[7] = {position_cv[0], position_cv[1], position_cv[2], orientation_cv[0], orientation_cv[1], orientation_cv[2], orientation_cv[3]};

    // set up initial camera estimate
    g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
    v_se3->setEstimateData(initdata);
    v_se3->setId( vertex_id );
    v_se3->setFixed( isFixed );
    return v_se3;
}



g2o::OptimizableGraph::Edge* PixelOpt::buildLeftEdge( const int markerRigId, const int carCamId, const int Bi_id, const int measurementid, const int featureid, const double area_ratio ){

    cv::Point2f& projection = leftsideMarkerimg2d_set.at(measurementid).at(featureid);

    Eigen::Vector2d z_mearsuement(projection.x, projection.y);

    g2o::EyeEyeLeftProjectEdge* e = new g2o::EyeEyeLeftProjectEdge();

    e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(markerRigId)->second);
    e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(carCamId)->second);
    e->vertices()[2] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(Bi_id)->second);

    e->setMeasurement( z_mearsuement );

    Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
    information(0, 0) = area_ratio;
    information(1, 1) = area_ratio;
    e->setInformation(information);

    Eigen::Vector3d singleObjPoint( objPoints[featureid].x, objPoints[featureid].y, objPoints[featureid].z);
    e->initializeKnownParam( singleObjPoint, KMat_leftcam );
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);

    return e;
}



g2o::OptimizableGraph::Edge* PixelOpt::buildRightEdge( const int markerRigId, const int carCamId, const int Ai_id, const int measurementid, const int featureid, const double area_ratio ){

    cv::Point2f& projection = rightsideMarkerimg2d_set.at(measurementid).at(featureid);
    Eigen::Vector2d z_mearsuement(projection.x, projection.y);

    g2o::EyeEyeRightProjectEdge* e = new g2o::EyeEyeRightProjectEdge();

    e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(markerRigId)->second);
    e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(carCamId)->second);
    e->vertices()[2] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertices().find(Ai_id)->second);

    e->setMeasurement( z_mearsuement );

    Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity();
    information(0, 0) = area_ratio;
    information(1, 1) = area_ratio;
    e->setInformation(information);

    Eigen::Vector3d singleObjPoint( objPoints[featureid].x, objPoints[featureid].y, objPoints[featureid].z);
    e->initializeKnownParam( singleObjPoint, KMat_rightcam );
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);

    return e;
}



void PixelOpt::store_error_results_number_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][10], double Error_TX[][10], double Error_RY[][10], double Error_TY[][10]){
    //sort_results_number( Error_RX, Error_TX, Error_RY, Error_TY );

    std::ofstream outfile;
    std::string file_path = store_path + file_flag + ".txt";
    outfile.open( file_path, std::fstream::app|std::fstream::out );

    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RX[i][0]<<" "<<Error_RX[i][1]<<" "<<Error_RX[i][2]<<" "<<Error_RX[i][3]<<" "<<Error_RX[i][4]<<" "
                              <<Error_RX[i][5]<<" "<<Error_RX[i][6]<<" "<<Error_RX[i][7]<<" "<<Error_RX[i][8]<<" "<<Error_RX[i][9]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TX[i][0]<<" "<<Error_TX[i][1]<<" "<<Error_TX[i][2]<<" "<<Error_TX[i][3]<<" "<<Error_TX[i][4]<<" "
                               <<Error_TX[i][5]<<" "<<Error_TX[i][6]<<" "<<Error_TX[i][7]<<" "<<Error_TX[i][8]<<" "<<Error_TX[i][9]<<" "<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RY[i][0]<<" "<<Error_RY[i][1]<<" "<<Error_RY[i][2]<<" "<<Error_RY[i][3]<<" "<<Error_RY[i][4]<<" "
                               <<Error_RY[i][5]<<" "<<Error_RY[i][6]<<" "<<Error_RY[i][7]<<" "<<Error_RY[i][8]<<" "<<Error_RY[i][9]<<" "<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TY[i][0]<<" "<<Error_TY[i][1]<<" "<<Error_TY[i][2]<<" "<<Error_TY[i][3]<<" "<<Error_TY[i][4]<<" "
                               <<Error_TY[i][5]<<" "<<Error_TY[i][6]<<" "<<Error_TY[i][7]<<" "<<Error_TY[i][8]<<" "<<Error_TY[i][9]<<" "<<std::endl;
    }
    outfile.close();
}



void PixelOpt::store_error_results_noise_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][8], double Error_TX[][8], double Error_RY[][8], double Error_TY[][8]){
    //sort_results_noise( Error_RX, Error_TX, Error_RY, Error_TY );

    std::ofstream outfile;
    std::string file_path = store_path + file_flag + ".txt";
    outfile.open( file_path, std::fstream::app|std::fstream::out );

    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RX[i][0]<<" "<<Error_RX[i][1]<<" "<<Error_RX[i][2]<<" "<<Error_RX[i][3]<<" "<<Error_RX[i][4]<<" "
                              <<Error_RX[i][5]<<" "<<Error_RX[i][6]<<" "<<Error_RX[i][7]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TX[i][0]<<" "<<Error_TX[i][1]<<" "<<Error_TX[i][2]<<" "<<Error_TX[i][3]<<" "<<Error_TX[i][4]<<" "
                               <<Error_TX[i][5]<<" "<<Error_TX[i][6]<<" "<<Error_TX[i][7]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RY[i][0]<<" "<<Error_RY[i][1]<<" "<<Error_RY[i][2]<<" "<<Error_RY[i][3]<<" "<<Error_RY[i][4]<<" "
                               <<Error_RY[i][5]<<" "<<Error_RY[i][6]<<" "<<Error_RY[i][7]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TY[i][0]<<" "<<Error_TY[i][1]<<" "<<Error_TY[i][2]<<" "<<Error_TY[i][3]<<" "<<Error_TY[i][4]<<" "
                               <<Error_TY[i][5]<<" "<<Error_TY[i][6]<<" "<<Error_TY[i][7]<<std::endl;
    }
    outfile.close();
}



void PixelOpt::store_error_results_datatype_to_single_file( std::string store_path, std::string file_flag, int iteration_round, double Error_RX[][4], double Error_TX[][4], double Error_RY[][4], double Error_TY[][4]){
    //sort_results_datatype( Error_RX, Error_TX, Error_RY, Error_TY );

    std::ofstream outfile;
    std::string file_path = store_path + file_flag + ".txt";
    outfile.open( file_path, std::fstream::app|std::fstream::out );

    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RX[i][0]<<" "<<Error_RX[i][1]<<" "<<Error_RX[i][2]<<" "<<Error_RX[i][3]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TX[i][0]<<" "<<Error_TX[i][1]<<" "<<Error_TX[i][2]<<" "<<Error_TX[i][3]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_RY[i][0]<<" "<<Error_RY[i][1]<<" "<<Error_RY[i][2]<<" "<<Error_RY[i][3]<<std::endl;
    }
    for(int i=0;i<iteration_round;i++){
        outfile<<Error_TY[i][0]<<" "<<Error_TY[i][1]<<" "<<Error_TY[i][2]<<" "<<Error_TY[i][3]<<std::endl;
    }
    outfile.close();
}



void PixelOpt::StoreCurrentRoundErrorResults_number( int iteration_round, int i ){
    Error_RX_BiEdge1_number[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge1_number[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge1_number[iteration_round][i]= calculateRotationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );
    Error_TY_BiEdge1_number[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );

    Error_RX_BiEdge2_number[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge2_number[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge2_number[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_biEdge_2, T_leftcam_rightcam_true );
    Error_TY_BiEdge2_number[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_2, T_leftcam_rightcam_true );

    Error_RX_RightEdge1_number[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_1 , T_leftmarker_rightmarker_true );
    Error_TX_RightEdge1_number[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_1 , T_leftmarker_rightmarker_true );
    Error_RY_RightEdge1_number[iteration_round][i] = calculateRotationError(T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );
    Error_TY_RightEdge1_number[iteration_round][i]  = calculateTranslationError(T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );

    Error_RX_RightEdge2_number[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_TX_RightEdge2_number[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_RY_RightEdge2_number[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true );
    Error_TY_RightEdge2_number[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true );


    Error_RX_Wang_number[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_TX_Wang_number[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_RY_Wang_number[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
    Error_TY_Wang_number[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
}



void PixelOpt::StoreCurrentRoundErrorResults_noise( int iteration_round, int i ){

    Error_RX_BiEdge1_noise[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge1_noise[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge1_noise[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );
    Error_TY_BiEdge1_noise[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );

    Error_RX_BiEdge2_noise[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge2_noise[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge2_noise[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_biEdge_2, T_leftcam_rightcam_true );
    Error_TY_BiEdge2_noise[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_2 ,T_leftcam_rightcam_true );
    Error_RX_RightEdge1_noise[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_1, T_leftmarker_rightmarker_true );
    Error_TX_RightEdge1_noise[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_1, T_leftmarker_rightmarker_true );
    Error_RY_RightEdge1_noise[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );
    Error_TY_RightEdge1_noise[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );

    Error_RX_RightEdge2_noise[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_TX_RightEdge2_noise[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_RY_RightEdge2_noise[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true );
    Error_TY_RightEdge2_noise[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true) ;

    Error_RX_Wang_noise[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_TX_Wang_noise[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_RY_Wang_noise[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
    Error_TY_Wang_noise[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
}



void PixelOpt::StoreCurrentRoundErrorResults_datatype( int iteration_round, int i ){

    Error_RX_BiEdge1_datatype[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge1_datatype[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_1, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge1_datatype[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );
    Error_TY_BiEdge1_datatype[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_1, T_leftcam_rightcam_true );

    Error_RX_BiEdge2_datatype[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_TX_BiEdge2_datatype[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_biEdge_2, T_leftmarker_rightmarker_true );
    Error_RY_BiEdge2_datatype[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_biEdge_2, T_leftcam_rightcam_true );
    Error_TY_BiEdge2_datatype[iteration_round][i] = calculateTranslationError( T_leftcam_rightcam_est_biEdge_2 ,T_leftcam_rightcam_true );

    Error_RX_RightEdge1_datatype[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_1, T_leftmarker_rightmarker_true );
    Error_TX_RightEdge1_datatype[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_1, T_leftmarker_rightmarker_true );
    Error_RY_RightEdge1_datatype[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );
    Error_TY_RightEdge1_datatype[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_right_1, T_leftcam_rightcam_true );

    Error_RX_RightEdge2_datatype[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_TX_RightEdge2_datatype[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_right_2, T_leftmarker_rightmarker_true );
    Error_RY_RightEdge2_datatype[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true );
    Error_TY_RightEdge2_datatype[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_right_2, T_leftcam_rightcam_true) ;

    Error_RX_Wang_datatype[iteration_round][i] = calculateRotationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_TX_Wang_datatype[iteration_round][i] = calculateTranslationError( T_leftmarker_rightmarker_est_3d, T_leftmarker_rightmarker_true );
    Error_RY_Wang_datatype[iteration_round][i] = calculateRotationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
    Error_TY_Wang_datatype[iteration_round][i]  = calculateTranslationError( T_leftcam_rightcam_est_3d, T_leftcam_rightcam_true );
}



void PixelOpt::StoreAllErrorResultsToFile_number( std::string store_path ){
    store_error_results_number_to_single_file( store_path, "Error_All_BiEdge1_number", 100, Error_RX_BiEdge1_number, Error_TX_BiEdge1_number, Error_RY_BiEdge1_number, Error_TY_BiEdge1_number);
    store_error_results_number_to_single_file( store_path, "Error_All_BiEdge2_number", 100, Error_RX_BiEdge2_number, Error_TX_BiEdge2_number, Error_RY_BiEdge2_number, Error_TY_BiEdge2_number);
    store_error_results_number_to_single_file( store_path, "Error_All_RightEdge1_number", 100, Error_RX_RightEdge1_number, Error_TX_RightEdge1_number, Error_RY_RightEdge1_number, Error_TY_RightEdge1_number);
    store_error_results_number_to_single_file( store_path, "Error_All_RightEdge2_number", 100, Error_RX_RightEdge2_number, Error_TX_RightEdge2_number, Error_RY_RightEdge2_number, Error_TY_RightEdge2_number);
    store_error_results_number_to_single_file( store_path, "Error_All_Wang_number", 100, Error_RX_Wang_number, Error_TX_Wang_number, Error_RY_Wang_number, Error_TY_Wang_number);
}



void PixelOpt::StoreAllErrorResultsToFile_noise( std::string store_path ){
    store_error_results_noise_to_single_file( store_path, "Error_All_BiEdge1_noise", 100, Error_RX_BiEdge1_noise, Error_TX_BiEdge1_noise, Error_RY_BiEdge1_noise, Error_TY_BiEdge1_noise);
    store_error_results_noise_to_single_file( store_path, "Error_All_BiEdge2_noise", 100, Error_RX_BiEdge2_noise, Error_TX_BiEdge2_noise, Error_RY_BiEdge2_noise, Error_TY_BiEdge2_noise);
    store_error_results_noise_to_single_file( store_path, "Error_All_RightEdge1_noise", 100, Error_RX_RightEdge1_noise, Error_TX_RightEdge1_noise, Error_RY_RightEdge1_noise, Error_TY_RightEdge1_noise);
    store_error_results_noise_to_single_file( store_path, "Error_All_RightEdge2_noise", 100, Error_RX_RightEdge2_noise, Error_TX_RightEdge2_noise, Error_RY_RightEdge2_noise, Error_TY_RightEdge2_noise);
    store_error_results_noise_to_single_file( store_path, "Error_All_Wang_noise", 100, Error_RX_Wang_noise, Error_TX_Wang_noise, Error_RY_Wang_noise, Error_TY_Wang_noise);
}



void PixelOpt::StoreAllErrorResultsToFile_datatype( std::string store_path ){
    store_error_results_datatype_to_single_file( store_path, "Error_All_BiEdge1_datatype", 100, Error_RX_BiEdge1_datatype, Error_TX_BiEdge1_datatype, Error_RY_BiEdge1_datatype, Error_TY_BiEdge1_datatype);
    store_error_results_datatype_to_single_file( store_path, "Error_All_BiEdge2_datatype", 100, Error_RX_BiEdge2_datatype, Error_TX_BiEdge2_datatype, Error_RY_BiEdge2_datatype, Error_TY_BiEdge2_datatype);
    store_error_results_datatype_to_single_file( store_path, "Error_All_RightEdge1_datatype", 100, Error_RX_RightEdge1_datatype, Error_TX_RightEdge1_datatype, Error_RY_RightEdge1_datatype, Error_TY_RightEdge1_datatype);
    store_error_results_datatype_to_single_file( store_path, "Error_All_RightEdge2_datatype", 100, Error_RX_RightEdge2_datatype, Error_TX_RightEdge2_datatype, Error_RY_RightEdge2_datatype, Error_TY_RightEdge2_datatype);
    store_error_results_datatype_to_single_file( store_path, "Error_All_Wang_datatype", 100, Error_RX_Wang_datatype, Error_TX_Wang_datatype, Error_RY_Wang_datatype, Error_TY_Wang_datatype);
}



int main( ){

    PixelOpt PixelOpt_obj;
    std::string largeproj_filepath, smallproj_filepath;
    largeproj_filepath = "/home/zaijuan/sim_space/src/pixel_opt/data/true_pose_pair_bank/Big_areas_poseset.txt";
    smallproj_filepath = "/home/zaijuan/sim_space/src/pixel_opt/data/true_pose_pair_bank/Small_areas_poseset.txt";

    PixelOpt_obj.ReadMeasurementsFromFile( largeproj_filepath, smallproj_filepath );
    //Only need to read once.

    std::string bank, set;
    bank = "bank";
    set = "set";
    std::string error_results_storepath;
    std::string pose_distribution_type;
    bool use_whole_set = true;
    bool use_sub_set = false;

    double noise_increase = 0.2;
    double noise_level = 0.05;
    int measurement_number_to_generate;

    //std::string noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj;
    //std::string noised_AisBis_storepath_largeproj_base, noised_AisBis_storepath_smallproj_base;
    //noised_AisBis_storepath_largeproj_base = "/home/zaijuan/sim_space/src/pixel_opt/data/noised_pose_pair_bank/Ai_Bi_Bigsize";
    //noised_AisBis_storepath_smallproj_base = "/home/zaijuan/sim_space/src/pixel_opt/data/noised_pose_pair_bank/Ai_Bi_Smallsize";

    /*********************************************************************************************/
    /************* This part is for calculating AiS and BiS of different noise level *************/
    /*********************************************************************************************/
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level0.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level0.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level1.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level1.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level2.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level2.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level3.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level3.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level4.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level4.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level5.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level5.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level6.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level6.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );

//    noise_level = noise_level + noise_increase;
//    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );
//    PixelOpt_obj.CalculateAAiandBBi( bank );
//    noised_AisBis_storepath_largeproj = noised_AisBis_storepath_largeproj_base + "_Level7.txt";
//    noised_AisBis_storepath_smallproj = noised_AisBis_storepath_smallproj_base + "_Level7.txt";
//    PixelOpt_obj.StoreAAiandBBiDiffrtNoiseLevel( noised_AisBis_storepath_largeproj, noised_AisBis_storepath_smallproj );


    /*******************************************************************************************************/
    /********This loop is for testing the calibration error with the change of measurement numbers. ********/
    /*******************************************************************************************************/
    pose_distribution_type = "random_normal";
    error_results_storepath = "/home/zaijuan/sim_space/src/pixel_opt/data/results/";
    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( 0.8 );

    for( int iteration_round=0; iteration_round<100; iteration_round++ ){

        for(int i=0; i<10; i++){ // 9 rounds: 10, 20, 30, 40, 50, 60, 70, 80, 90, 100,

            measurement_number_to_generate = 10*(i+1);

            PixelOpt_obj.GenerateSetNumberofMeasurements( measurement_number_to_generate, pose_distribution_type );
            PixelOpt_obj.CalculateAAiandBBi( set );
            //PixelOpt_obj.PickOutSubset();
            PixelOpt_obj.SolveAAXXequalsYYBB( use_whole_set );

            PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
            PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

            PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
            PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

            PixelOpt_obj.StoreCurrentRoundErrorResults_number( iteration_round, i );
        }
    }
    PixelOpt_obj.StoreAllErrorResultsToFile_number( error_results_storepath );

    /**********************************************************************************************/
    /******This loop is for testing the calibration error with the change of noise level. *********/
    /**********************************************************************************************/
    pose_distribution_type = "random_normal";
    error_results_storepath = "/home/zaijuan/sim_space/src/pixel_opt/data/results/";

    for( int i=0; i<8; i++){ // 8 rounds:  0.05, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4,
        noise_level = noise_level + noise_increase*i;
        PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( noise_level );

        for( int iteration_round=0; iteration_round<100; iteration_round++ ){
            PixelOpt_obj.GenerateSetNumberofMeasurements( 60, pose_distribution_type );
            PixelOpt_obj.CalculateAAiandBBi( set );
            PixelOpt_obj.PickOutSubset( );
            PixelOpt_obj.SolveAAXXequalsYYBB( use_sub_set );

            PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
            PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

            PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
            PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

            PixelOpt_obj.StoreCurrentRoundErrorResults_noise(iteration_round, i );
        }
    }
    PixelOpt_obj.StoreAllErrorResultsToFile_noise( error_results_storepath );


    /**********************************************************************************************/
    /******This loop is for testing the calibration error with the change of data types.  *********/
    /**********************************************************************************************/
    error_results_storepath = "/home/zaijuan/sim_space/src/pixel_opt/data/results/";
    PixelOpt_obj.GenerateDifferentNoiseLevelMeasurements( 0.8 );
    measurement_number_to_generate = 60;

    for( int iteration_round=0; iteration_round<100; iteration_round++ ){

        for(int i=0; i<=4; i++){

            if( i==0 ){
                pose_distribution_type = "large_projection_scattered";
                PixelOpt_obj.GenerateSetNumberofMeasurements( measurement_number_to_generate, pose_distribution_type );
                PixelOpt_obj.CalculateAAiandBBi( set );
                PixelOpt_obj.PickOutSubset();
                PixelOpt_obj.SolveAAXXequalsYYBB( use_sub_set );

                PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

                PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

                //PixelOpt_obj.SolveAAXXequalsYYBB( use_whole_set );

                PixelOpt_obj.StoreCurrentRoundErrorResults_datatype( iteration_round, i );
            }
            if( i==1 ){
                pose_distribution_type = "large_projection_clustered";
                PixelOpt_obj.GenerateSetNumberofMeasurements( measurement_number_to_generate, pose_distribution_type );
                PixelOpt_obj.CalculateAAiandBBi( set );
                PixelOpt_obj.PickOutSubset();
                PixelOpt_obj.SolveAAXXequalsYYBB( use_sub_set );

                PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

                PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

                PixelOpt_obj.StoreCurrentRoundErrorResults_datatype( iteration_round, i );
            }
            if( i==2 ){
                pose_distribution_type = "small_projection_scattered";

                PixelOpt_obj.GenerateSetNumberofMeasurements( measurement_number_to_generate, pose_distribution_type );
                PixelOpt_obj.CalculateAAiandBBi( set );
                PixelOpt_obj.PickOutSubset();
                PixelOpt_obj.SolveAAXXequalsYYBB( use_sub_set );

                PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

                PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

                PixelOpt_obj.StoreCurrentRoundErrorResults_datatype( iteration_round, i );
            }
            if( i==3 ){
                pose_distribution_type = "small_projection_clustered";

                PixelOpt_obj.GenerateSetNumberofMeasurements( measurement_number_to_generate, pose_distribution_type );
                PixelOpt_obj.CalculateAAiandBBi( set );
                PixelOpt_obj.PickOutSubset();
                PixelOpt_obj.SolveAAXXequalsYYBB( use_sub_set );

                PixelOpt_obj.StartPixelLevelOptimization_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_with_adaptive_informat( );

                PixelOpt_obj.StartPixelLevelOptimization_right_without_adaptive_informat( );
                PixelOpt_obj.StartPixelLevelOptimization_right_with_adaptive_informat( );

                PixelOpt_obj.StoreCurrentRoundErrorResults_datatype( iteration_round, i );
            }
        }
    }
    PixelOpt_obj.StoreAllErrorResultsToFile_datatype( error_results_storepath );

 /*****************************************/
 /*************   THE END  ****************/
 /*****************************************/
}
