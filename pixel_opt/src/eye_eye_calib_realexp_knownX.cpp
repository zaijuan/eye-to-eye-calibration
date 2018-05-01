
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include "types_edge_extension.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "cv2_eigen.hpp"

#include <g2o/core/sparse_optimizer_terminate_action.h>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


class PixelOpt{

private:
    g2o::SparseOptimizer optimizer_;

    // In this simulation, there are two vertex, which are represented as X_vertex and Y_vertex.
    int X_vertexID;  // vertex index of X_vertex, which represents the transformation from right marker frame to left marker frame.
    int Y_vertexID;  // vertex index of Y_vertex, which represents the transformation from right camera frame to left camera frame.
    int B_vertexID;  // for building left edge, it needs to be initialized during the adding of the vertex.
    int A_vertexID;  // for building right edge, it needs to be initialized during the adding of the vertex.

    double leftcameraparam[3][3];
    double rightcameraparam[3][3];
    double leftcam_distCoeffparam[4][1];
    double rightcam_distCoeffparam[4][1];
    Eigen::Matrix3d KMat_leftcam;   // This redundant definition of camera parameter is for g2o optimization.
    Eigen::Matrix3d KMat_rightcam;  // This redundant definition of camera parameter is for g2o optimization.

    cv::Mat leftcam_distCoeff;
    cv::Mat rightcam_distCoeff;
    cv::Mat leftCamParam;
    cv::Mat rightCamParam;  // The camera parameter.

    tf::Transform T_leftmarker_rightmarker_est_3d;  // the estimated X from AAXX = YYBB;
    tf::Transform T_leftcam_rightcam_est_3d;        // the estimated Y from AAXX = YYBB;

    tf::Transform T_leftmarker_rightmarker_est_pixel;  // the estimated X from final nonlinear pixel-level optimization without building adaptive information..
    tf::Transform T_leftcam_rightcam_est_pixel_2;        // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.
    tf::Transform T_leftcam_rightcam_est_pixel_1;

    tf::Transform T_leftcam_rightcam_est_pixel_withknownX_2;
    tf::Transform T_leftcam_rightcam_est_pixel_withknownX_1;

    tf::Transform T_leftcam_rightcam_init;        // for the Y vertex input initialization.
    tf::Transform T_leftmarker_rightmarker_init;  // for the X vertex input initialization.

    tf::Transform T_leftmarker_rightmarker_groundtruth;
    tf::Transform T_marker_target;

    std::vector<cv::Point3f> objPointsLeft;
    std::vector<cv::Point3f> objPointsRight;
    std::vector<cv::Point3f> obj4Points;

    std::vector< std::vector<cv::Point2f> > leftsideMarkerimg2d_set;
    std::vector< std::vector<cv::Point2f> > rightsideMarkerimg2d_set;

    int valid_posepairnumber;
    int cornerPointNum;
    double full_area_in, full_area_out;

    std::vector<cv::Mat> rod_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> t_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> rod_rightcam_rightmarker_est_set;
    std::vector<cv::Mat> t_rightcam_rightmarker_est_set;

    std::vector<tf::Transform> T_leftcam_leftmarker_est_set;
    std::vector<tf::Transform> T_rightcam_rightmarker_est_set;

    std::vector< double > projection_area_leftmarker_set;
    std::vector< double > projection_area_rightmarker_set;

    std::vector<cv::Mat> rvecsAA_set; // the set of rotation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsAA_set; // the set of translation vectors of AA for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> rvecsBB_set; // the set of rotation vectors of BB for the function (AAXX = YYBB) optimization
    std::vector<cv::Mat> tvecsBB_set; // the set of translation vectors of BB for the function (AAXX = YYBB) optimization


    tf::Vector3 get_R_multiply_T( tf::Matrix3x3 R, tf::Vector3 T );
    void optimizeAAXXequalsYYBB( int numberOfImagesToUse, cv::Mat& rotationMatrixX, cv::Mat& rotationMatrixY, cv::Mat& tx, cv::Mat& ty,
                             const std::vector<cv::Mat>& rvecsAA_set, const std::vector<cv::Mat>& tvecsAA_set,
                             const std::vector<cv::Mat>& rvecsBB_set, const std::vector<cv::Mat>& tvecsBB_set);
    cv::Mat makeMatrixFromVector( cv::Mat rotationVector );
    tf::Quaternion toQuatfromPRY( double pitch, double roll, double yaw );
    g2o::OptimizableGraph::Vertex* BuildVertex( int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed);
    g2o::OptimizableGraph::Edge* BuildLeftEdge( int markerRigId, int carCamId, int Bi_id, const int measurementid, const int featureid, const double area_ratio=1);
    g2o::OptimizableGraph::Edge* BuildRightEdge( int markerRigId, int carCamId, int Ai_id, const int measurementid, const int featureid, const double area_ratio=1);


public:
    PixelOpt();
    void ReadImagesFromFile( );
    void CalculateTrueX( );
    void SolveAAXXequalsYYBB( );
    void StartOptimization_withoutgroundtruth_withAdptInfoMat( );
    void StartOptimization_withoutgroundtruth_withoutAdptInfoMat( );
    void StartOptimization_withgroundtruth_withAdptInfoMat( );
    void StartOptimization_withgroundtruth_withoutAdptInfoMat( );

};


PixelOpt::PixelOpt():
    Y_vertexID(0), X_vertexID(1), B_vertexID(1), A_vertexID(1),
    leftcameraparam{{1759.08510363067, 0, 622.651632662846}, {0, 1759.96628495008, 521.885140498267}, {0, 0, 1}},
    rightcameraparam{{1244.7564038987603, 0, 594.80056145240155}, {0, 1268.2150246291599, 244.355290258134 }, {0, 0, 1}},
    leftcam_distCoeffparam{ 0, 0, 0, 0},
    rightcam_distCoeffparam{ 0, 0, 0, 0},
    //leftcam_distCoeffparam{-0.281848673125568, 0.253969639102471, 0, 0},
    //rightcam_distCoeffparam{ -0.15742329374755687,-0.070813896150477562, 0, 0}, //From Julian's experiment.
    valid_posepairnumber(0),
    cornerPointNum(48){//From Julian's experiment
    //SETUP the 3D features of the marker board.//
    double intrinsic_size = 0.027;
    double extrinsic_size = 0.038;
    cv::Point3f obj_3dPoint_in, obj_3dPoint_out;
    for(int X=0; X<8; X++){
        for(int Y=0; Y<6; Y++){
            obj_3dPoint_in = cv::Point3f( X*intrinsic_size, Y*intrinsic_size, 0 );
            objPointsLeft.push_back( obj_3dPoint_in );
            obj_3dPoint_out = cv::Point3f(X*extrinsic_size, Y*extrinsic_size, 0 );
            objPointsRight.push_back( obj_3dPoint_out );
        }
    }

    full_area_in = (2*622)*(2*521);
    full_area_out = (2*594)*(2*244);

    KMat_leftcam(0,0) = leftcameraparam[0][0];   KMat_leftcam(0,1) = 0;                        KMat_leftcam(0,2) = leftcameraparam[0][2];
    KMat_leftcam(1,0) = 0;                       KMat_leftcam(1,1) = leftcameraparam[1][1];    KMat_leftcam(1,2) = leftcameraparam[1][2];
    KMat_leftcam(2,0) = 0;                       KMat_leftcam(2,1) = 0;                        KMat_leftcam(2,2) = 1;

    KMat_rightcam(0,0) = rightcameraparam[0][0];  KMat_rightcam(0,1) =0;                           KMat_rightcam(0,2) = rightcameraparam[0][2];
    KMat_rightcam(1,0) = 0;                       KMat_rightcam(1,1) = rightcameraparam[1][1];;    KMat_rightcam(1,2) = rightcameraparam[1][2];
    KMat_rightcam(2,0) = 0;                       KMat_rightcam(2,1) = 0;                          KMat_rightcam(2,2) = 1;

    leftcam_distCoeff = cv::Mat(4, 1, CV_32FC1, leftcam_distCoeffparam );
    rightcam_distCoeff = cv::Mat(4, 1, CV_32FC1, rightcam_distCoeffparam );

    leftCamParam = cv::Mat(3, 3, CV_64F, leftcameraparam);
    rightCamParam = cv::Mat(3, 3, CV_64F, rightcameraparam);  // The camera parameter.

    T_leftmarker_rightmarker_est_pixel.setIdentity();
    T_leftcam_rightcam_est_pixel_2.setIdentity();
    T_leftcam_rightcam_est_pixel_1.setIdentity();

    T_leftcam_rightcam_est_pixel_withknownX_2.setIdentity();
    T_leftcam_rightcam_est_pixel_withknownX_1.setIdentity();


    tf::Matrix3x3 rot_marker_target;
    tf::Vector3 trans_marker_target;
    rot_marker_target = tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    trans_marker_target = tf::Vector3(-0.05, 0, 0.0229);
    T_marker_target.setBasis(rot_marker_target);
    T_marker_target.setOrigin(trans_marker_target);

    // ********************SETUP G2O************************* //
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,2> > Block_;
    Block_::LinearSolverType* linearSolver = new g2o::LinearSolverPCG<Block_::PoseMatrixType>();
    Block_* solver_ptr = new Block_( linearSolver );

    // Choosing the method to use by the optimizer
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    optimizer_.setAlgorithm( solver );
    optimizer_.setVerbose( false );
}


tf::Quaternion PixelOpt::toQuatfromPRY( double pitch, double roll, double yaw ){
        // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    double w = cy * cr * cp + sy * sr * sp;
    double x = cy * sr * cp - sy * cr * sp;
    double y = cy * cr * sp + sy * sr * cp;
    double z = sy * cr * cp - cy * sr * sp;

    tf::Quaternion q = tf::Quaternion(x, y, z, w);

    return q;
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



void PixelOpt::ReadImagesFromFile( ){
    cv::Mat image_left, image_right;
    std::string imagepath_left, imagepath_right;
    bool patternfound_left, patternfound_right;
    cv::Size patternsize( 8, 6 );          //number of centers
    std::vector<cv::Point2f> corners_leftmarker, corners_rightmarker; //this will be filled by the detected centers
//    std::vector<cv::Point2f> corners_leftmarker_repro;

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    tf::Transform T_cam_marker;

    cv::Mat cv_cam_marker;
    tf::Matrix3x3 tf_cam_marker;
    tf::Vector3 tf_trans;

    double side_length_1, side_length_2, side_length_diag, half_perim, side_length_3, side_length_4;
    double area_1, area_2, area_quadrilateral;

    for( int pose_pair_id=0; pose_pair_id<=26; pose_pair_id++ ){

        /********** For leftside image loading ***********/
        imagepath_left= "/home/zaijuan/data_collection/src/data_collector/dataimages/Bilder/Inner/" + patch::to_string( pose_pair_id ) + ".tiff";
        image_left = cv::imread( imagepath_left, CV_LOAD_IMAGE_GRAYSCALE );

        patternfound_left = findChessboardCorners( image_left, patternsize, corners_leftmarker);
        if( !patternfound_left ) continue;
        else{
            cv::cornerSubPix(image_left, corners_leftmarker, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

/*            cv::drawChessboardCorners( image_left, patternsize, corners_leftmarker, patternfound_left );
            cv::imshow( imagepath_left, image_left );
            cv::waitKey(0)*/;

            leftsideMarkerimg2d_set.push_back( corners_leftmarker );
            side_length_1 =  std::sqrt( std::pow(abs(corners_leftmarker.at(0).x - corners_leftmarker.at(7).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(0).y - corners_leftmarker.at(7).y), 2) );
            side_length_2 =  std::sqrt( std::pow(abs(corners_leftmarker.at(7).x - corners_leftmarker.at(47).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(7).y - corners_leftmarker.at(47).y), 2) );
            side_length_diag = std::sqrt( std::pow(abs(corners_leftmarker.at(0).x - corners_leftmarker.at(47).x), 2) +
                                          std::pow(abs(corners_leftmarker.at(0).y - corners_leftmarker.at(47).y), 2) );
            half_perim = (side_length_1 + side_length_2 + side_length_diag)/2;
            area_1 = std::sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag) );

            side_length_3 =  std::sqrt( std::pow(abs(corners_leftmarker.at(47).x - corners_leftmarker.at(40).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(47).y - corners_leftmarker.at(40).y), 2) );
            side_length_4 =  std::sqrt( std::pow(abs(corners_leftmarker.at(40).x - corners_leftmarker.at(0).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(40).y - corners_leftmarker.at(0).y), 2) );
            half_perim = (side_length_3 + side_length_4 + side_length_diag)/2;
            area_2 = std::sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag) );

            area_quadrilateral = area_1 + area_2;
            area_quadrilateral = area_quadrilateral/full_area_in;
            projection_area_leftmarker_set.push_back(area_quadrilateral);

            cv::solvePnP( objPointsLeft, corners_leftmarker, leftCamParam, leftcam_distCoeff, rvec, tvec );

            rvecsAA_set.push_back( rvec.clone() );
            tvecsAA_set.push_back( tvec.clone() );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans );
            std::cout<<"Left side translation: "<<T_cam_marker.getOrigin().getX()<<" "<<T_cam_marker.getOrigin().getY()<<" "<<T_cam_marker.getOrigin().getZ()<<std::endl;
        }


        /********** For rightside image loading ***********/
        imagepath_right = "/home/zaijuan/data_collection/src/data_collector/dataimages/Bilder/Outer/" + patch::to_string( pose_pair_id ) + ".png";
        image_right = cv::imread( imagepath_right, CV_LOAD_IMAGE_GRAYSCALE );
        patternfound_right = findChessboardCorners( image_right, patternsize, corners_rightmarker);

        if( !patternfound_right ) {
            leftsideMarkerimg2d_set.pop_back();
            rvecsAA_set.pop_back();
            tvecsAA_set.pop_back();
            projection_area_leftmarker_set.pop_back();
            continue;
        }
        else{
            cv::cornerSubPix(image_right, corners_rightmarker, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            //Now it is saft to push back the leftside pose.
            T_leftcam_leftmarker_est_set.push_back( T_cam_marker );

            rightsideMarkerimg2d_set.push_back( corners_rightmarker );
            side_length_1 =  std::sqrt( std::pow(abs(corners_rightmarker.at(0).x - corners_rightmarker.at(7).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(0).y - corners_rightmarker.at(7).y), 2) );
            side_length_2 =  std::sqrt( std::pow(abs(corners_rightmarker.at(7).x - corners_rightmarker.at(47).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(7).y - corners_rightmarker.at(47).y), 2) );
            side_length_diag = std::sqrt( std::pow(abs(corners_rightmarker.at(0).x - corners_rightmarker.at(47).x), 2) +
                                          std::pow(abs(corners_rightmarker.at(0).y - corners_rightmarker.at(47).y), 2) );
            half_perim = (side_length_1 + side_length_2 + side_length_diag)/2;
            area_1 = std::sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag) );

            side_length_3 =  std::sqrt( std::pow(abs(corners_rightmarker.at(47).x - corners_rightmarker.at(40).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(47).y - corners_rightmarker.at(40).y), 2) );
            side_length_4 =  std::sqrt( std::pow(abs(corners_rightmarker.at(40).x - corners_rightmarker.at(0).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(40).y - corners_rightmarker.at(0).y), 2) );
            half_perim = (side_length_3 + side_length_4 + side_length_diag)/2;
            area_2 = std::sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag) );

            area_quadrilateral = area_1 + area_2;
            area_quadrilateral = area_quadrilateral/full_area_out;
            projection_area_rightmarker_set.push_back(area_quadrilateral);

//            cv::drawChessboardCorners( image_right, patternsize, corners_rightmarker, patternfound_right );
//            cv::imshow(imagepath_right,image_right);
//            cv::waitKey(0);

            cv::solvePnP( objPointsRight, corners_rightmarker, rightCamParam, rightcam_distCoeff, rvec, tvec );
            rvecsBB_set.push_back( rvec.clone() );
            tvecsBB_set.push_back( tvec.clone() );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans);
            std::cout<<"Right side translation: "<<T_cam_marker.getOrigin().getX()<<" "<<T_cam_marker.getOrigin().getY()<<" "<<
                       T_cam_marker.getOrigin().getZ()<<std::endl;
            T_rightcam_rightmarker_est_set.push_back( T_cam_marker );
        }
    }
    valid_posepairnumber = T_rightcam_rightmarker_est_set.size();
    std::cout<<"Overall measurement pair size: "<<valid_posepairnumber<<std::endl;
}



void PixelOpt::CalculateTrueX( ){
    double t_x, t_y, t_z;

    tf::Matrix3x3 tf_mat;
    tf::Vector3 tf_trans;

    t_x = 0.0316111161398567;
    t_y = 1.48710137050006;
    t_z = 0.683009823369446;

    tf_mat= tf::Matrix3x3( 0.999586604577627, 0.0316561869430761, -0.00870968239930957,
                           0.00445642965000881, -0.568410161756883, -0.822722004602589,
                           -0.0277483860635740, 0.822233518579918, -0.568316257168082);
    tf_trans = tf::Vector3( t_x, t_y, t_z );
    T_leftmarker_rightmarker_groundtruth = tf::Transform( tf_mat, tf_trans );
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



void PixelOpt::SolveAAXXequalsYYBB(){

    int numberOfImagesToUse = rvecsAA_set.size();  // number of the measurement
    int numberOfIterations = 500;      // number of iteration

    cv::Mat rotationMatrixX = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rotationMatrixY = (cv::Mat_<double>(3, 3) <<1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat tvecX = cv::Mat_<double>(3, 1);
    cv::Mat tvecY = cv::Mat_<double>(3, 1);

    /**************** START OPTIMIZATION: AAXX = YYBB *****************/
    for (int i = 0; i < numberOfIterations; i++)
        optimizeAAXXequalsYYBB(numberOfImagesToUse, rotationMatrixX, rotationMatrixY, tvecX, tvecY, rvecsAA_set, tvecsAA_set, rvecsBB_set, tvecsBB_set);

    tf::Matrix3x3 rot_Matrix;
    rot_Matrix = tf::Matrix3x3(rotationMatrixX.at<double>(0,0),rotationMatrixX.at<double>(0,1),rotationMatrixX.at<double>(0,2),
                               rotationMatrixX.at<double>(1,0),rotationMatrixX.at<double>(1,1),rotationMatrixX.at<double>(1,2),
                               rotationMatrixX.at<double>(2,0),rotationMatrixX.at<double>(2,1),rotationMatrixX.at<double>(2,2));
    tf::Vector3 tf_vecX = tf::Vector3(tvecX.at<double>(0,0),tvecX.at<double>(1,0),tvecX.at<double>(2,0));
    T_leftmarker_rightmarker_est_3d = tf::Transform(rot_Matrix, tf_vecX);  // the estimated Y from AAXX = YYBB.

    rot_Matrix = tf::Matrix3x3(rotationMatrixY.at<double>(0,0),rotationMatrixY.at<double>(0,1),rotationMatrixY.at<double>(0,2),
                               rotationMatrixY.at<double>(1,0),rotationMatrixY.at<double>(1,1),rotationMatrixY.at<double>(1,2),
                               rotationMatrixY.at<double>(2,0),rotationMatrixY.at<double>(2,1),rotationMatrixY.at<double>(2,2));
    tf::Vector3 tf_vecY = tf::Vector3(tvecY.at<double>(0,0),tvecY.at<double>(1,0),tvecY.at<double>(2,0));

    T_leftcam_rightcam_est_3d= tf::Transform(rot_Matrix, tf_vecY);  // the estimated X from AAXX = YYBB.
    std::cout<<"3D method without knowing the ground truth of X:"<<std::endl
             <<T_leftcam_rightcam_est_3d.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_3d.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_3d.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_3d.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_3d.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_3d.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_3d.getOrigin().getZ()<<std::endl
             <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}



void PixelOpt::StartOptimization_withoutgroundtruth_withAdptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );
    auto externMarkerVertex = BuildVertex(X_vertexID, q_cam, t_cam, false);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    B_vertexID = A_vertexID = X_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.08;
    double left_edge_infomat_weight, right_edge_infomat_weight;

    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){
        area_ratio_rightside_marker = projection_area_rightmarker_set.at(i);
        left_edge_infomat_weight = std::sqrt( area_ratio_rightside_marker );
        area_ratio_leftside_marker = projection_area_leftmarker_set.at(i);
        right_edge_infomat_weight = std::sqrt( area_ratio_leftside_marker );

        if( area_ratio_rightside_marker>area_ratio_threshold && area_ratio_leftside_marker>area_ratio_threshold ){
            B_vertexID = A_vertexID + 1;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j, left_edge_infomat_weight );
                optimizer_.addEdge( edge_left );
            }

            A_vertexID = B_vertexID + 1;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ());

            auto Ai_vertex = BuildVertex( A_vertexID, q_cam, t_cam, true );
            // B_vertexID represents the inverse of B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = BuildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j, right_edge_infomat_weight );
                optimizer_.addEdge( edge_right );
            }
        }        
    }

    /**************************Above are single doubles***********************************/
    /*************************************************************************************/

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

    for(int opt_round =0; opt_round<100; opt_round++){
        externmarkervertex_pointer->setFixed(true);
        carcamvertex_pointer->setFixed(false);
        optimizer_.optimize( 20 );

        externmarkervertex_pointer->setFixed(false);
        carcamvertex_pointer->setFixed(true);
        optimizer_.optimize( 20 );
    }

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_pixel = tf::Transform(tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)));

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );

    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_2 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(with adaptive InfoMat) without knowing the ground truth of X:"<<std::endl
             <<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getZ()<<std::endl
             <<0<<"  "<<0<<"  "<<0<<"  "<<1<<std::endl;
}



void PixelOpt::StartOptimization_withoutgroundtruth_withoutAdptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    q_cam = cv::Vec4d( T_leftmarker_rightmarker_init.getRotation().getX(), T_leftmarker_rightmarker_init.getRotation().getY(), T_leftmarker_rightmarker_init.getRotation().getZ(), T_leftmarker_rightmarker_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_init.getOrigin().getX(), T_leftmarker_rightmarker_init.getOrigin().getY(), T_leftmarker_rightmarker_init.getOrigin().getZ() );
    auto externMarkerVertex = BuildVertex(X_vertexID, q_cam, t_cam, false);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );

    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    B_vertexID = X_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.08;

    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){
        area_ratio_rightside_marker = projection_area_rightmarker_set.at(i);

        if( area_ratio_rightside_marker>area_ratio_threshold ){
            B_vertexID ++;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j);
                optimizer_.addEdge( edge_left );
            }
        }
    }

    A_vertexID = B_vertexID;
    for (int i=0; i<valid_posepairnumber; i++){
        area_ratio_leftside_marker = projection_area_leftmarker_set.at(i);

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ());

            auto Ai_vertex = BuildVertex( A_vertexID, q_cam, t_cam, true );
            // B_vertexID represents the inverse of B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = BuildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j );
                optimizer_.addEdge( edge_right );
            }
        }
    }
    /**************************Above are single doubles***********************************/
    /*************************************************************************************/

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_X = optimizer_.vertices().find( X_vertexID );
    g2o::VertexSE3 *externmarkervertex_pointer = dynamic_cast< g2o::VertexSE3* >( v_it_X->second );
    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );

    for(int opt_round =0; opt_round<100; opt_round++){
        externmarkervertex_pointer->setFixed(true);
        carcamvertex_pointer->setFixed(false);
        optimizer_.optimize( 20 );

        externmarkervertex_pointer->setFixed(false);
        carcamvertex_pointer->setFixed(true);
        optimizer_.optimize( 20 );
    }

    Eigen::Vector3d position_x = externmarkervertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_x = externmarkervertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_x( matOrientation_x );
    tf::Quaternion tqOrientation_x( qOrientation_x.x(), qOrientation_x.y(), qOrientation_x.z(), qOrientation_x.w() );

    // The final estimation of X.
    T_leftmarker_rightmarker_est_pixel = tf::Transform(tqOrientation_x, tf::Vector3(position_x(0), position_x(1), position_x(2)));

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );

    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_1 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(without adaptive InfoMat) without knowing the ground truth of X:"<<std::endl
             <<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getZ()<<std::endl
             <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}



void PixelOpt::StartOptimization_withgroundtruth_withAdptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    q_cam = cv::Vec4d( T_leftmarker_rightmarker_groundtruth.getRotation().getX(), T_leftmarker_rightmarker_groundtruth.getRotation().getY(), T_leftmarker_rightmarker_groundtruth.getRotation().getZ(), T_leftmarker_rightmarker_groundtruth.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_groundtruth.getOrigin().getX(), T_leftmarker_rightmarker_groundtruth.getOrigin().getY(), T_leftmarker_rightmarker_groundtruth.getOrigin().getZ() );

    auto externMarkerVertex = BuildVertex(X_vertexID, q_cam, t_cam, true);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );


    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    B_vertexID = X_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.08;
    double left_edge_infomat_weight, right_edge_infomat_weight;
    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){
        area_ratio_rightside_marker = projection_area_rightmarker_set.at(i);
        left_edge_infomat_weight = std::sqrt( area_ratio_rightside_marker );

        if( area_ratio_rightside_marker>area_ratio_threshold ){
            B_vertexID ++;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j, left_edge_infomat_weight );
                optimizer_.addEdge( edge_left );
            }
        }
    }

    A_vertexID = B_vertexID;
    for (int i=0; i<valid_posepairnumber; i++){
        area_ratio_leftside_marker = projection_area_leftmarker_set.at(i);
        right_edge_infomat_weight = std::sqrt( area_ratio_leftside_marker );

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ());

            auto Ai_vertex = BuildVertex( A_vertexID, q_cam, t_cam, true );
            // B_vertexID represents the inverse of B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = BuildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j, right_edge_infomat_weight );
                optimizer_.addEdge( edge_right );
            }
        }
    }
    /**************************Above are single doubles***********************************/
    /*************************************************************************************/

    optimizer_.initializeOptimization();
    optimizer_.optimize( 1000 );

    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );
    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );

    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_withknownX_2 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(with adaptive InfoMat) with ground truth of X:"<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_2.getOrigin().getZ()<<std::endl
             <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}



void PixelOpt::StartOptimization_withgroundtruth_withoutAdptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    T_leftmarker_rightmarker_init = T_leftmarker_rightmarker_est_3d;
    T_leftcam_rightcam_init = T_leftcam_rightcam_est_3d;

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    q_cam = cv::Vec4d( T_leftmarker_rightmarker_groundtruth.getRotation().getX(), T_leftmarker_rightmarker_groundtruth.getRotation().getY(), T_leftmarker_rightmarker_groundtruth.getRotation().getZ(), T_leftmarker_rightmarker_groundtruth.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftmarker_rightmarker_groundtruth.getOrigin().getX(), T_leftmarker_rightmarker_groundtruth.getOrigin().getY(), T_leftmarker_rightmarker_groundtruth.getOrigin().getZ() );

    auto externMarkerVertex = BuildVertex(X_vertexID, q_cam, t_cam, true);
    // X_vertexID represents to-be-optimized X node.
    optimizer_.addVertex( externMarkerVertex );



    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    B_vertexID = X_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.08;

    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){
        area_ratio_rightside_marker = projection_area_rightmarker_set.at(i);

        if( area_ratio_rightside_marker>area_ratio_threshold ){
            B_vertexID ++;

            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j );
                optimizer_.addEdge( edge_left );
            }
        }
    }

    A_vertexID = B_vertexID;
    for (int i=0; i<valid_posepairnumber; i++){
        area_ratio_leftside_marker = projection_area_leftmarker_set.at(i);

        if( area_ratio_leftside_marker>area_ratio_threshold ){
            A_vertexID++;

            q_cam = cv::Vec4d( T_leftcam_leftmarker_est_set.at(i).getRotation().getX(), T_leftcam_leftmarker_est_set.at(i).getRotation().getY(), T_leftcam_leftmarker_est_set.at(i).getRotation().getZ(), T_leftcam_leftmarker_est_set.at(i).getRotation().getW());
            t_cam = cv::Vec3d( T_leftcam_leftmarker_est_set.at(i).getOrigin().getX(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getY(), T_leftcam_leftmarker_est_set.at(i).getOrigin().getZ());

            auto Ai_vertex = BuildVertex( A_vertexID, q_cam, t_cam, true );
            // B_vertexID represents the inverse of B_i, it is fixed.
            optimizer_.addVertex( Ai_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_right = BuildRightEdge( X_vertexID, Y_vertexID, A_vertexID, i, j );
                optimizer_.addEdge( edge_right );
            }
        }
    }
    /**************************Above are single doubles***********************************/
    /*************************************************************************************/

    optimizer_.initializeOptimization();
    optimizer_.optimize( 1000 );

    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );
    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );

    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_withknownX_1 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(without adaptive InfoMat) with ground truth of X:"<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_withknownX_1.getOrigin().getZ()<<std::endl
             <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}



g2o::OptimizableGraph::Vertex* PixelOpt::BuildVertex( int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed )
{
    double initdata[7] = {position_cv[0], position_cv[1], position_cv[2], orientation_cv[0], orientation_cv[1], orientation_cv[2], orientation_cv[3]};

    // set up initial camera estimate
    g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
    v_se3->setEstimateData(initdata);
    v_se3->setId( vertex_id );
    v_se3->setFixed( isFixed );

    return v_se3;
}



g2o::OptimizableGraph::Edge* PixelOpt::BuildLeftEdge( int markerRigId, int carCamId, int Bi_id, const int measurementid, const int featureid, const double area_ratio ){

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

    Eigen::Vector3d singleObjPoint( objPointsRight[featureid].x, objPointsRight[featureid].y, objPointsRight[featureid].z);
    e->initializeKnownParam( singleObjPoint, KMat_leftcam );
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);

    return e;
}



g2o::OptimizableGraph::Edge* PixelOpt::BuildRightEdge( int markerRigId, int carCamId, int Ai_id, const int measurementid, const int featureid, const double area_ratio ){

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

    Eigen::Vector3d singleObjPoint( objPointsLeft[featureid].x, objPointsLeft[featureid].y, objPointsLeft[featureid].z);
    e->initializeKnownParam( singleObjPoint, KMat_rightcam );
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);

    return e;
}



int main( ){

    PixelOpt PixelOpt_obj;

    PixelOpt_obj.ReadImagesFromFile( );
    PixelOpt_obj.SolveAAXXequalsYYBB();
    PixelOpt_obj.StartOptimization_withoutgroundtruth_withoutAdptInfoMat();
    PixelOpt_obj.StartOptimization_withoutgroundtruth_withAdptInfoMat();

    PixelOpt_obj.CalculateTrueX( );
    PixelOpt_obj.StartOptimization_withgroundtruth_withoutAdptInfoMat();
    PixelOpt_obj.StartOptimization_withgroundtruth_withAdptInfoMat();
}
