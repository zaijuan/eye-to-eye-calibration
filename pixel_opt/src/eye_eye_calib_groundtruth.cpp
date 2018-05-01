
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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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

    std::string leftimagestorepath_base, rightimagestorepath_base;

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

    tf::Transform T_leftcam_rightcam_init;        // for the Y vertex input initialization.

    tf::Transform T_leftcam_rightcam_est_pixel_1;        // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.
    tf::Transform T_leftcam_rightcam_est_pixel_2;        // the estimated Y from final nonlinear pixel-level optimization without building adaptive information.

    std::vector<cv::Point3f> objPoints; // Marker feature 3d points. In this experiment the two marker boards used are the same.

    std::vector< std::vector<cv::Point2f> > leftsideMarkerimg2d_set;
    std::vector< std::vector<cv::Point2f> > rightsideMarkerimg2d_set; // Those two VECTOR of VECTORs store the noise-corrupted 2-d image coordinates of the marker features.

    int valid_posepairnumber;      // This is not fixed.
    int cornerPointNum;  // This is determined by the design.
    double full_area_left, full_area_right;

    std::vector<cv::Mat> rod_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> t_leftcam_leftmarker_est_set;
    std::vector<cv::Mat> rod_rightcam_rightmarker_est_set;
    std::vector<cv::Mat> t_rightcam_rightmarker_est_set;

    std::vector<tf::Transform> T_leftcam_leftmarker_est_set;
    std::vector<tf::Transform> T_rightcam_rightmarker_est_set;
    std::vector<tf::Transform> T_leftmarker_rightmarker_groundtruth_set;

    std::vector< double > projection_area_leftmarker_set;
    std::vector< double > projection_area_rightmarker_set;

    tf::Transform T_marker_target;


    g2o::OptimizableGraph::Vertex* BuildVertex( int vertex_id, cv::Vec4d orientation_cv,cv::Vec3d position_cv, const bool isFixed);
    g2o::OptimizableGraph::Edge* BuildLeftEdge( int markerRigId, int carCamId, int Bi_id, const int measurementid, const int featureid, const double area_ratio=1);
    g2o::OptimizableGraph::Edge* BuildRightEdge( int markerRigId, int carCamId, int Ai_id, const int measurementid, const int featureid, const double area_ratio=1);


public:
    PixelOpt();
    void ReadImagesFromFile( int pose_pair_number );
    void CalculateXisFromFile( std::string filename, int posepairnumber );
    void SolveUnknownY();
    void StartOptimization_withoutAdaptInfoMat();
    void StartOptimization_withAdaptInfoMat();

};



PixelOpt::PixelOpt():leftimagestorepath_base("/home/zaijuan/data_collection/src/data_collector/dataimages/groundtruth/leftimageraw"),
    rightimagestorepath_base("/home/zaijuan/data_collection/src/data_collector/dataimages/groundtruth/rightimageraw"),
    Y_vertexID(0), X_vertexID(1), B_vertexID(1), A_vertexID(1),
    leftcameraparam{{730.6115019147749, 0, 338.0632521377977}, {0, 732.0125341567017, 245.8084690922416}, {0, 0, 1}},
    rightcameraparam{{732.3028793847045, 0, 328.0003018679911}, {0, 734.5061329962873, 275.5781042854172}, {0,  0,  1}},
    leftcam_distCoeffparam{0, 0, 0, 0}, rightcam_distCoeffparam{0, 0, 0, 0},
    //leftcam_distCoeffparam{0.07580611852926922, -0.5875226225843417, -0.001164303045430629, -0.001475905129137468},
    //rightcam_distCoeffparam{0.07841899800114871, -0.4962999717496948, -0.0005858667944341521, -0.001685541678594631},
    valid_posepairnumber( 0 ), cornerPointNum(20){

    //SETUP the 3D features of the marker board.//
    cv::Point3f obj_3dPoint1(      0,        0,   0);    objPoints.push_back(obj_3dPoint1);
    cv::Point3f obj_3dPoint2(      0,   0.0372,   0);    objPoints.push_back(obj_3dPoint2);
    cv::Point3f obj_3dPoint3(      0,   0.0744,   0);    objPoints.push_back(obj_3dPoint3);
    cv::Point3f obj_3dPoint4(      0,   0.1116,   0);    objPoints.push_back(obj_3dPoint4);
    cv::Point3f obj_3dPoint5( 0.0372,        0,   0);    objPoints.push_back(obj_3dPoint5);
    cv::Point3f obj_3dPoint6( 0.0372,   0.0372,   0);    objPoints.push_back(obj_3dPoint6);
    cv::Point3f obj_3dPoint7( 0.0372,   0.0744,   0);    objPoints.push_back(obj_3dPoint7);
    cv::Point3f obj_3dPoint8( 0.0372,   0.1116,   0);    objPoints.push_back(obj_3dPoint8);
    cv::Point3f obj_3dPoint9( 0.0744,        0,   0);    objPoints.push_back(obj_3dPoint9);
    cv::Point3f obj_3dPoint10(0.0744,   0.0372,   0);    objPoints.push_back(obj_3dPoint10);
    cv::Point3f obj_3dPoint11(0.0744,   0.0744,   0);    objPoints.push_back(obj_3dPoint11);
    cv::Point3f obj_3dPoint12(0.0744,   0.1116,   0);    objPoints.push_back(obj_3dPoint12);
    cv::Point3f obj_3dPoint13(0.1116,        0,   0);    objPoints.push_back(obj_3dPoint13);
    cv::Point3f obj_3dPoint14(0.1116,   0.0372,   0);    objPoints.push_back(obj_3dPoint14);
    cv::Point3f obj_3dPoint15(0.1116,   0.0744,   0);    objPoints.push_back(obj_3dPoint15);
    cv::Point3f obj_3dPoint16(0.1116,   0.1116,   0);    objPoints.push_back(obj_3dPoint16);
    cv::Point3f obj_3dPoint17(0.1488,        0,   0);    objPoints.push_back(obj_3dPoint17);
    cv::Point3f obj_3dPoint18(0.1488,   0.0372,   0);    objPoints.push_back(obj_3dPoint18);
    cv::Point3f obj_3dPoint19(0.1488,   0.0744,   0);    objPoints.push_back(obj_3dPoint19);
    cv::Point3f obj_3dPoint20(0.1488,   0.1116,   0);    objPoints.push_back(obj_3dPoint20);

    full_area_left = 2*338*2*245;
    full_area_right = 2*328*2*275;

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

    T_leftcam_rightcam_est_pixel_1.setIdentity();
    T_leftcam_rightcam_est_pixel_2.setIdentity();
    T_marker_target.setIdentity();


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



void PixelOpt::ReadImagesFromFile( int pose_pair_number ){
    cv::Mat image_left, image_right;
    std::string imagepath_left, imagepath_right;
    bool patternfound_left, patternfound_right;
    cv::Size patternsize( 4, 5 );          //number of centers
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

    for( int pose_pair_id=1; pose_pair_id<=pose_pair_number; pose_pair_id++ ){

        /********** For leftside image loading ***********/
        imagepath_left= leftimagestorepath_base + patch::to_string( pose_pair_id ) + ".png";
        image_left = cv::imread( imagepath_left, CV_LOAD_IMAGE_GRAYSCALE );

        patternfound_left = findChessboardCorners( image_left, patternsize, corners_leftmarker);
        if( !patternfound_left ) continue;
        else{
            cv::cornerSubPix(image_left, corners_leftmarker, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

//            cv::undistort( image_left, image_left_, leftCamParam, leftcam_distCoeff );
//            cv::imshow(imagepath_left, image_left_);
//            cv::waitKey(0);

            leftsideMarkerimg2d_set.push_back( corners_leftmarker );
            side_length_1 =  std::sqrt( std::pow(abs(corners_leftmarker.at(0).x - corners_leftmarker.at(3).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(0).y - corners_leftmarker.at(3).y), 2) );
            side_length_2 =  std::sqrt( std::pow(abs(corners_leftmarker.at(3).x - corners_leftmarker.at(19).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(3).y - corners_leftmarker.at(19).y), 2) );
            side_length_diag = std::sqrt( std::pow(abs(corners_leftmarker.at(0).x - corners_leftmarker.at(19).x), 2) +
                                          std::pow(abs(corners_leftmarker.at(0).y - corners_leftmarker.at(19).y), 2) );
            half_perim = (side_length_1 + side_length_2 + side_length_diag)/2;
            area_1 = std::sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag) );

            side_length_3 =  std::sqrt( std::pow(abs(corners_leftmarker.at(19).x - corners_leftmarker.at(16).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(19).y - corners_leftmarker.at(16).y), 2) );
            side_length_4 =  std::sqrt( std::pow(abs(corners_leftmarker.at(16).x - corners_leftmarker.at(0).x), 2) +
                                        std::pow(abs(corners_leftmarker.at(16).y - corners_leftmarker.at(0).y), 2) );
            half_perim = (side_length_3 + side_length_4 + side_length_diag)/2;
            area_2 = std::sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag) );

            area_quadrilateral = area_1 + area_2;
            area_quadrilateral = area_quadrilateral/full_area_left;
            projection_area_leftmarker_set.push_back(area_quadrilateral);

            //  This is for showing the corners on the image.
//            cv::drawChessboardCorners( image_left, patternsize, corners_leftmarker, patternfound_left);
//            cv::imshow(imagepath_left, image_left);
//            cv::waitKey(0);

//            std::cout<<"leftsidemarker corner size: "<<corners_leftmarker.size()<<std::endl;
//            std::cout<<"leftsidemarker "<<corners_leftmarker.at(0).x<<" "<<corners_leftmarker.at(0).y<<std::endl;
//            std::cout<<"leftsidemarker "<<corners_leftmarker.at(3).x<<" "<<corners_leftmarker.at(3).y<<std::endl;
//            std::cout<<"leftsidemarker "<<corners_leftmarker.at(16).x<<" "<<corners_leftmarker.at(16).y<<std::endl;
//            std::cout<<"leftsidemarker "<<corners_leftmarker.at(19).x<<" "<<corners_leftmarker.at(19).y<<std::endl;

            cv::solvePnP( objPoints, corners_leftmarker, leftCamParam, leftcam_distCoeff, rvec, tvec );
//            cv::projectPoints(objPoints, rvec, tvec, leftCamParam, leftcam_distCoeff, corners_leftmarker_repro );
//            double point0_x_dif, point0_y_dif, point3_x_dif, point3_y_dif, point16_x_dif, point16_y_dif, point19_x_dif, point19_y_dif;

//            point0_x_dif = corners_leftmarker.at(0).x-corners_leftmarker_repro.at(0).x;
//            point0_y_dif = corners_leftmarker.at(0).y-corners_leftmarker_repro.at(0).y;
//            point3_x_dif = corners_leftmarker.at(3).x-corners_leftmarker_repro.at(3).x;
//            point3_y_dif = corners_leftmarker.at(3).y-corners_leftmarker_repro.at(3).y;
//            point16_x_dif = corners_leftmarker.at(16).x-corners_leftmarker_repro.at(16).x;
//            point16_y_dif = corners_leftmarker.at(16).y-corners_leftmarker_repro.at(16).y;
//            point19_x_dif = corners_leftmarker.at(19).x-corners_leftmarker_repro.at(19).x;
//            point19_y_dif = corners_leftmarker.at(19).y-corners_leftmarker_repro.at(19).y;
//            std::cout<<"Repro Error:"<<point0_x_dif<<"  "<<point0_y_dif<<std::endl;
//            std::cout<<"Repro Error:"<<point3_x_dif<<"  "<<point3_y_dif<<std::endl;
//            std::cout<<"Repro Error:"<<point16_x_dif<<"  "<<point16_y_dif<<std::endl;
//            std::cout<<"Repro Error:"<<point19_x_dif<<"  "<<point19_y_dif<<std::endl;

            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans );
//            std::cout<<"Left side translation: "<<T_cam_marker.getOrigin().getX()<<" "<<T_cam_marker.getOrigin().getY()<<" "<<T_cam_marker.getOrigin().getZ()<<std::endl;
        }


        /********** For rightside image loading ***********/
        imagepath_right = rightimagestorepath_base + patch::to_string( pose_pair_id ) + ".png";
        image_right = cv::imread( imagepath_right, CV_LOAD_IMAGE_GRAYSCALE );
        patternfound_right = findChessboardCorners( image_right, patternsize, corners_rightmarker);

        if( !patternfound_right ) {
            leftsideMarkerimg2d_set.pop_back();
            projection_area_leftmarker_set.pop_back();
            continue;
        }
        else{
            cv::cornerSubPix(image_right, corners_rightmarker, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            //Now it is saft to push back the leftside pose.
            T_leftcam_leftmarker_est_set.push_back( T_cam_marker );

            rightsideMarkerimg2d_set.push_back( corners_rightmarker );
            side_length_1 =  std::sqrt( std::pow(abs(corners_rightmarker.at(0).x - corners_rightmarker.at(3).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(0).y - corners_rightmarker.at(3).y), 2) );
            side_length_2 =  std::sqrt( std::pow(abs(corners_rightmarker.at(3).x - corners_rightmarker.at(19).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(3).y - corners_rightmarker.at(19).y), 2) );
            side_length_diag = std::sqrt( std::pow(abs(corners_rightmarker.at(0).x - corners_rightmarker.at(19).x), 2) +
                                          std::pow(abs(corners_rightmarker.at(0).y - corners_rightmarker.at(19).y), 2) );
            half_perim = (side_length_1 + side_length_2 + side_length_diag)/2;
            area_1 = std::sqrt( half_perim*(half_perim-side_length_1)*(half_perim-side_length_2)*(half_perim-side_length_diag) );

            side_length_3 =  std::sqrt( std::pow(abs(corners_rightmarker.at(19).x - corners_rightmarker.at(16).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(19).y - corners_rightmarker.at(16).y), 2) );
            side_length_4 =  std::sqrt( std::pow(abs(corners_rightmarker.at(16).x - corners_rightmarker.at(0).x), 2) +
                                        std::pow(abs(corners_rightmarker.at(16).y - corners_rightmarker.at(0).y), 2) );
            half_perim = (side_length_3 + side_length_4 + side_length_diag)/2;
            area_2 = std::sqrt( half_perim*(half_perim-side_length_3)*(half_perim-side_length_4)*(half_perim-side_length_diag) );

            area_quadrilateral = area_1 + area_2;
            area_quadrilateral = area_quadrilateral/full_area_right;
            projection_area_rightmarker_set.push_back(area_quadrilateral);

//            cv::drawChessboardCorners( image_right, patternsize, corners_rightmarker, patternfound_right );
//            cv::imshow(imagepath_right,image_right);
//            cv::waitKey(0);

            cv::solvePnP( objPoints, corners_rightmarker, rightCamParam, rightcam_distCoeff, rvec, tvec );
            cv::Rodrigues( rvec, cv_cam_marker );
            tf_cam_marker = tf::Matrix3x3( cv_cam_marker.at<double>(0,0),cv_cam_marker.at<double>(0,1),cv_cam_marker.at<double>(0,2),
                                           cv_cam_marker.at<double>(1,0),cv_cam_marker.at<double>(1,1),cv_cam_marker.at<double>(1,2),
                                           cv_cam_marker.at<double>(2,0),cv_cam_marker.at<double>(2,1),cv_cam_marker.at<double>(2,2) );
            tf_trans = tf::Vector3( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) );
            T_cam_marker = tf::Transform( tf_cam_marker, tf_trans);
//            std::cout<<T_cam_marker.getOrigin().getX()<<" "<<T_cam_marker.getOrigin().getY()<<" "<<
//                       T_cam_marker.getOrigin().getZ()<<" rightsidemarkertrans"<<std::endl;
            T_rightcam_rightmarker_est_set.push_back( T_cam_marker );
        }
    }
    valid_posepairnumber = T_rightcam_rightmarker_est_set.size();
    std::cout<<"Overall measurement pair size: "<<valid_posepairnumber<<std::endl;
}



void PixelOpt::CalculateXisFromFile( std::string filename, int posepairnumber ){
    std::ifstream infile( filename );

    double a, b, c, d, e, f, g;
    double quat_x, quat_y, quat_z, quat_w, t_x, t_y, t_z;
    int line_index = 0;

    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;
    tf::Transform T_world_lefttarget, T_world_righttarget, T_leftmarker_rightmarker_groundtruth;

    while ( infile >> a >> b >> c >> d >> e >> f >> g ){
        line_index++;
        if( (line_index%2)==1 ){
            quat_x = a; quat_y = b; quat_z = c; quat_w = d;
            t_x = e; t_y = f; t_z = g;
            tf_quat = tf::Quaternion( quat_x, quat_y, quat_z, quat_w );
            tf_trans = tf::Vector3( t_x, t_y, t_z );
            T_world_lefttarget = tf::Transform( tf_quat, tf_trans );
        }
        if( (line_index%2)==0 ){
            quat_x = a; quat_y = b; quat_z = c; quat_w = d;
            t_x = e; t_y = f; t_z = g;
            tf_quat = tf::Quaternion( quat_x, quat_y, quat_z, quat_w );
            tf_trans = tf::Vector3( t_x, t_y, t_z );
            T_world_righttarget = tf::Transform( tf_quat, tf_trans );

            /* Then calculate the ground truth pose between leftmarker and rightmarker.*/
            T_leftmarker_rightmarker_groundtruth = T_marker_target*T_world_lefttarget.inverse()*
                    T_world_righttarget*T_marker_target.inverse();
            T_leftmarker_rightmarker_groundtruth_set.push_back( T_leftmarker_rightmarker_groundtruth );
        }
    } // the end of reading data file.
    infile.close();
}



void PixelOpt::SolveUnknownY(){
    /********** Only use the first group data to get the initial value of Y **********/
    T_leftcam_rightcam_init= T_leftcam_leftmarker_est_set.at(0).inverse()*T_leftmarker_rightmarker_groundtruth_set.at(0)*T_rightcam_rightmarker_est_set.at(0);
}



void PixelOpt::StartOptimization_withoutAdaptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    A_vertexID = Y_vertexID;
    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){

        B_vertexID = A_vertexID +1;
        q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
        t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

        auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
        optimizer_.addVertex( Bi_vertex );

        X_vertexID = B_vertexID + 1;
        q_cam = cv::Vec4d( T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getX(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getY(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getZ(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getW() );
        t_cam = cv::Vec3d( T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getX(), T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getY(), T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getZ() );

        auto Xi_vertex = BuildVertex( X_vertexID, q_cam, t_cam, true );
        // X_vertexID represents the A_i, and it is fixed.
        optimizer_.addVertex( Xi_vertex );

        for(int j=0; j<cornerPointNum; j++){
            auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j );
            optimizer_.addEdge( edge_left );
        }

        A_vertexID = X_vertexID + 1;
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
    /**************************Above are single doubles***********************************/
    /*************************************************************************************/

    optimizer_.initializeOptimization();

    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );
    optimizer_.optimize( 1000 );

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_1 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(unfixed X) without adaptive InfoMat"<<std::endl;
    std::cout<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_1.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_1.getOrigin().getZ()<<std::endl
             <<0<<" "<<0<<" "<<0<<" "<<1<<std::endl;
}



void PixelOpt::StartOptimization_withAdaptInfoMat(){
    // freeing the graph memory
    optimizer_.clear();

    cv::Vec4d q_cam;
    cv::Vec3d t_cam;
    q_cam = cv::Vec4d( T_leftcam_rightcam_init.getRotation().getX(), T_leftcam_rightcam_init.getRotation().getY(), T_leftcam_rightcam_init.getRotation().getZ(), T_leftcam_rightcam_init.getRotation().getW() );
    t_cam = cv::Vec3d( T_leftcam_rightcam_init.getOrigin().getX(), T_leftcam_rightcam_init.getOrigin().getY(), T_leftcam_rightcam_init.getOrigin().getZ() );
    auto carCamVertex = BuildVertex(Y_vertexID, q_cam, t_cam, false);
    // Y_vertexID represents to-be-optimized Y node.
    optimizer_.addVertex( carCamVertex );

    /*************************************************************************************/
    /**************************Below are single doubles***********************************/
    A_vertexID = Y_vertexID;
    double area_ratio_rightside_marker, area_ratio_leftside_marker, area_ratio_threshold = 0.08;
    double left_edge_infomat_weight, right_edge_infomat_weight;

    //Add vertex of As and Bs from the measurement, all those vertex are fixed, since they are optimal and does not need to be further optimized.
    for ( int i=0; i<valid_posepairnumber; i++ ){

        area_ratio_rightside_marker = projection_area_rightmarker_set.at(i);
        left_edge_infomat_weight = std::sqrt( area_ratio_rightside_marker );
        area_ratio_leftside_marker = projection_area_leftmarker_set.at(i);
        right_edge_infomat_weight = std::sqrt( area_ratio_leftside_marker );

        if( area_ratio_rightside_marker>area_ratio_threshold && area_ratio_leftside_marker>area_ratio_threshold ){
            B_vertexID = A_vertexID +1;
            q_cam = cv::Vec4d( T_rightcam_rightmarker_est_set.at(i).getRotation().getX(), T_rightcam_rightmarker_est_set.at(i).getRotation().getY(), T_rightcam_rightmarker_est_set.at(i).getRotation().getZ(), T_rightcam_rightmarker_est_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_rightcam_rightmarker_est_set.at(i).getOrigin().getX(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getY(), T_rightcam_rightmarker_est_set.at(i).getOrigin().getZ() );

            auto Bi_vertex = BuildVertex( B_vertexID, q_cam, t_cam, true ); // A_vertexID represents the A_i, it is fixed.
            optimizer_.addVertex( Bi_vertex );

            X_vertexID = B_vertexID + 1;
            q_cam = cv::Vec4d( T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getX(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getY(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getZ(), T_leftmarker_rightmarker_groundtruth_set.at(i).getRotation().getW() );
            t_cam = cv::Vec3d( T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getX(), T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getY(), T_leftmarker_rightmarker_groundtruth_set.at(i).getOrigin().getZ() );

            auto Xi_vertex = BuildVertex( X_vertexID, q_cam, t_cam, true );
            // X_vertexID represents the A_i, and it is fixed.
            optimizer_.addVertex( Xi_vertex );

            for(int j=0; j<cornerPointNum; j++){
                auto edge_left = BuildLeftEdge( X_vertexID, Y_vertexID, B_vertexID, i, j, left_edge_infomat_weight );
                optimizer_.addEdge( edge_left );
            }

            A_vertexID = X_vertexID + 1;
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

    g2o::HyperGraph::VertexIDMap::iterator v_it_Y = optimizer_.vertices().find( Y_vertexID );
    g2o::VertexSE3 *carcamvertex_pointer = dynamic_cast< g2o::VertexSE3 * > ( v_it_Y->second );
    optimizer_.optimize( 1000 );

    Eigen::Vector3d position_y = carcamvertex_pointer->estimate().translation();
    Eigen::Matrix3d matOrientation_y = carcamvertex_pointer->estimate().rotation();
    Eigen::Quaterniond qOrientation_y( matOrientation_y );
    tf::Quaternion tqOrientation_y( qOrientation_y.x(), qOrientation_y.y(), qOrientation_y.z(), qOrientation_y.w() );
    // The final estimation of Y.
    T_leftcam_rightcam_est_pixel_2 = tf::Transform( tqOrientation_y, tf::Vector3(position_y(0), position_y(1), position_y(2)));

    std::cout<<"Estimated Y(unfixed X) with adaptive InfoMat"<<std::endl;
    std::cout<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(0).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getX()<<std::endl
             <<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(1).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getY()<<std::endl
             <<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getX()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getY()<<"  "<<T_leftcam_rightcam_est_pixel_2.getBasis().getRow(2).getZ()<<"  "<<T_leftcam_rightcam_est_pixel_2.getOrigin().getZ()<<std::endl
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

    Eigen::Vector3d singleObjPoint( objPoints[featureid].x, objPoints[featureid].y, objPoints[featureid].z);
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

    Eigen::Vector3d singleObjPoint( objPoints[featureid].x, objPoints[featureid].y, objPoints[featureid].z);
    e->initializeKnownParam( singleObjPoint, KMat_rightcam );
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);

    return e;
}



int main( ){

    PixelOpt PixelOpt_obj;
    PixelOpt_obj.ReadImagesFromFile( 25 );
    std::string filename;
    filename = "/home/zaijuan/data_collection/src/data_collector/dataimages/groundtruth/lefttarget_righttarget_pose.txt";
    PixelOpt_obj.CalculateXisFromFile( filename, 25 );
    PixelOpt_obj.SolveUnknownY();

    PixelOpt_obj.StartOptimization_withAdaptInfoMat();
    PixelOpt_obj.StartOptimization_withoutAdaptInfoMat();
}
