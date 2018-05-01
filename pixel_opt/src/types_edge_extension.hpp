#pragma once

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/se3quat.h>
#include <cmath>

#include <tf/tf.h>


namespace g2o {

using namespace Eigen;

class G2O_TYPES_SLAM3D_API EyeMarkerUpperProjectEdge : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EyeMarkerUpperProjectEdge();
    EyeMarkerUpperProjectEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* B_i_);
    void initializeKnownParam( Eigen::Vector3d objPoint, Eigen::Matrix3d KMat ){objPoint_ = objPoint; KMat_ = KMat;}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {

        VertexSE3 *inRobot = static_cast<VertexSE3*>(_vertices[0]);
        VertexSE3 *betweenRig_ = static_cast<VertexSE3*>(_vertices[1]);
        VertexSE3 *B_i_ = static_cast<VertexSE3*>(_vertices[2]);

        Eigen::Vector3d p =   inRobot->estimate() * B_i_->estimate() * betweenRig_->estimate() * objPoint_;
        
        Eigen::Vector3d p_ =  KMat_ * p;
        Eigen::Vector2d repro_err_2d;
        repro_err_2d = p_.head<2>()/p_(2);

        _error = repro_err_2d - _measurement;

//        std::cout<<"the upper edge error:  "<<_error(0)<<"   "<<_error(1)<<std::endl;
//        std::cout<<"Measurement(2-d coordinates) from upper edge: "<<_measurement(0)<<"   "<<measurement()(1)<<std::endl;
//        std::cout<<"Reprojection 2-d coordinates from upper edge: "<<repro_err_2d(0)<<"   "<<repro_err_2d(1)<<std::endl;
    }

private:
    Eigen::Vector3d objPoint_;
    Eigen::Matrix3d KMat_;
};


class G2O_TYPES_SLAM3D_API EyeMarkerLowerProjectEdge : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EyeMarkerLowerProjectEdge();
    EyeMarkerLowerProjectEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* A_i_);
    void initializeKnownParam(  Eigen::Vector3d objPoint, Eigen::Matrix3d KMat ){objPoint_ = objPoint; KMat_ = KMat;}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {

        const VertexSE3* inRobot = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* betweenRig_ = static_cast<const VertexSE3*>(_vertices[1]);
        const VertexSE3* A_i_ = static_cast<const VertexSE3*>(_vertices[2]);

        Eigen::Vector3d p = betweenRig_->estimate() * A_i_->estimate() * inRobot->estimate() * objPoint_;
        Eigen::Vector3d p_ = KMat_ * p;

        Eigen::Vector2d repro_err_2d;
        repro_err_2d = p_.head<2>()/p_(2);

        _error = repro_err_2d - _measurement;

//        std::cout<<"the lower edge error:  "<<_error(0)<<"   "<<_error(1)<<std::endl;
//        std::cout<<"Measurement(2-d coordinates) from lower edge: "<<_measurement(0)<<"   "<<measurement()(1)<<std::endl;
//        std::cout<<"Reprojection 2-d coordinates from lower edge: "<<repro_err_2d(0)<<"   "<<repro_err_2d(1)<<std::endl;
    }

private:
    Eigen::Vector3d objPoint_;
    Eigen::Matrix3d KMat_;

};



class G2O_TYPES_SLAM3D_API EyeEyeLeftProjectEdge : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EyeEyeLeftProjectEdge();
    EyeEyeLeftProjectEdge( VertexSE3* markerRig, VertexSE3* carCam, VertexSE3* B_i_ );
    void initializeKnownParam( Eigen::Vector3d objPoint, Eigen::Matrix3d KMat ){objPoint_ = objPoint; KMat_ = KMat;}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {

        VertexSE3 *markerRig = static_cast<VertexSE3*>(_vertices[0]);
        VertexSE3 *carCam = static_cast<VertexSE3*>(_vertices[1]);
        VertexSE3 *B_i_ = static_cast<VertexSE3*>(_vertices[2]);

        Eigen::Vector3d p =   carCam->estimate() * B_i_->estimate() * markerRig->estimate().inverse() * objPoint_;

        Eigen::Vector3d p_ =  KMat_ * p;
        Eigen::Vector2d repro_err_2d;
        repro_err_2d = p_.head<2>()/p_(2);

        _error = repro_err_2d - _measurement;

//        std::cout<<"the left edge error:  "<<_error(0)<<"   "<<_error(1)<<std::endl;
//        std::cout<<"Measurement(2-d coordinates) from left edge: "<<_measurement(0)<<"   "<<measurement()(1)<<std::endl;
//        std::cout<<"Reprojection 2-d coordinates from left edge: "<<repro_err_2d(0)<<"   "<<repro_err_2d(1)<<std::endl;
    }

private:
    Eigen::Vector3d objPoint_;
    Eigen::Matrix3d KMat_;
};


class G2O_TYPES_SLAM3D_API EyeEyeRightProjectEdge : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EyeEyeRightProjectEdge();
    EyeEyeRightProjectEdge( VertexSE3* markerRig, VertexSE3* carCam, VertexSE3* A_i_ );
    void initializeKnownParam(  Eigen::Vector3d objPoint, Eigen::Matrix3d KMat ){ objPoint_ = objPoint; KMat_ = KMat; }
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {

        const VertexSE3* markerRig = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* carCam = static_cast<const VertexSE3*>(_vertices[1]);
        const VertexSE3* A_i_ = static_cast<const VertexSE3*>(_vertices[2]);

        Eigen::Vector3d p = carCam->estimate().inverse() * A_i_->estimate() * markerRig->estimate() * objPoint_;
        Eigen::Vector3d p_ = KMat_ * p;

        Eigen::Vector2d repro_err_2d;
        repro_err_2d = p_.head<2>()/p_(2);

        _error = repro_err_2d - _measurement;

//        std::cout<<"the right edge error:  "<<_error(0)<<"   "<<_error(1)<<std::endl;
//        std::cout<<"Measurement(2-d coordinates) from right edge: "<<_measurement(0)<<"   "<<measurement()(1)<<std::endl;
//        std::cout<<"Reprojection 2-d coordinates from right edge: "<<repro_err_2d(0)<<"   "<<repro_err_2d(1)<<std::endl;
    }

private:
    Eigen::Vector3d objPoint_;
    Eigen::Matrix3d KMat_;

};



class G2O_TYPES_SLAM3D_API TransferedProjectAreaEdge : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TransferedProjectAreaEdge();
    TransferedProjectAreaEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* A_i_);
    void initializeKnownParam( double p_x_max, double p_y_max, Eigen::Vector3d objPoint1, Eigen::Vector3d objPoint2, Eigen::Vector3d objPoint3, Eigen::Vector3d objPoint4, Eigen::Matrix3d KMat )
    {
        p_x_max_ = p_x_max; p_y_max_ = p_y_max;
        objPoint1_ = objPoint1; objPoint2_ = objPoint2;
        objPoint3_ = objPoint3; objPoint4_ = objPoint4;
        KMat_ = KMat;}

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {

        const VertexSE3* inRobot = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* betweenRig_ = static_cast<const VertexSE3*>(_vertices[1]);
        const VertexSE3* A_i_ = static_cast<const VertexSE3*>(_vertices[2]);

        Eigen::Vector3d p1 = betweenRig_->estimate() * A_i_->estimate() * inRobot->estimate() * objPoint1_;
        Eigen::Vector3d p1_ = KMat_ * p1;
        Eigen::Vector2d point1_;
        point1_ = p1_.head<2>()/p1_(2);

        Eigen::Vector3d p2 = betweenRig_->estimate() * A_i_->estimate() * inRobot->estimate() * objPoint2_;
        Eigen::Vector3d p2_ = KMat_ * p2;
        Eigen::Vector2d point2_;
        point2_ = p2_.head<2>()/p2_(2);

        Eigen::Vector3d p3 = betweenRig_->estimate() * A_i_->estimate() * inRobot->estimate() * objPoint3_;
        Eigen::Vector3d p3_ = KMat_ * p3;
        Eigen::Vector2d point3_;
        point3_ = p3_.head<2>()/p3_(2);

        Eigen::Vector3d p4 = betweenRig_->estimate() * A_i_->estimate() * inRobot->estimate() * objPoint4_;
        Eigen::Vector3d p4_ = KMat_ * p4;
        Eigen::Vector2d point4_;
        point4_ = p4_.head<2>()/p4_(2);

        _error(0) = (  p_x_max_*p_y_max_ + 0.5*( point1_(0)*point3_(1) + point2_(0)*point1_(1) + point3_(0)*point4_(1) + point4_(0)*point2_(1) )
                - 0.5*( point1_(1)*point3_(0) + point2_(1)*point1_(0) + point3_(1)*point4_(0) + point4_(1)*point2_(0))  )/(p_x_max_*p_y_max_); //normalize the error to be less than 1;
        _error(1) = 0;
    }

private:
    double p_x_max_;
    double p_y_max_;

    Eigen::Vector3d objPoint1_;
    Eigen::Vector3d objPoint2_;
    Eigen::Vector3d objPoint3_;
    Eigen::Vector3d objPoint4_;
    Eigen::Matrix3d KMat_;
};

} // g2o
