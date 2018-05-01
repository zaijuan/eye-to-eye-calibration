#include "types_edge_extension.hpp"

namespace g2o {

using namespace std;

EyeMarkerUpperProjectEdge::EyeMarkerUpperProjectEdge() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    //information().setIdentity();
}

EyeMarkerUpperProjectEdge::EyeMarkerUpperProjectEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* B_i_) : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    _vertices[0] = inRobot;
    _vertices[1] = betweenRig_;
    _vertices[2] = B_i_;

}

bool EyeMarkerUpperProjectEdge::read(std::istream& is)
{
    // measured keypoint
    for (int i=0; i<2; i++)
        is>>_measurement[i];
    setMeasurement(_measurement);
    information().setIdentity();
    return true;
}

bool EyeMarkerUpperProjectEdge::write(std::ostream& os) const
{
    for (int i=0; i<2; i++)
        os<<measurement()[i]<<" ";
    return os.good();
}



EyeMarkerLowerProjectEdge::EyeMarkerLowerProjectEdge() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    //information().setIdentity();
}

EyeMarkerLowerProjectEdge::EyeMarkerLowerProjectEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* A_i_) : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    _vertices[0] = inRobot;
    _vertices[1] = betweenRig_;
    _vertices[2] = A_i_;

}

bool EyeMarkerLowerProjectEdge::read(std::istream& is)
{
    // measured keypoint
    for (int i=0; i<2; i++)
        is>>_measurement[i];
    setMeasurement(_measurement);
    information().setIdentity();
    return true;
}

bool EyeMarkerLowerProjectEdge::write(std::ostream& os) const
{
    for (int i=0; i<2; i++)
        os<< measurement()[i] << " ";
    return os.good();
}



EyeEyeLeftProjectEdge::EyeEyeLeftProjectEdge() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    //information().setIdentity();
}

EyeEyeLeftProjectEdge::EyeEyeLeftProjectEdge( VertexSE3* markerRig, VertexSE3* carCam, VertexSE3* B_i_ ) : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    _vertices[0] = markerRig;
    _vertices[1] = carCam;
    _vertices[2] = B_i_;

}

bool EyeEyeLeftProjectEdge::read(std::istream& is)
{
    // measured keypoint
    for (int i=0; i<2; i++)
        is>>_measurement[i];
    setMeasurement(_measurement);
    information().setIdentity();
    return true;
}

bool EyeEyeLeftProjectEdge::write(std::ostream& os) const
{
    for (int i=0; i<2; i++)
        os<<measurement()[i]<<" ";
    return os.good();
}



EyeEyeRightProjectEdge::EyeEyeRightProjectEdge() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    //information().setIdentity();
}

EyeEyeRightProjectEdge::EyeEyeRightProjectEdge( VertexSE3* markerRig, VertexSE3* carCam, VertexSE3* A_i_ ) : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    _vertices[0] = markerRig;
    _vertices[1] = carCam;
    _vertices[2] = A_i_;

}

bool EyeEyeRightProjectEdge::read(std::istream& is)
{
    // measured keypoint
    for (int i=0; i<2; i++)
        is>>_measurement[i];
    setMeasurement(_measurement);
    information().setIdentity();
    return true;
}

bool EyeEyeRightProjectEdge::write(std::ostream& os) const
{
    for (int i=0; i<2; i++)
        os<< measurement()[i] << " ";
    return os.good();
}


TransferedProjectAreaEdge::TransferedProjectAreaEdge() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    //information().setIdentity();
}

TransferedProjectAreaEdge::TransferedProjectAreaEdge(VertexSE3* inRobot, VertexSE3* betweenRig_, VertexSE3* B_i_) : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
    _vertices[0] = inRobot;
    _vertices[1] = betweenRig_;
    _vertices[2] = B_i_;
}

bool TransferedProjectAreaEdge::read(std::istream& is)
{
    // measured keypoint
    for (int i=0; i<2; i++)
        is>>_measurement[i];
    setMeasurement(_measurement);
    information().setIdentity();
    return true;
}

bool TransferedProjectAreaEdge::write(std::ostream& os) const
{
    for (int i=0; i<2; i++)
        os<<measurement()[i]<<" ";
    return os.good();
}

} // g2o
