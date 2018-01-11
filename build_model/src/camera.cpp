#include "camera.h"

namespace buildmodel
{

Camera::Camera()
{
//    _fx = Config::get<float>("camera.fx");
//    _fy = Config::get<float>("camera.fy");
//    _cx = Config::get<float>("camera.cx");
//    _cy = Config::get<float>("camera.cy");
//    _depth_scale = Config::get<float>("camera.depth_scale");
//    _dispara1 = Config::get<float>("camera.dispara1");
//    _dispara2 = Config::get<float>("camera.dispara2");
//    _dispara3 = Config::get<float>("camera.dispara3");
//    _dispara4 = Config::get<float>("camera.dispara4");

    _fx = 615;
    _fy = 615;
    _cx = 314;
    _cy = 238;
    _depth_scale = 1;

    _dispara1 = 0;
    _dispara2 = 0;
    _dispara3 = 0;
    _dispara4 = 0;

    _intrinsic_matrix << _fx,0,_cx,0,
                        0,_fy,_cy,0,
                        0,0,1,0;
    _camera_intrinsic_matrix  <<_fx, 0, _cx,
                                0, _fy, _cy,
                                0, 0, 1;
    _distParam <<_dispara1, _dispara2, _dispara3, _dispara4;
}

Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               _fx * p_c ( 0,0 ) / p_c ( 2,0 ) + _cx,
               _fy * p_c ( 1,0 ) / p_c ( 2,0 ) + _cy
           );
}

Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
               ( p_p ( 0,0 )-_cx ) *depth/_fx,
               ( p_p ( 1,0 )-_cy ) *depth/_fy,
               depth
           );
}

Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
