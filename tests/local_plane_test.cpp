#include "local_plane.hpp"
#include <cassert>
#include <limits>

int main()
{
    Eigen::Matrix<float, 5, 3> points;
    points << -1,-1,0, 1,-1,0, 1,1,0, -1,1,0, 0,0,0;
    Eigen::Vector4f plane;
    assert(fitLocalPlane(points, plane));
    assert(std::abs(plane.z()) > .999f && std::abs(plane.w()) < 1e-5f);
    points.rowwise() += Eigen::RowVector3f(100,200,3);
    assert(fitLocalPlane(points, plane));
    for (int i=0;i<5;++i) assert(std::abs(plane.head<3>().dot(points.row(i)) + plane.w()) < 1e-3f);
    points << 0,0,0, 1,0,0, 2,0,0, 3,0,0, 4,0,0;
    assert(!fitLocalPlane(points, plane));
    points << 0,0,0, 1,0,0, 0,1,0, 0,0,1, 1,1,1;
    assert(!fitLocalPlane(points, plane));
    points(0,0) = std::numeric_limits<float>::quiet_NaN();
    assert(!fitLocalPlane(points, plane));
}
