#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// Centering makes the fit valid even for a plane through the map origin.
inline bool fitLocalPlane(const Eigen::Matrix<float, 5, 3>& points, Eigen::Vector4f& plane)
{
    if (!points.allFinite()) return false;
    const Eigen::Vector3f center = points.colwise().mean();
    const Eigen::Matrix<float, 5, 3> centered = points.rowwise() - center.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(centered.transpose() * centered);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1e-5f ||
        solver.eigenvalues()[0] > 0.1f * solver.eigenvalues()[1]) return false;
    const Eigen::Vector3f normal = solver.eigenvectors().col(0);
    plane << normal, -normal.dot(center);
    return plane.allFinite() && (centered * normal).cwiseAbs().maxCoeff() <= 0.2f;
}
