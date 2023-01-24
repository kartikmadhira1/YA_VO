#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include <sophus/se3.hpp>
#include <iostream>
#include "Image.hpp"

using namespace std;




// Eigen::aligned_allocator is used to allocate memory for Eigen objects.
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

/*

Part of Slambook2 series by Gaoxiang Zhang www.github.com/gaoxiang12/slambook2
Non-linear least squares (reprojection error) as a graph optimization problem.
vertices -> camera pose
edges -> reprojection error

*/



/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  // Vertex update function: oplusImpl. We know that the most important thing in the 
  // optimization process is the calculation of incremental ∆x, and this function deals with xk+1 = xk + ∆x process.
  // oplusImpl calculates the new pose from the old pose and the update.
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];

    // X + ∆X (in SE(3) space) = X * exp(∆X) (in se(3) space)
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;       
        EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}
        virtual void computeError() override {
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }
        // The Jacobian calculation function: linearizeOplus. 
        //In this function, we calcu- late the Jacobian of each edge relative to the vertex.
        virtual void linearizeOplus() override {
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            double Z2 = Z * Z;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;
        }

        virtual bool read(istream &in) override {}

        virtual bool write(ostream &out) const override {}

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};



class Optimizer {
  public:
    void partialBA(const vector<cv::Point3d> &points3D, const vector<cv::Point2d> &points2D, const cv::Mat &K, Sophus::SE3d &pose);
    void convertToEigen(const vector<cv::Point3d> &points3D, const vector<cv::Point2d> &points2D, VecVector2d &points2Deigen, VecVector3d &points3Deigen);

};

#endif // TODOITEM_H
