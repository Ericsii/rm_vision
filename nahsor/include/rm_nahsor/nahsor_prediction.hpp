#ifndef NAHSOR_MODULES_NAHSOR_PREDICTION_H
#define NAHSOR_MODULES_NAHSOR_PREDICTION_H
#include <opencv2/opencv.hpp>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/types_slam.3d.h"
//#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/stuff/command_args.h"
#include <iostream>
#include "Eigen/Core"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <chrono>
#include <cmath>
namespace nahsor
{
    class NahsorPrediction
    {
    public:
        
    };

    //以下为优化类，抄的书上的代码，就不写注释了
    class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        virtual void setToOriginImpl();
        virtual void oplusImpl(const double *update);
        virtual bool read(std::istream &in);
        virtual bool write(std::ostream &out) const;
    };
    class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CurveFittingEdge(double x);
        virtual void computeError() override;
        virtual void linearizeOplus() override;
        virtual bool read(std::istream &in);
        virtual bool write(std::ostream &out) const;

    public:
        double _x;
    };
}
#endif