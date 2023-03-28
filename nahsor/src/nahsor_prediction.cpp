#include "rm_nahsor/nahsor_prediction.hpp"

namespace nahsor
{
    void CurveFittingVertex::setToOriginImpl()
    {
        _estimate << 1, 0.5, 1;
    }

    void CurveFittingVertex::oplusImpl(const double *update)
    {
        _estimate += Eigen::Vector3d(update);
    }

    bool CurveFittingVertex::read(std::istream &in)
    {
        (void)in;
        return true;
    }
    bool CurveFittingVertex::write(std::ostream &out) const
    {
        (void)out;
        return true;
    }

    CurveFittingEdge::CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    void CurveFittingEdge::computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d params = v->estimate(); //0:起始时间,1:初相
        //_error(0, 0) = _measurement + (0.785 / 1.884 * cos(1.884 * (params(0, 0) + _x)) + 1.305 * _x + params(1, 0));
        //v = a * sin(w * (t + t0)) + b，其中b = 2.09 - a
        _error(0, 0) = _measurement - (params(0, 0) * sin(params(1, 0) * (params(2, 0) + _x)) + 2.0 - params(0,0));
    }

    void CurveFittingEdge::linearizeOplus()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d params = v->estimate();
        // double u = 1.884 * (params(0, 0) + _x);
        double u = params[1] * (_x + params[2]);
        _jacobianOplusXi[0] = 1 - sin(u);
        _jacobianOplusXi[1] = -params[0] * (_x + params[2]) * cos(u);
        _jacobianOplusXi[2] = -params[0] * params[1] * cos(u);
    }

    bool CurveFittingEdge::read(std::istream &in)
    {
        (void)in;
        return true;
    }
    bool CurveFittingEdge::write(std::ostream &out) const
    {
        (void)out;
        return true;
    }

}