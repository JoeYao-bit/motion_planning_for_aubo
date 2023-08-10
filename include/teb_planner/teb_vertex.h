//
// Created by yaozhuo on 2021/11/27.
//

#ifndef FREENAV_VERTEX_H
#define FREENAV_VERTEX_H


#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "pose_se2.h"


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

using namespace std;

/**
  * @class VertexPose
  * @brief This class stores and wraps a SE2 pose (position and orientation) into a vertex that can be optimized via g2o
  * @see PoseSE2
  * @see VertexTimeDiff
  */
class VertexPose : public g2o::BaseVertex<3, PoseSE2 >
{
public:

    /**
      * @brief Default constructor
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexPose(bool fixed = false)
    {
        setToOriginImpl();
        setFixed(fixed);
    }

    /**
      * @brief Construct pose using a given PoseSE2
      * @param pose PoseSE2 defining the pose [x, y, angle_rad]
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexPose(const PoseSE2& pose, bool fixed = false)
    {
        _estimate = pose;
        setFixed(fixed);
    }

    /**
      * @brief Construct pose using a given 2D position vector and orientation
      * @param position Eigen::Vector2d containing x and y coordinates
      * @param theta yaw-angle
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed = false)
    {
        _estimate.position() = position;
        _estimate.theta() = theta;
        setFixed(fixed);
    }

    /**
      * @brief Construct pose using single components x, y, and the yaw angle
      * @param x x-coordinate
      * @param y y-coordinate
      * @param theta yaw angle in rad
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexPose(double x, double y, double theta, bool fixed = false)
    {
        _estimate.x() = x;
        _estimate.y() = y;
        _estimate.theta() = theta;
        setFixed(fixed);
    }

    /**
      * @brief Access the pose
      * @see estimate
      * @return reference to the PoseSE2 estimate
      */
    inline PoseSE2& pose() {return _estimate;}

    /**
      * @brief Access the pose (read-only)
      * @see estimate
      * @return const reference to the PoseSE2 estimate
      */
    inline const PoseSE2& pose() const {return _estimate;}


    /**
      * @brief Access the 2D position part
      * @see estimate
      * @return reference to the 2D position part
      */
    inline Eigen::Vector2d& position() {return _estimate.position();}

    /**
      * @brief Access the 2D position part (read-only)
      * @see estimate
      * @return const reference to the 2D position part
      */
    inline const Eigen::Vector2d& position() const {return _estimate.position();}

    /**
      * @brief Access the x-coordinate the pose
      * @return reference to the x-coordinate
      */
    inline double& x() {return _estimate.x();}

    /**
      * @brief Access the x-coordinate the pose (read-only)
      * @return const reference to the x-coordinate
      */
    inline const double& x() const {return _estimate.x();}

    /**
      * @brief Access the y-coordinate the pose
      * @return reference to the y-coordinate
      */
    inline double& y() {return _estimate.y();}

    /**
      * @brief Access the y-coordinate the pose (read-only)
      * @return const reference to the y-coordinate
      */
    inline const double& y() const {return _estimate.y();}

    /**
      * @brief Access the orientation part (yaw angle) of the pose
      * @return reference to the yaw angle
      */
    inline double& theta() {return _estimate.theta();}

    /**
      * @brief Access the orientation part (yaw angle) of the pose (read-only)
      * @return const reference to the yaw angle
      */
    inline const double& theta() const {return _estimate.theta();}

    /**
      * @brief Set the underlying estimate (2D vector) to zero.
      */
    virtual void setToOriginImpl() override
    {
        _estimate.setZero();
    }

    /**
      * @brief Define the update increment \f$ \mathbf{x}_{k+1} = \mathbf{x}_k + \Delta \mathbf{x} \f$.
      * A simple addition for the position.
      * The angle is first added to the previous estimated angle and afterwards normalized to the interval \f$ [-\pi \pi] \f$
      * @param update increment that should be added to the previous esimate
      */
    virtual void oplusImpl(const double* update) override
    {
        _estimate.plus(update);
    }

    /**
      * @brief Read an estimate from an input stream.
      * First the x-coordinate followed by y and the yaw angle.
      * @param is input stream
      * @return always \c true
      */
    virtual bool read(std::istream& is) override
    {
        is >> _estimate.x() >> _estimate.y() >> _estimate.theta();
        return true;
    }

    /**
      * @brief Write the estimate to an output stream
      * First the x-coordinate followed by y and the yaw angle.
      * @param os output stream
      * @return \c true if the export was successful, otherwise \c false
      */
    virtual bool write(std::ostream& os) const override
    {
        os << _estimate.x() << " " << _estimate.y() << " " << _estimate.theta();
        return os.good();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
  * @class VertexTimeDiff
  * @brief This class stores and wraps a time difference \f$ \Delta T \f$ into a vertex that can be optimized via g2o
  */
class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:

    /**
      * @brief Default constructor
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexTimeDiff(bool fixed = false)
    {
        setToOriginImpl();
        setFixed(fixed);
    }

    /**
      * @brief Construct the TimeDiff vertex with a value
      * @param dt time difference value of the vertex
      * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
      */
    VertexTimeDiff(double dt, bool fixed = false)
    {
        _estimate = dt;
        setFixed(fixed);
    }

    /**
      * @brief Access the timediff value of the vertex
      * @see estimate
      * @return reference to dt
      */
    inline double& dt() {return _estimate;}

    /**
      * @brief Access the timediff value of the vertex (read-only)
      * @see estimate
      * @return const reference to dt
      */
    inline const double& dt() const {return _estimate;}

    /**
      * @brief Set the underlying TimeDiff estimate \f$ \Delta T \f$ to default.
      */
    virtual void setToOriginImpl() override
    {
        _estimate = 0.1;
    }

    /**
      * @brief Define the update increment \f$ \Delta T_{k+1} = \Delta T_k + update \f$.
      * A simple addition implements what we want.
      * @param update increment that should be added to the previous estimate
      */
    virtual void oplusImpl(const double* update) override
    {
        _estimate += *update;
    }

    /**
      * @brief Read an estimate of \f$ \Delta T \f$ from an input stream
      * @param is input stream
      * @return always \c true
      */
    virtual bool read(std::istream& is) override
    {
        is >> _estimate;
        return true;
    }

    /**
      * @brief Write the estimate \f$ \Delta T \f$ to an output stream
      * @param os output stream
      * @return \c true if the export was successful, otherwise \c false
      */
    virtual bool write(std::ostream& os) const override
    {
        os << estimate();
        return os.good();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //FREENAV_VERTEX_H
