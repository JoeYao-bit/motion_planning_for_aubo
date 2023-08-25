//
// Created by yaozhuo on 2021/11/27.
//

#ifndef BASIC_BASE_TEB_EDGES_H
#define BASIC_BASE_TEB_EDGES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include "teb_config.h"

/**
 * @class BaseTebUnaryEdge
 * @brief Base edge connecting a single vertex in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimizer class.
 * @see BaseTebMultiEdge, BaseTebBinaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:

    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    /**
    * @brief Compute and return error / cost value.
    *
    * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
    * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
    */
    ErrorVector& getError()
    {
        computeError();
        return _error;
    }

    /**
     * @brief Read values from input stream
     */
    virtual bool read(std::istream& is)
    {
        // TODO generic read
        return true;
    }

    /**
     * @brief Write values to an output stream
     */
    virtual bool write(std::ostream& os) const
    {
        // TODO generic write
        return os.good();
    }

    /**
     * @brief Assign the TebConfig class for parameters.
     * @param cfg TebConfig class
     */
    void setTebConfig(const TebConfig& cfg)
    {
        cfg_ = &cfg;
    }

protected:

    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

    const TebConfig* cfg_; //!< Store TebConfig class for parameters

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class BaseTebBinaryEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimizer class.
 * @see BaseTebMultiEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:

    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    /**
    * @brief Compute and return error / cost value.
    *
    * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
    * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
    */
    ErrorVector& getError()
    {
        computeError();
        return _error;
    }

    /**
     * @brief Read values from input stream
     */
    virtual bool read(std::istream& is)
    {
        // TODO generic read
        return true;
    }

    /**
     * @brief Write values to an output stream
     */
    virtual bool write(std::ostream& os) const
    {
        // TODO generic write
        return os.good();
    }

    /**
     * @brief Assign the TebConfig class for parameters.
     * @param cfg TebConfig class
     */
    void setTebConfig(const TebConfig& cfg)
    {
        cfg_ = &cfg;
    }

protected:

    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

    const TebConfig* cfg_; //!< Store TebConfig class for parameters

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class BaseTebMultiEdge
 * @brief Base edge connecting multiple vertices in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimizer class.
 * @see BaseTebBinaryEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:

    using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
    using g2o::BaseMultiEdge<D, E>::computeError;

    // Overwrites resize() from the parent class
    virtual void resize(size_t size)
    {
        g2o::BaseMultiEdge<D, E>::resize(size);

        for(std::size_t i=0; i<_vertices.size(); ++i)
            _vertices[i] = NULL;
    }

    /**
    * @brief Compute and return error / cost value.
    *
    * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
    * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
    */
    ErrorVector& getError()
    {
        computeError();
        return _error;
    }

    /**
     * @brief Read values from input stream
     */
    virtual bool read(std::istream& is)
    {
        // TODO generic read
        return true;
    }

    /**
     * @brief Write values to an output stream
     */
    virtual bool write(std::ostream& os) const
    {
        // TODO generic write
        return os.good();
    }

    /**
     * @brief Assign the TebConfig class for parameters.
     * @param cfg TebConfig class
     */
    void setTebConfig(const TebConfig& cfg)
    {
        cfg_ = &cfg;
    }

protected:

    using g2o::BaseMultiEdge<D, E>::_error;
    using g2o::BaseMultiEdge<D, E>::_vertices;

    const TebConfig* cfg_; //!< Store TebConfig class for parameters

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //FREENAV_BASIC_BASE_TEB_EDGES_H
