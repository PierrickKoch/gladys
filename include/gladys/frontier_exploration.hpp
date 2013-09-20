/*
 * frontier_exploration.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 * This class aims at providing a list of frontiers based on the cuurent
 * knowledge of the environment to perform exploration.
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-06
 * license: BSD
 */
#ifndef FRONTIER_EXPLORATION_HPP
#define FRONTIER_EXPLORATION_HPP

#include <vector>
#include <ostream>

#include "gladys/point.hpp"
#include "gladys/nav_graph.hpp"

// NOTE : it currently work only with 2D points

namespace gladys {

/*{{{ f_atttributes class
 ****************************************************************************/
class f_attributes {

public:
    /* the frontier attributes */
    // NB: the attributes of a frontier are dependent from others' attributes…
    double ID;                  // ID of the frontier (= its position in the vector)
    double size;                // nbr of frontier points
    double ratio ;              // importance of the frontier among others 
                                // ( max = 1  ; "value < 0" <=> unknown)
    point_xy_t lookout ;        // point from wich we want to observe the frontier
    path_t path ;               // path to the lookout, from the robot
    double distance;            // the cost of this path
    unsigned int proximity ;    // proximity is the number of robot closer to
                                // the look-out than the robot which computed
                                // the frontier
};

std::ostream& operator<< (std::ostream &out, const f_attributes& f) {//{{{
    out << "{ #" << f.ID << ": size = " << f.size << "; ratio = " << f.ratio 
        << "; lookout = (" << f.lookout[0] << "," << f.lookout[1] 
        << "); path size = " << f.path.size() << "; distance = " << f.distance
        << "; proximity = " << f.proximity 
        << " }";
    return out;
}///}}}

//}}}

/*{{{ frontier_detector class
 ****************************************************************************/

/* frontier_detector class */
class frontier_detector {

private :
    /* internal data */
    nav_graph ng ;                              // use for its weight_map and
                                                // the path pjanning
                                                // TODO  use as constant
    std::vector< points_t > frontiers ;         // the list of the frontiers
    std::vector< f_attributes > attributes ;    // the frontiers attributes

    /* hidden computing functions */
    /** compute_frontiers_WFD
     *
     * Compute the frontiers with the classical WFD (Wavefront Frontier
     * Detector) algorithm. Result is stored in frontiers.
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     * @throws : throw an exception if the seed is not valid.
     *
     */
    void compute_frontiers_WFD( const point_xy_t &seed );

    /** filter frontiers
     *
     * Aims at reducing the number of frontier by keeping only the nost
     * promising ones, and discarding the others. (This speeds up the
     * computation of attributes, and eases the planning beyond.)
     *
     */
    void filter_frontiers( size_t max_nf, size_t min_size ) ;

    /** compute_attributes
     *
     * Compute the frontier attributes for each elements in the frontiers list
     * frontiers.
     *
     * @param r_pos : the position of all the robot in the team. The first one
     * is assume to be the robot running the algorithm.
     *
     */
    void compute_attributes( const points_t &r_pos );

    /** is_frontier
     *
     * Tell if the given point is a frontier point.
     *
     * @param p : the point which is tested.
     *
     */
    bool is_frontier(  const point_xy_t &p, 
                      size_t height, size_t width,
                      const gdal::raster& data, const weight_map& map ) ;

    /** find_neighbours()
     *
     * Return the list of adjacent points (or neighbours) of p
     *
     * @param p : the point we focus.
     *
     * @param height : the height of the map.
     *
     * @param width : the height of the map.
     * 
     */
    points_t find_neighbours( const point_xy_t &p, size_t height, size_t width);

public:
    /* Name of the available algorithms to compute frontiers */
    typedef enum {WFD, FFD} algo_t;

    frontier_detector() ;
    /** frontier_detector constructor
     *
     * Create a nav_graph which loads region and robot model
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model to generate the weight map (at least its size)
     *
     */
    frontier_detector( const std::string& f_region, const std::string& f_robot_model ) ;

    /* public computing functions */
    /** compute_frontiers
     *
     * Compute the frontiers with the given algorithm and parameters.
     * Compute their attributes.
     *
     * @param r_pos : the position of all the robot in the team. The first one
     * is assume to be the robot running the algorithm.
     *
     * @param max_nf : max number of frontiers to consider (other are ignored ;
     * the filter uses some heuristics) ; default is 10
     * is WFD ( Wavefront Frontier Detection).
     *
     * @param min_size : minimal size of the frontier (other are ignored) ;
     * default is 2.
     *
     * @param algo : chose the algorithm used to compute the frontiers ; default
     * is WFD ( Wavefront Frontier Detection).
     *
     * @throws : throw an exception if the algo provided is invalid.
     *
     */
    void compute_frontiers( const points_t &r_pos,
                            size_t max_nf = 10, size_t min_size = 2,
                            algo_t algo = WFD );

    /* getters */
    const nav_graph& get_graph() const {//{{{
        return ng;
    }//}}}
    const std::vector< points_t >& get_frontiers() const {//{{{
        return frontiers;
    }//}}}
    const std::vector< f_attributes >& get_attributes() const {//{{{
        return attributes;
    }//}}}

};//}}}

} // namespace gladys

#endif // FRONTIER_EXPLORATION_HPP

