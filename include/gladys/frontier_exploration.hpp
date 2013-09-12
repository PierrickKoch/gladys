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

#include "gladys/point.hpp"
#include "gladys/weight_map.hpp"

// NOTE : it currently work only with 2D points

namespace gladys {

/*{{{ frontier class
 ****************************************************************************/
class Frontier {

protected :
    /* list of the frontier points */
    points_t points ;

    /* the frontier attributes */
    double size;                // nbr of frontier points
    double distance;            // some distance between XX and the frontier
    //TODO add other meaningful attributes

public :
    /* constructors */
    Frontier();
    Frontier(const points_t &_points);

    /* computing functions */
    void compute_attributes();

    /* setters */
    void add_point( const point_xy_t _points);

    /* getters */
    const points_t& get_points();
    //TODO get attributes ?

};//}}}

/*{{{ fExploration class
 ****************************************************************************/

/* New type definition */
typedef std::vector< Frontier > frontiers_list_t;

/* fExploration class */
class fExploration {

private :
    /* internal data */
    const weight_map& map ;                     // the map used to compute frontiers
    weight_map imap ;                           // internal map, used if no extenal weight_map is provided
                                                // TODO remove it
    frontiers_list_t fList ;                    // the list of the frontiers

    /* hidden computing functions */
    /** compute_frontiers_WFD
     *
     * Compute the frontiers with the classical WFD (Wavefront Frontier
     * Detector) algorithm. Result is stored in fList.
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     * @throws : throw an exception if the seed is not valid.
     *
     */
    void compute_frontiers_WFD(const point_xy_t seed);
    
    /** compute_attributes
     *
     * Compute the frontier attributes for each elements in the frontiers list
     * fList.
     *
     */
    void compute_attributes();

    /** isFrontier
     *
     * Tell if the given point is a frontier point.
     *
     * @param p : the point which is tested.
     *
     */
    bool isFrontier(  const point_xy_t p, 
                      size_t height, size_t width,
                      const gdal::raster& data ) ;

    /** hasOpenSpaceNeighbour
     *
     * Tell if the given point has at least one neighbour in the OpenSpace
     *
     * @param p : the point which is tested.
     *
     */
    bool hasOpenSpaceNeighbour( const point_xy_t p,
                                size_t height, size_t width,
                                const gdal::raster& data ) ;

    /** isOpenSpace
     *
     * Tell if the given point is in the OpenSpace
     *
     * @param p : the point which is tested.
     *
     */
    bool isOpenSpace( const point_xy_t p,
                      const gdal::raster& data ) ;

    /** findNeighbours()
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
    points_t findNeighbours( const point_xy_t p, size_t height, size_t width);

public:
    /* Name of the available algorithms to compute frontiers */
    typedef enum {WFD, FFD} algo_t;

    
    /** fExploration constructor
     *
     * Constuctor when there is no pre-existant weight_map.
     * Create a weight_map which load region and robot model
     * (see also the weighte map class)
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model to generate the weight map (at least its size)
     *
     */

    fExploration( const std::string& f_region, const std::string& f_robot_model ) ;

    /** fExploration constructor
     *
     * Constuctor using a pre-existant weight_map.
     *
     * @param weight_map : a previously computed weight_map
     *
     */
    fExploration( const weight_map& _map ) ;

    /* hidden computing functions */
    /** compute_frontiers
     *
     * Compute the frontiers with the given algorithm and parameters.
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     * @param algo : chose the algorithm used to compute the frontiers ; default
     * is WFD ( Wavefront Frontier Detection).
     *
     * @throws : throw an exception if the algo provided is invalid.
     *
     */
    void compute_frontiers(const point_xy_t seed, const algo_t algo = WFD);

    //void save_frontiers(const std::string& filepath) ;
    void save_frontiers() ;

    /* getters */
    const weight_map& get_map() const ;
    const frontiers_list_t& get_frontiers() const ;

};//}}}

} // namespace gladys

#endif // FRONTIER_EXPLORATION_HPP

