/*
 * gladys.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-27
 * license: BSD
 */
#ifndef GLADYS_HPP
#define GLADYS_HPP

#include <vector>
#include <string>

#include "gladys/nav_graph.hpp"
#include "gladys/visibility_map.hpp"

namespace gladys {

typedef struct {
    int id;
    point_xy_t position;
    // TODO covariance
    // TODO state of data shared between robots
    // TODO "utilities" summarize past state? (see spec...)
} state_t;

typedef struct {
    double time;
    double energy;
    double distance;
} motion_constraints_t;

typedef std::pair<point_xy_t, float> points_prob_t;
typedef std::vector<points_prob_t>   points_probs_t;
typedef struct {} behaviour_t;

/*
 * gladys
 */
class gladys {
    nav_graph navigation_graph;
    visibility_map visibility;
public:
    /** gladys constructor
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_dtm path to a dtm.tif file
     * (multi-layers terrains elevation model, float32)
     *
     * @param f_robot_model TODO path to a robot model
     * to generate the weight map (at least its size)
     *
     */
    gladys(const std::string& f_region, const std::string& f_dtm,
            const std::string& f_robot_model) {
        navigation_graph.load(f_region, f_robot_model);
        visibility.load(f_dtm, f_robot_model);
    }

    /* state */
    state_t get_current_state() const;
    state_t create_new_state(int P, int A) const;
    void load_state(int id) const;
    void delete_state(int id) const;
    void clear_old_state() const; // clear all except current
    /* motion */
    points_t accessibility(const points_t& start,
        const motion_constraints_t& constraints) const;
    /* simulation */
    points_probs_t simulation(const points_t& start,
        const motion_constraints_t& constraints, behaviour_t behaviour) const;
    /* navigation */
    path_cost_util_t navigation(const points_t& start, const points_t& goal,
        int optimisation, const motion_constraints_t& constraints);
    /* perception TODO location = <X,Y,W> */
    bool is_visible(const point_xy_t& locA, const point_xy_t& locB) const;
    points_probs_t can_see(const point_xy_t& locA, const points_t& llocB) const;
    points_t is_visible_from(const point_xy_t& locA, const points_t& llocB,
        float qmin) const;
    bool test_visibility_link(const points_t& llocA, const points_t& llocB,
        float qmin) const;
    double look_at(int sensor, const points_t& observe) const;
    /* communication location = <X,Y,Z> */
    bool can_communicate(const point_xyz_t& locA, const point_xyz_t& locB) const;

};

} // namespace gladys

#endif // GLADYS_HPP

