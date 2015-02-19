/*
 * gladys.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-27
 * license: BSD
 */

#include <vector>
#include <string>

#include "gladys/gladys.hpp"

namespace gladys {

/* state */

state_t gladys::get_current_state() const {
    state_t state;
    return state;
}
state_t gladys::create_new_state(int P, int A) const {
    state_t state;
    return state;
}
void gladys::load_state(int id) const {
}
void gladys::delete_state(int id) const {
}
void gladys::clear_old_state() const {
    // clear all except current
}

/* motion */

points_t gladys::accessibility(const points_t& start,
    const motion_constraints_t& constraints) const
{
    points_t points;
    // TODO
    return points;
}

/* simulation */

points_probs_t gladys::simulation(const points_t& start,
    const motion_constraints_t& constraints, behaviour_t behaviour) const
{
    points_probs_t pbs;
    // TODO
    return pbs;
}

/* navigation */

//path_cost_util_t gladys::navigation(const points_t& start, const points_t& goal,
//    int optimisation, const motion_constraints_t& constraints)
path_cost_util_t gladys::navigation(const points_t& start, const points_t& goal)
{
    return navigation_graph.astar_search_custom(start, goal);
}

std::vector<double> gladys::single_source_all_costs(const point_xy_t& start, const points_t& goals){
    return navigation_graph.single_source_all_costs(start, goals);
}

/* perception */

bool gladys::is_visible(const point_xy_t& locSensor, const point_xy_t& locTarget) const
{
    return visibility.is_visible(locSensor, locTarget);
}

points_probs_t gladys::can_see(const point_xy_t& locA, const points_t& llocB) const
{
    points_probs_t pbs(llocB.size());
    for (point_xy_t locB : llocB)
        pbs.push_back({locB, is_visible(locA, locB) ? 1.0 : 0.0});
    return pbs;
}

points_t gladys::is_visible_from(const point_xy_t& locA, const points_t& llocB,
    float qmin) const
{
    points_t points;
    for (point_xy_t locB : llocB)
        if (is_visible(locA, locB))
            points.push_back(locB);
    return points;
}

bool gladys::test_visibility_link(const points_t& llocA, const points_t& llocB,
    float qmin) const
{
    for (const point_xy_t& locA : llocA)
        for (const point_xy_t& locB : llocB)
            if (is_visible(locA, locB))
                return true;
    return false;
}
double gladys::look_at(int sensor, const points_t& observe) const {
    // TODO
    return 1.0;
}

/* communication */

bool gladys::can_communicate(const point_xyz_t& locA, const point_xyz_t& locB) const
{
    return visibility.is_visible(locA, locB);
}

} // namespace gladys
