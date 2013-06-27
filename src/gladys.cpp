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

state_t gladys::get_current_state() {
    state_t state;
    return state;
}
state_t gladys::create_new_state(int P, int A) {
    state_t state;
    return state;
}
void gladys::load_state(int id) {
}
void gladys::delete_state(int id) {
}
void gladys::clear_old_state() {
    // clear all except current
}

/* motion */

points_t gladys::accessibility(const points_t& start, const motion_constraints_t& constraints) {
    points_t points;
    return points;
}

/* simulation */

points_probs_t gladys::simulation(const points_t& start, const motion_constraints_t& constraints, behaviour_t behaviour) {
    points_probs_t pbs;
    return pbs;
}

/* navigation */

path_cost_util_t gladys::navigation(const points_t& start, const points_t& goal, int optimisation, const motion_constraints_t& constraints) {
    path_cost_util_t pcu;
    return pcu;
}

/* perception */

points_probs_t gladys::can_see(int sensor, const points_t& location) {
    points_probs_t pbs;
    return pbs;
}
points_t gladys::is_visible_from(int sensor, const points_t& visible, float qmin) {
    points_t points;
    return points;
}
bool gladys::test_visibility_link(int sensor, const points_t& location, const points_t& visible, float qmin) {
    return true;
}
double gladys::look_at(int sensor, const points_t& observe) {
    return 1.0;
}

} // namespace gladys


// TODO Boost.Test <utf> Unit Test Framework - http://www.boost.org/libs/test

int main(int argc, char * argv[])
{
    if (argc < 3)
        return 1;

    gladys::gladys obj(argv[1], argv[2]);

    std::cout<<"gladys::gladys!"<<std::endl;

    return EXIT_SUCCESS;
}
