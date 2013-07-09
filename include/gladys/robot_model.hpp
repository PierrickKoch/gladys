/*
 * robot_model.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-02
 * license: BSD
 */
#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <string>

#include <boost/property_tree/json_parser.hpp>

namespace gladys {

/*
 * from robot model (in json)
 */
class robot_model {
    boost::property_tree::ptree pt;
public:
    /** load robot model
     *
     * @param filepath path to a robot model
     *
     */
    void load(const std::string& filepath) {
        read_json(filepath, pt);
        // throw an exception if a key is not found
        if (pt.get<double>("robot.radius") <= 0 or
            pt.get<double>("robot.velocity") <= 0)
            throw std::runtime_error("[robot_model] radius and velocity must be positive");
    }

    double get_radius() const {
        return pt.get<double>("robot.radius");
    }

    void set_radius(double radius) {
        pt.put("robot.radius", radius);
    }

    double get_velocity() const {
        return pt.get<double>("robot.velocity");
    }

    void set_velocity(double velocity) {
        pt.put("robot.velocity", velocity);
    }

    void save(const std::string& filepath) const {
        write_json(filepath, pt);
    }
};

} // namespace gladys

#endif // ROBOT_MODEL_HPP

