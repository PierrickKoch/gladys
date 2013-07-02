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
    }

    double get_radius() {
        return pt.get("robot.radius", 0.0);
    }

    void set_radius(double radius) {
        pt.put("robot.radius", radius);
    }

    void save(const std::string& filepath) {
        write_json(filepath, pt);
    }
};

} // namespace gladys

#endif // ROBOT_MODEL_HPP

