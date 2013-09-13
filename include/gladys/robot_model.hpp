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

#include <boost/version.hpp>
// json_parser bug #6785 in boost 1.49
// https://svn.boost.org/trac/boost/ticket/6785
#if ((BOOST_VERSION % 100) == 1) && ((BOOST_VERSION / 100 % 1000) == 49)
#error Boost 1.49 json_parser bug #6785
#endif

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
        if (pt.get<double>("robot.eyez")     <= 0 or
            pt.get<double>("robot.mass")     <= 0 or
            pt.get<double>("robot.radius")   <= 0 or
            pt.get<double>("robot.velocity") <= 0 )
            throw std::runtime_error("[robot_model] eyez, mass, radius and velocity must be positive");
    }

    /* TODO get a dymanic map for weight function

       string("REGION_MAP_BAND_OR_CLASS_NAME") -> float(ponderation)

       string("OBSTACCLE")  -> float(+infinite)
       string("FLAT")       -> float(0.0)
       string("ROUGH")      -> float(0.5)
       string("SLOPE")      -> float(0.6)
       ...
     */

    /** get the Z position of the eye sensor
     * (relative to the ground level, aka. sensor's height)
     *
     * used to build the visibility map
     */
    double get_eyez() const {
        return pt.get<double>("robot.eyez");
    }

    void set_eyez(double eyez) {
        pt.put("robot.eyez", eyez);
    }

    double get_mass() const {
        return pt.get<double>("robot.mass");
    }

    void set_mass(double mass) {
        pt.put("robot.mass", mass);
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

