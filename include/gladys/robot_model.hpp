/*
 * robot_model.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 *          Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-07-02
 * license: BSD
 */
#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <string>
#include <map>

#include <boost/version.hpp>
// json_parser bug #6785 in boost 1.49
// https://svn.boost.org/trac/boost/ticket/6785
// https://bugs.launchpad.net/bugs/1232800
#if ((BOOST_VERSION % 100) == 1) && ((BOOST_VERSION / 100 % 1000) == 49)
#error Boost 1.49 json_parser bug #6785
#endif

#include <boost/property_tree/json_parser.hpp>
// http://www.boost.org/doc/libs/1_54_0/doc/html/boost_propertytree/parsers.html#boost_propertytree.parsers.json_parser

#include "gladys/point.hpp"

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
        if (pt.get<double>("robot.mass")     <= 0 or
            pt.get<double>("robot.radius")   <= 0 or
            pt.get<double>("robot.velocity") <= 0 )
            throw std::runtime_error("[robot_model] mass, radius and velocity must be positive");
    }

    /* TODO get a **dymanic** map for weight function

       string("REGION_MAP_BAND_OR_CLASS_NAME") -> float(ponderation)

       string("OBSTACLE")   -> float(+infinite)
       string("FLAT")       -> float(0.0)
       string("ROUGH")      -> float(0.5)
       string("SLOPE")      -> float(0.6)
       ...
     */
    std::map<std::string, float> get_costs() const {
        std::map<std::string, float> costs;
        costs["FLAT"]  = 0.0; // v-max     on flat   ground
        costs["ROUGH"] = 5.0; // v-max / 5 on rought ground
        // costs["SLOPE"] = 3.0; // v-max / 3 on slope  ground
        // threshold not cost ["OBSTACLE"] = 1E+6;
        // threshold not cost ["NO_3D_CLASS"] = -1;
        return costs;
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

    /** get the position of the eye sensor
     *
     * (relative to the ground level, aka. sensor's height)
     * used to build the visibility map
     * we need to consider a better orientation for flying observers
     */
    point_xyzt_t get_sensor_pose() const {
        point_xyzt_t p ;
        p[0] = pt.get<double>("sensor.pose.x");
        p[1] = pt.get<double>("sensor.pose.y");
        p[2] = pt.get<double>("sensor.pose.z");
        p[3] = pt.get<double>("sensor.pose.t");
        return p ;
    }

    double get_sensor_fov() const {
        return pt.get<double>("sensor.fov");
    }

    double get_sensor_range() const {
        return pt.get<double>("sensor.range");
    }

    void set_sensor_pose( point_xyzt_t p ) {
        pt.put("sensor.pose.x", p[0] );
        pt.put("sensor.pose.y", p[1] );
        pt.put("sensor.pose.z", p[2] );
        pt.put("sensor.pose.t", p[3] );
    }

    void set_sensor_fov(double fov) {
        pt.put("sensor.fov", fov );
    }

    void set_sensor_range(double range) {
        pt.put("sensor.range", range );
    }

    void save(const std::string& filepath) const {
        write_json(filepath, pt);
    }
};

} // namespace gladys

#endif // ROBOT_MODEL_HPP

