#include "dynacom/contact_point.hpp"

#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <eigenpy/eigenpy.hpp>

#include "dynacom/python.hpp"

namespace dynacom {
namespace python {
namespace bp = boost::python;

bp::dict get_settings(ContactPoint& self) {
  bp::dict settings;
  ContactPointSettings conf = self.getSettings();
  settings["frame_name"] = conf.frame_name;
  settings["mu"] = conf.mu;
  settings["weights"] = conf.weights;

  return settings;
}

void set_weights(ContactPointSettings& self, Eigen::Vector3d w) {
  self.weights = w;
}

Eigen::Vector3d get_weights(ContactPointSettings& self) { return self.weights; }

void exposeContactPoint() {
  bp::class_<ContactPointSettings>("ContactPointSettings")
      .def_readwrite("frame_name", &ContactPointSettings::frame_name)
      .def_readwrite("mu", &ContactPointSettings::mu)
      .add_property("weights", &get_weights, &set_weights)
      .def("__repr__", &ContactPointSettings::to_string)
      .def("__eq__", &ContactPointSettings::operator==)
      .def("__ne__", &ContactPointSettings::operator!=);

  bp::class_<ContactPoint, bp::bases<ContactBase>, boost::noncopyable>(
      "ContactPoint", bp::init<>())
      .def("initialize", &ContactPoint::initialize,
           bp::args("self", "settings"))
      .def("get_settings", &get_settings, bp::args("self"))
      .def("set_mu", &ContactPoint::setMu, bp::args("self", "mu"))
      .def("set_force_weights", &ContactPoint::setForceWeights,
           bp::args("self", "force_weights"))
      .def("updateNewtonEuler", &ContactPoint::updateNewtonEuler,
           bp::args("self", "CoM", "oMf"))
      .def("getFrameName", &ContactPoint::getFrameName,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"));
}
}  // namespace python
}  // namespace dynacom
