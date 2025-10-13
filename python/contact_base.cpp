#include "dynacom/contact_base.hpp"

#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <eigenpy/eigenpy.hpp>

#include "dynacom/python.hpp"

namespace dynacom {
namespace python {
namespace bp = boost::python;

void exposeContactBase() {
  class ContactBaseWrapper : public ContactBase,
                             public boost::python::wrapper<ContactBase> {
   public:
    virtual std::string& getFrameName() {
      return this->get_override("getFrameName")();
    }

    virtual void updateNewtonEuler() {
      this->get_override("updateNewtonEuler")();
    }
  };

  bp::class_<ContactBaseWrapper, boost::noncopyable>("ContactBase", bp::no_init)
      .def("getFrameName", bp::pure_virtual(&ContactBase::getFrameName),
           bp::return_value_policy<bp::reference_existing_object>())
      .def("updateNewtonEuler",
           bp::pure_virtual(&ContactBase::updateNewtonEuler))
      .def("getPose", &ContactBase::getPose,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("setPose", &ContactBase::setPose, bp::args("self"))
      .def("setFrameID", &ContactBase::setFrameID, bp::args("self", "frameID"))
      .def("applyForce", &ContactBase::applyForce, bp::args("self", "force"))
      .def("deactivate", &ContactBase::deactivate, bp::args("self"))
      .def("getFrameID", &ContactBase::getFrameID, bp::args("self"))
      .def("uni_rows", &ContactBase::uni_rows, bp::args("self"))
      .def("fri_rows", &ContactBase::fri_rows, bp::args("self"))
      .def("cols", &ContactBase::cols, bp::args("self"))
      .def("appliedForce", &ContactBase::appliedForce,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("uni_A", &ContactBase::uni_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("uni_b", &ContactBase::uni_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("fri_A", &ContactBase::fri_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("fri_b", &ContactBase::fri_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("reg_A", &ContactBase::reg_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("reg_b", &ContactBase::reg_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("ne_A", &ContactBase::ne_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      // .def("toWorldForces", &ContactBase::toWorldForces, bp::args("self"))
      .def("toCoMForces", &ContactBase::toCoMForces, bp::args("self"));
}

}  // namespace python
}  // namespace dynacom
