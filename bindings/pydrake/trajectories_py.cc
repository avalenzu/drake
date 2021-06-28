#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace pydrake {
namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;
  constexpr auto& doc = pydrake_doc.drake.trajectories;

  {
    using Class = Trajectory<T>;
    constexpr auto& cls_doc = doc.Trajectory;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "Trajectory", param, cls_doc.doc);
    cls  // BR
        .def("value", &Class::value, py::arg("t"),
            cls_doc.value.doc)
        .def("vector_values", &Class::vector_values,
            cls_doc.vector_values.doc)
        .def("EvalDerivative", &Class::EvalDerivative, py::arg("t"),
            py::arg("derivative_order") = 1, cls_doc.EvalDerivative.doc)
        .def("MakeDerivative", &Class::MakeDerivative,
            py::arg("derivative_order") = 1, cls_doc.MakeDerivative.doc)
        .def("start_time", &Class::start_time,
            cls_doc.start_time.doc)
        .def("end_time", &Class::end_time, cls_doc.end_time.doc)
        .def("rows", &Class::rows, cls_doc.rows.doc)
        .def("cols", &Class::cols, cls_doc.cols.doc);
  }

  {
    using Class = BsplineTrajectory<T>;
    constexpr auto& cls_doc = doc.BsplineTrajectory;
    auto cls = DefineTemplateClassWithDefault<Class, Trajectory<T>>(
        m, "BsplineTrajectory", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(py::init<math::BsplineBasis<T>, std::vector<MatrixX<T>>>(),
            py::arg("basis"), py::arg("control_points"),
            cls_doc.ctor.doc)
        .def("Clone", &Class::Clone,
            cls_doc.Clone.doc)
        .def("num_control_points", &Class::num_control_points,
            cls_doc.num_control_points.doc)
        .def("control_points", &Class::control_points,
            cls_doc.control_points.doc)
        .def("InitialValue", &Class::InitialValue,
            cls_doc.InitialValue.doc)
        .def("FinalValue", &Class::FinalValue,
            cls_doc.FinalValue.doc)
        .def("basis", &Class::basis,
            cls_doc.basis.doc)
        .def("InsertKnots", &Class::InsertKnots,
            py::arg("additional_knots"), cls_doc.InsertKnots.doc)
        .def("CopyBlock", &Class::CopyBlock, py::arg("start_row"),
            py::arg("start_col"), py::arg("block_rows"), py::arg("block_cols"),
            cls_doc.CopyBlock.doc)
        .def("CopyHead", &Class::CopyHead, py::arg("n"),
            cls_doc.CopyHead.doc);
  }
}

// void DoScalarIndependentDefinitions(py::module m) {
//   // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
//   using namespace drake::trajectories;
//   constexpr auto& doc = pydrake_doc.drake.trajectories;
//   py::module::import("pydrake.polynomial");

//   using T = double;

//   py::class_<PiecewiseTrajectory<T>, Trajectory<T>>(
//       m, "PiecewiseTrajectory", doc.PiecewiseTrajectory.doc)
//       .def("get_number_of_segments",
//           &PiecewiseTrajectory<T>::get_number_of_segments,
//           doc.PiecewiseTrajectory.get_number_of_segments.doc)
//       .def("start_time",
//           overload_cast_explicit<double, int>(
//               &PiecewiseTrajectory<T>::start_time),
//           py::arg("segment_index"), doc.PiecewiseTrajectory.start_time.doc)
//       .def("end_time",
//           overload_cast_explicit<double, int>(
//               &PiecewiseTrajectory<T>::end_time),
//           py::arg("segment_index"), doc.PiecewiseTrajectory.end_time.doc)
//       .def("duration", &PiecewiseTrajectory<T>::duration,
//           py::arg("segment_index"), doc.PiecewiseTrajectory.duration.doc)
//       // N.B. We must redefine these two overloads, as we cannot use the base
//       // classes' overloads. See: https://github.com/pybind/pybind11/issues/974
//       .def("start_time",
//           overload_cast_explicit<double>(&PiecewiseTrajectory<T>::start_time),
//           doc.PiecewiseTrajectory.start_time.doc)
//       .def("end_time",
//           overload_cast_explicit<double>(&PiecewiseTrajectory<T>::end_time),
//           doc.PiecewiseTrajectory.end_time.doc)
//       .def("is_time_in_range", &PiecewiseTrajectory<T>::is_time_in_range,
//           py::arg("t"), doc.PiecewiseTrajectory.is_time_in_range.doc)
//       .def("get_segment_index", &PiecewiseTrajectory<T>::get_segment_index,
//           py::arg("t"), doc.PiecewiseTrajectory.get_segment_index.doc)
//       .def("get_segment_times", &PiecewiseTrajectory<T>::get_segment_times,
//           doc.PiecewiseTrajectory.get_segment_times.doc);

//   py::class_<PiecewisePolynomial<T>, PiecewiseTrajectory<T>>(
//       m, "PiecewisePolynomial", doc.PiecewisePolynomial.doc)
//       .def(py::init<>(), doc.PiecewisePolynomial.ctor.doc_0args)
//       .def(py::init<const Eigen::Ref<const MatrixX<T>>&>(),
//           doc.PiecewisePolynomial.ctor.doc_1args_constEigenMatrixBase)
//       .def(py::init<std::vector<MatrixX<Polynomial<T>>> const&,
//                std::vector<double> const&>(),
//           doc.PiecewisePolynomial.ctor.doc_2args_polynomials_matrix_breaks)
//       .def(py::init<std::vector<Polynomial<T>> const&,
//                std::vector<double> const&>(),
//           doc.PiecewisePolynomial.ctor.doc_2args_polynomials_breaks)
//       .def_static("ZeroOrderHold",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&>(
//               &PiecewisePolynomial<T>::ZeroOrderHold),
//           doc.PiecewisePolynomial.ZeroOrderHold.doc)
//       .def_static("FirstOrderHold",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&>(
//               &PiecewisePolynomial<T>::FirstOrderHold),
//           doc.PiecewisePolynomial.FirstOrderHold.doc)
//       .def_static("CubicShapePreserving",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&, bool>(
//               &PiecewisePolynomial<T>::CubicShapePreserving),
//           py::arg("breaks"), py::arg("samples"),
//           py::arg("zero_end_point_derivatives") = false,
//           doc.PiecewisePolynomial.CubicShapePreserving.doc)
//       .def_static("CubicWithContinuousSecondDerivatives",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&,
//               const Eigen::Ref<const VectorX<T>>&,
//               const Eigen::Ref<const VectorX<T>>&>(
//               &PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives),
//           py::arg("breaks"), py::arg("samples"), py::arg("sample_dot_at_start"),
//           py::arg("sample_dot_at_end"),
//           doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
//               .doc_4args)
//       .def_static("CubicHermite",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&,
//               const Eigen::Ref<const MatrixX<T>>&>(
//               &PiecewisePolynomial<T>::CubicHermite),
//           py::arg("breaks"), py::arg("samples"), py::arg("samples_dot"),
//           doc.PiecewisePolynomial.CubicHermite.doc)
//       .def_static("CubicWithContinuousSecondDerivatives",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&, bool>(
//               &PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives),
//           py::arg("breaks"), py::arg("samples"), py::arg("periodic_end"),
//           doc.PiecewisePolynomial.CubicWithContinuousSecondDerivatives
//               .doc_3args)
//       .def_static("LagrangeInterpolatingPolynomial",
//           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
//               const Eigen::Ref<const MatrixX<T>>&>(
//               &PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial),
//           py::arg("times"), py::arg("samples"),
//           doc.PiecewisePolynomial.LagrangeInterpolatingPolynomial.doc)
//       .def("derivative", &PiecewisePolynomial<T>::derivative,
//           py::arg("derivative_order") = 1,
//           doc.PiecewisePolynomial.derivative.doc)
//       .def("getPolynomialMatrix", &PiecewisePolynomial<T>::getPolynomialMatrix,
//           py::arg("segment_index"),
//           doc.PiecewisePolynomial.getPolynomialMatrix.doc)
//       .def("getPolynomial", &PiecewisePolynomial<T>::getPolynomial,
//           py::arg("segment_index"), py::arg("row") = 0, py::arg("col") = 0,
//           doc.PiecewisePolynomial.getPolynomial.doc)
//       .def("getSegmentPolynomialDegree",
//           &PiecewisePolynomial<T>::getSegmentPolynomialDegree,
//           py::arg("segment_index"), py::arg("row") = 0, py::arg("col") = 0,
//           doc.PiecewisePolynomial.getSegmentPolynomialDegree.doc)
//       .def("isApprox", &PiecewisePolynomial<T>::isApprox, py::arg("other"),
//           py::arg("tol"), py::arg("tol_type") = drake::ToleranceType::kRelative,
//           doc.PiecewisePolynomial.isApprox.doc)
//       .def("Reshape", &PiecewisePolynomial<T>::Reshape, py::arg("rows"),
//           py::arg("cols"), doc.PiecewisePolynomial.Reshape.doc)
//       .def("Block", &PiecewisePolynomial<T>::Block, py::arg("start_row"),
//           py::arg("start_col"), py::arg("block_rows"), py::arg("block_cols"),
//           doc.PiecewisePolynomial.Block.doc)
//       .def("ConcatenateInTime", &PiecewisePolynomial<T>::ConcatenateInTime,
//           py::arg("other"), doc.PiecewisePolynomial.ConcatenateInTime.doc)
//       .def("AppendCubicHermiteSegment",
//           &PiecewisePolynomial<T>::AppendCubicHermiteSegment, py::arg("time"),
//           py::arg("sample"), py::arg("sample_dot"),
//           doc.PiecewisePolynomial.AppendCubicHermiteSegment.doc)
//       .def("AppendFirstOrderSegment",
//           &PiecewisePolynomial<T>::AppendFirstOrderSegment, py::arg("time"),
//           py::arg("sample"),
//           doc.PiecewisePolynomial.AppendFirstOrderSegment.doc)
//       .def("RemoveFinalSegment", &PiecewisePolynomial<T>::RemoveFinalSegment,
//           doc.PiecewisePolynomial.RemoveFinalSegment.doc)
//       .def("ReverseTime", &PiecewisePolynomial<T>::ReverseTime,
//           doc.PiecewisePolynomial.ReverseTime.doc)
//       .def("ScaleTime", &PiecewisePolynomial<T>::ScaleTime, py::arg("scale"),
//           doc.PiecewisePolynomial.ScaleTime.doc)
//       .def("slice", &PiecewisePolynomial<T>::slice,
//           py::arg("start_segment_index"), py::arg("num_segments"),
//           doc.PiecewisePolynomial.slice.doc)
//       .def("shiftRight", &PiecewisePolynomial<T>::shiftRight, py::arg("offset"),
//           doc.PiecewisePolynomial.shiftRight.doc)
//       .def(py::self + py::self)
//       .def("setPolynomialMatrixBlock",
//           &PiecewisePolynomial<T>::setPolynomialMatrixBlock,
//           py::arg("replacement"), py::arg("segment_index"),
//           py::arg("row_start") = 0, py::arg("col_start") = 0,
//           doc.PiecewisePolynomial.setPolynomialMatrixBlock.doc);

//   py::class_<PiecewiseQuaternionSlerp<T>, PiecewiseTrajectory<T>>(
//       m, "PiecewiseQuaternionSlerp", doc.PiecewiseQuaternionSlerp.doc)
//       .def(py::init<>(), doc.PiecewiseQuaternionSlerp.ctor.doc_0args)
//       .def(py::init<const std::vector<double>&,
//                const std::vector<Quaternion<T>>&>(),
//           py::arg("breaks"), py::arg("quaternions"),
//           doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_quaternions)
//       .def(py::init<const std::vector<double>&,
//                const std::vector<Matrix3<T>>&>(),
//           py::arg("breaks"), py::arg("rotation_matrices"),
//           doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_rotation_matrices)
//       .def(py::init<const std::vector<double>&,
//                const std::vector<math::RotationMatrix<T>>&>(),
//           py::arg("breaks"), py::arg("rotation_matrices"),
//           doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_rotation_matrices)
//       .def(py::init<const std::vector<double>&,
//                const std::vector<AngleAxis<T>>&>(),
//           py::arg("breaks"), py::arg("angle_axes"),
//           doc.PiecewiseQuaternionSlerp.ctor.doc_2args_breaks_angle_axes)
//       .def("Append",
//           py::overload_cast<const T&, const Quaternion<T>&>(
//               &PiecewiseQuaternionSlerp<T>::Append),
//           py::arg("time"), py::arg("quaternion"),
//           doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_quaternion)
//       .def("Append",
//           py::overload_cast<const T&, const math::RotationMatrix<T>&>(
//               &PiecewiseQuaternionSlerp<T>::Append),
//           py::arg("time"), py::arg("rotation_matrix"),
//           doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_rotation_matrix)
//       .def("Append",
//           py::overload_cast<const T&, const AngleAxis<T>&>(
//               &PiecewiseQuaternionSlerp<T>::Append),
//           py::arg("time"), py::arg("angle_axis"),
//           doc.PiecewiseQuaternionSlerp.Append.doc_2args_time_angle_axis)
//       .def("orientation", &PiecewiseQuaternionSlerp<T>::orientation,
//           py::arg("time"), doc.PiecewiseQuaternionSlerp.orientation.doc)
//       .def("angular_velocity", &PiecewiseQuaternionSlerp<T>::angular_velocity,
//           py::arg("time"), doc.PiecewiseQuaternionSlerp.angular_velocity.doc)
//       .def("angular_acceleration",
//           &PiecewiseQuaternionSlerp<T>::angular_acceleration, py::arg("time"),
//           doc.PiecewiseQuaternionSlerp.angular_acceleration.doc);
// }
}  // namespace

PYBIND11_MODULE(trajectories, m) {
//   DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
