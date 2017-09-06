# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "kinematic_trajectory_optimization",
    srcs = ["kinematic_trajectory_optimization.cc"],
    hdrs = ["kinematic_trajectory_optimization.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/primitives:linear_system",
        "//drake/systems/trajectory_optimization:multiple_shooting",
        "//drake/systems/trajectory_optimization:direct_transcription",
    ],
)

drake_cc_library(
    name = "constraint_relaxing_ik",
    srcs = ["constraint_relaxing_ik.cc"],
    hdrs = ["constraint_relaxing_ik.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "robot_plan_interpolator",
    srcs = ["robot_plan_interpolator.cc"],
    hdrs = ["robot_plan_interpolator.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "iiwa_scratch",
    srcs = ["iiwa_scratch.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":kinematic_trajectory_optimization",
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:linear_system",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/primitives:trajectory_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "constraint_relaxing_ik_test",
    size = "medium",
    srcs = ["test/constraint_relaxing_ik_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    # TODO(sam.creasey) This test currently times out on Mac with
    # IPOPT.  Once IPOPT is tuned to provide better/faster IK results
    # (see https://github.com/RobotLocomotion/drake/issues/3128 ) we
    # should enable this test.
    tags = ["snopt"],
    deps = [
        ":constraint_relaxing_ik",
        "//drake/common:find_resource",
    ],
)

drake_cc_googletest(
    name = "robot_plan_interpolator_test",
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":robot_plan_interpolator",
        "//drake/common:find_resource",
        "//drake/systems/framework",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_googletest(
    name = "kinematic_trajectory_optimization_test",
    deps = [
        ":kinematic_trajectory_optimization",
    ],
)

add_lint_tests()
