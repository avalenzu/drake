# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_binary",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
)

drake_cc_library(
    name = "pick_and_place",
    srcs = [
        "action.cc",
        "pick_and_place_state_machine.cc",
        "world_state.cc",
    ],
    hdrs = [
        "action.h",
        "pick_and_place_state_machine.h",
        "world_state.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcmtypes:schunk",
        "//drake/manipulation/planner:constraint_relaxing_ik",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/util:lcm_util",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "ik_traj_scratch",
    srcs = [
        "ik_traj_scratch.cc",
    ],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":pick_and_place",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/lcm",
        "//drake/common:find_resource",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:zero_order_hold",
        "//drake/systems/primitives:signal_logger",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_googletest(
    name = "pick_and_place_state_machine_test",
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":pick_and_place",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/lcm",
        "//drake/common:find_resource",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:zero_order_hold",
        "//drake/systems/primitives:signal_logger",
    ],
)

drake_cc_googletest(
    name = "world_state_test",
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":pick_and_place",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
    ],
)

add_lint_tests()
