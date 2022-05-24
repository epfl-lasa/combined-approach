#!/usr/bin/python3
import numpy as np

# Various Tools
from vartools.dynamical_systems import LinearSystem

from dynamic_obstacle_avoidance.containers import ObstacleContainer
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.avoidance import ModulationAvoider


def test_single_circle_3d():
    my_container = ObstacleContainer()

    my_container.append(
        Ellipse(
            center_position=np.array([0.5, 0, 0.25]),
            axes_length=np.array([0.3, 0.3, 0.3]),
            linear_velocity=np.zeros(3),
            margin_absolut=0,
            tail_effect=False,
            repulsion_coeff=1.0,
        ),
    )

    target_position = np.array([0.5, 0.5, 0])

    ds_position_only = LinearSystem(
        attractor_position=target_position, maximum_velocity=0.25
    )

    modulator = ModulationAvoider(obstacle_environment=my_container)

    position = np.array([0, 0, 0.183])
    vel_init = ds_position_only.evaluate(position)
    print("vel_init", vel_init)

    vel_modu = modulator.avoid(position, velocity=vel_init)
    print("vel_modu", vel_modu)


def test_state_representation():
    # Control-Libraries
    import state_representation as sr
    from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

    # Control Libraries Based Approach
    target = sr.CartesianPose("world")
    target.set_position(target_position)
    target.set_orientation([0, 0, 0, 1])

    _ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)

    _ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    _ds.set_parameter_value(
        "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
    )
    _ds.set_parameter_value(
        "gain", [50, 50, 50, 10, 10, 10], sr.ParameterType.DOUBLE_ARRAY
    )

    cp_pose = sr.CartesianPose()
    cp_pose.set_position(position)

    init_twist = sr.CartesianTwist(_ds.evaluate(cp_pose))
    init_twist.clamp(0.25, 0.25)
    init_linear_vel = init_twist.get_linear_velocity()
    print("vel_init [control libraries]", init_linear_vel)


if (__name__) == "__main__":
    test_single_circle_3d()
