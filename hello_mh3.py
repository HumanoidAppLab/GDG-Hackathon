"""Demonstrate the MH3 Genesis Python API in a single, runnable example script."""

from __future__ import annotations

import numpy as np

from mh3_genesis import MH3ControlPanel, MH3Simulation, SimulatorConfig


def main() -> None:
    """Run an end-to-end demonstration of the MH3 Genesis simulation API.
    :return: None when the demonstration script completes.
    :rtype: None
    """
    # Load configuration from the default YAML file.
    config = SimulatorConfig.from_yaml("config/mh3.yaml")

    # Instantiate the MH3 simulation controller.
    sim = MH3Simulation(config)

    # Register a scene hook to add custom geometry before Genesis builds the scene.
    def add_custom_world(simulation: MH3Simulation) -> None:
        """Add custom scene entities before physics compilation.
        :param simulation: MH3 simulation instance providing access to the Genesis scene.
        :type simulation: MH3Simulation
        :return: None after adding custom entities.
        :rtype: None
        """
        # Add a colored reference block onto the ground plane in front of the robot.
        simulation.scene.add_entity(
            simulation.gs.morphs.Box(
                pos=(1.0, 0.0, 0.25),
                size=(0.3, 0.3, 0.5),
            )
        )

    # Attach the scene hook prior to calling build().
    sim.add_scene_hook(add_custom_world)

    # Build the Genesis scene, compile kernels, and spawn the MH3 articulation.
    sim.build()

    # Discover and print robot properties, controllable joints, links, and camera sensors.
    print(f"Compute Backend: {sim.compute_backend}")
    print(f"Controllable joints ({len(sim.joint_names)}): {sim.joint_names}")
    print(f"Robot links ({len(sim.link_names)}): {sim.link_names}")
    print(f"Available cameras ({len(sim.camera_names)}): {sim.camera_names}")

    # Instantly set the torso slider position so the simulation starts at that height without waiting.
    # Note: sim.home_positions preserves the true neutral startup home pose, so returning home still works properly.
    sim.set_joint_positions({"slider": 0.1}, immediate=True)

    # Set dynamic joint position targets for the head (actuated with physics once stepping begins).
    sim.set_joint_positions(
        {
            "head_yaw": 0.2,
            "head_pitch": -0.2,
        }
    )

    # Close the fingers slightly on the left hand.
    sim.set_joint_positions(
        {
            "left_index": 0.6,
            "left_middle": 0.6,
            "left_thumb_1": 0.5,
        }
    )

    # Command mobile base velocity (linear in m/s, angular in rad/s).
    sim.drive_base(linear_mps=0.1, angular_radps=0.0)

    # Step the simulation forward to let initial movements begin.
    for _ in range(30):
        sim.step()

    # Stop the mobile base.
    sim.stop_base()

    # Inspect current joint positions, velocities, and applied efforts.
    joint_state = sim.get_joint_state(["head_yaw", "head_pitch", "slider"])
    print("\n--- Joint State ---")
    for name, pos, vel in zip(joint_state.names, joint_state.positions, joint_state.velocities, strict=True):
        print(f"  {name}: pos={pos:.3f}, vel={vel:.3f}")

    # Solve inverse kinematics for the left arm.
    # Supports target position, orientation (Euler angles or quaternion), and reference frames (default: world space).
    # Passing position=None solves for orientation while automatically holding the current end-effector position.
    left_arm_joints = [f"left_{i}" for i in range(1, 8)]
    target_pos = (0.45, 0.25, 0.85)
    print(f"\n--- Inverse Kinematics ---\nSolving IK for 'left_gripper_base' to {target_pos}...")
    solution = sim.solve_ik(
        end_effector="left_gripper_base",
        position=target_pos,
        joints=left_arm_joints,
        apply=True,
    )
    print(f"Solved configuration vector length: {len(solution)}")

    # Capture vision sensor data from the head camera.
    frame = sim.capture_camera("head_left", rgb=True, depth=True)
    if frame.rgb is not None and frame.depth is not None:
        print(f"\n--- Camera Capture ---\nRGB shape: {frame.rgb.shape}, Depth shape: {frame.depth.shape}")

    # Capture a 3D point cloud and depth validity mask.
    points, valid_depth = sim.capture_pointcloud("head_left")
    valid_points_count = int(np.sum(valid_depth))
    print(f"Point cloud points: {points.shape}, Valid depth points: {valid_points_count}")

    # Register a step hook callback to coordinate behavior over time.
    def behavior_controller(simulation: MH3Simulation) -> None:
        """Run a simple timed routine during stepping.
        :param simulation: MH3 simulation instance.
        :type simulation: MH3Simulation
        :return: None after updating controls for this step.
        :rtype: None
        """
        # Nod head down after 0.5 simulated seconds.
        if 0.5 <= simulation.sim_time < 1.0:
            simulation.set_joint_positions({"head_pitch": -0.3})
        # Return head to level between 1.0 and 1.5 simulated seconds.
        elif 1.0 <= simulation.sim_time < 1.5:
            simulation.set_joint_positions({"head_pitch": 0.0})

    # Register the step hook callback.
    sim.add_step_hook(behavior_controller)

    # Start the browser dashboard control panel.
    panel = MH3ControlPanel(sim)
    panel.start()
    sim.add_step_hook(panel.update)

    # Run the simulation indefinitely so the viewer stays open for observation (stop with Ctrl+C).
    # To run for a fixed number of steps instead, pass steps (e.g. sim.run(steps=300)).
    # To run for a fixed duration of simulated time (e.g. 5.0 seconds), calculate steps as int(5.0 / sim.config.dt).
    print("\nRunning simulation indefinitely (press Ctrl+C in terminal to stop)...")
    try:
        sim.run()
    finally:
        # Shut down the web dashboard server cleanly on exit.
        panel.close()


if __name__ == "__main__":
    # Execute the demonstration routine.
    main()
