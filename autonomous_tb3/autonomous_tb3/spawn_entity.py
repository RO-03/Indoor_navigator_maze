
# Spawner script for Gazebo Harmonic (ROS 2 Jazzy)
# Uses ros_gz_sim 'create' executable instead of Gazebo Classic spawn_entity service.
#
# Usage: ros2 run autonomous_tb3 sdf_spawner <sdf_path> <name> [x] [y]

'''
This Python script is used to spawn Gazebo Harmonic models in a ROS 2 Jazzy environment.
It takes the path of an SDF model file and the name of the model as arguments,
and can optionally take the X and Y coordinates of the initial position.
It invokes `ros2 run ros_gz_sim create` under the hood, which is the Gazebo
Harmonic equivalent of the old gazebo_msgs SpawnEntity service.
'''
import sys
import subprocess
import rclpy


def main():
    argv = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node('Spawning_Node')

    if len(argv) < 2:
        node.get_logger().error('Usage: sdf_spawner <sdf_path> <model_name> [x] [y]')
        rclpy.shutdown()
        return

    sdf_path = argv[0]
    name = argv[1]
    x = argv[2] if len(argv) > 2 else '0.0'
    y = argv[3] if len(argv) > 3 else '0.0'
    # Beer is spawned elevated on a surface (table height)
    z = '1.5' if name == 'beer' else '0.01'

    node.get_logger().info(f"Spawning '{name}' from {sdf_path} at ({x}, {y}, {z})")

    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-file', sdf_path,
        '-x', x,
        '-y', y,
        '-z', z,
    ]

    result = subprocess.run(cmd)

    if result.returncode == 0:
        node.get_logger().info(f"Successfully spawned '{name}'.")
    else:
        node.get_logger().error(f"Failed to spawn '{name}'. Return code: {result.returncode}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()