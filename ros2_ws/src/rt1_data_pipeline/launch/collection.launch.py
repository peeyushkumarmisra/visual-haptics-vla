# Starts every Phase 2 node except the collector (run that separately so you
# can watch it and stop it cleanly). The simulation must already be running.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    split = LaunchConfiguration("split")
    stage = LaunchConfiguration("stage")
    sim_time = {"use_sim_time": True}

    def pipeline_node(executable, extra_params=None):
        return Node(package="rt1_data_pipeline", executable=executable, output="screen",
                    parameters=[sim_time] + ([extra_params] if extra_params else []))

    return LaunchDescription([
        DeclareLaunchArgument("split", default_value="train"),
        DeclareLaunchArgument("stage", default_value="A"),
        pipeline_node("dual_cam_stitcher_node"),
        pipeline_node("probe_metrics_node"),
        pipeline_node("mystery_spawner.py", {"split": split, "stage": stage}),
        pipeline_node("probe_controller_node"),
        pipeline_node("episode_logger.py", {"split": split}),
    ])