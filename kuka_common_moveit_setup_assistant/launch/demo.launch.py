from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("kuka_kr3r540", package_name="kuka_common_moveit_setup_assistant").to_moveit_configs()
    return generate_demo_launch(moveit_config)
