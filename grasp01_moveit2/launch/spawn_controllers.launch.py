from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("grasp01_description", package_name="grasp01_moveit2").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
