import sys
import os
from launch import LaunchService
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 run_launch.py <path_to_launch_file> [launch_arguments...]")
        sys.exit(1)

    launch_file_path = os.path.abspath(sys.argv[1])
    
    # Parse extra launch arguments from command line: arg:=value
    launch_args = {}
    for arg in sys.argv[2:]:
        if ':=' in arg:
            k, v = arg.split(':=', 1)
            launch_args[k] = v

    print(f"Loading launch file: {launch_file_path}")
    print(f"Arguments: {launch_args}")

    # Create the launch service
    ls = LaunchService()
    
    # Include the specified launch file
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments=launch_args.items()
        )
    ])
    
    ls.include_launch_description(ld)
    
    # Run the launch service
    print("Launching ROS 2 Simulation...")
    try:
        ls.run()
    except KeyboardInterrupt:
        print("Received Ctrl+C, shutting down...")

if __name__ == '__main__':
    main()
