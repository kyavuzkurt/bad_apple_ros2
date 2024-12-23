# Bad Apple ROS2 ASCII Publisher

I couldn't find a implementation of the "Bad Apple" in ROS2, so i created one out of boredom. Enjoy!

## Installation

1. **Clone the repository inside your ROS2 workspace**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/kyavuzkurt/bad_apple_ros2.git
    cd ..
    colcon build --packages-select bad_apple
    ```

## Usage

1. **Generate ASCII frames**
    Frame files exceeds github file limit, so i'm not uploading them. But you can generate them yourself easily.

    Go to the `bad_apple` directory and run the `make_ascii.py` script.
    ```bash
    cd bad_apple
    python3 make_ascii.py
    ```

2. **Run the ROS2 node**
    Run the launch file to start the ASCII animation on a full screen terminal.
    ```bash
    ros2 launch bad_apple bad_apple.launch.py
    ```

Here is a demonstration of the ASCII animation:

![Bad Apple ASCII Animation](./demo.gif)

## License

MIT License
