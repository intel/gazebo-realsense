# Gazebo RealSense Plugin #

A RealSense Camera Gazebo plugin.

This Gazebo plugin simulates a RealSense camera by publishing the 4 main
RealSense streams: Depth, Infrared, Infrared2 and Color. It is associated to a
RealSense model that is providade in ./models.

## Requirements ##
    * Gazebo 7.0+

## Build ##

1. Create a build folder and make using CMAKE as follows:

    ```
    mkdir ./gzrs/build
    cd gzrs/build
    cmake ..
    make
    cd -
    ```
## Run ##

1. Set environment variables and open Gazebo. 

    ```
    GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PWD}/gzrs/build GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PWD}/models gazebo --verbose
    ```

2. Insert the realsense model into the simulation.
 - Click on the INSERT pane, select the realsense model and move it to the world.

## Interaction ##

The Plugin publishes the simulated streams to ImageStamped topics. They can be inspected with the Gazebo Topic Viewer (Window->Topic Visualization) under ~/realsense/rs/stream/TOPIC_NAME.
