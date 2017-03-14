# Gazebo RealSense Plugin #

<a href="https://scan.coverity.com/projects/01org-gazebo-realsense">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/11940/badge.svg"/>
</a>

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
    ```

## Install ##

The plugin binaries will be installed so that Gazebo finds them. Also the
needed models will be copied to the default gazebo models folder.

    make install

[comment] Default plugin path arch: /usr/local/lib64/gazebo-8/plugins/.
[comment] Default model path arch: ~/.gazebo/models. Make install is copying
[comment]   the models to this place changing ownership to root. We still need
[comment]   to fix it.

## Run ##

    ```
    gazebo --verbose
    ```

2. Insert the realsense model into the simulation.
 - Click on the INSERT pane, select the realsense model and move it to the world.

## Interaction ##

The Plugin publishes the simulated streams to ImageStamped topics. They can be inspected with the Gazebo Topic Viewer (Window->Topic Visualization) under ~/realsense/rs/stream/TOPIC_NAME.
