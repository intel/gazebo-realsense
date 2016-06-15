/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{

class GAZEBO_VISIBLE RealSensePlugin : public ModelPlugin
{

  public:
    RealSensePlugin();
    ~RealSensePlugin();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnNewDepthFrame(const float *_image, unsigned int _width,
                                 unsigned int _height, unsigned int _depth,
                                 const std::string &_format);
    virtual void OnNewIR1Frame(const unsigned char *_image, unsigned int _width,
                               unsigned int _height, unsigned int /*_depth*/,
                               const std::string &
                               /*_format*/);
    virtual void OnNewIR2Frame(const unsigned char *_image, unsigned int _width,
                               unsigned int _height, unsigned int /*_depth*/,
                               const std::string &
                               /*_format*/);
    virtual void OnNewColorFrame(const unsigned char *_image,
                                 unsigned int _width, unsigned int _height,
                                 unsigned int /*_depth*/, const std::string &
                                 /*_format*/);

  private:

    physics::ModelPtr rs_model;
    physics::WorldPtr world;

    // Renderers
    rendering::DepthCameraPtr depth_cam;
    rendering::CameraPtr color_cam;
    rendering::CameraPtr ired1_cam;
    rendering::CameraPtr ired2_cam;

    sensors::SensorManager *smanager;

    transport::NodePtr node;
    transport::PublisherPtr depth_pub;
    transport::PublisherPtr depth_view_pub;
    transport::PublisherPtr color_pub;
    transport::PublisherPtr ired1_pub;
    transport::PublisherPtr ired2_pub;
    
    event::ConnectionPtr new_depth_frame_conn;
    event::ConnectionPtr new_ired1_frame_conn;
    event::ConnectionPtr new_ired2_frame_conn;
    event::ConnectionPtr new_color_frame_conn;
    event::ConnectionPtr update_connection;

    uint16_t *depth_map = NULL;

    void OnUpdate();
};
}

#endif
