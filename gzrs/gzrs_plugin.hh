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

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>

namespace gazebo
{

  /// \brief A plugin that simulates Real Sense camera streams.
  class GAZEBO_VISIBLE RealSensePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public:
    RealSensePlugin();

    /// \brief Destructor.
    public:
    ~RealSensePlugin();

    // Documentation Inherited.
    public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that publishes a received Depth Frame as an ImageStamped
    /// message.
    public:
    virtual void OnNewDepthFrame(const float *_image, unsigned int _width,
                                 unsigned int _height, unsigned int _depth,
                                 const std::string &_format);

    /// \brief Callback that publishes a received Infrared Frame as an
    /// ImageStamped message.
    public:
    virtual void OnNewIR1Frame(const unsigned char *_image, unsigned int _width,
                               unsigned int _height, unsigned int /*_depth*/,
                               const std::string &
                               /*_format*/) const;

    /// \brief Callback that publishes a received Infrared2 Frame as an
    /// ImageStamped message.
    public:
    virtual void OnNewIR2Frame(const unsigned char *_image, unsigned int _width,
                               unsigned int _height, unsigned int /*_depth*/,
                               const std::string &
                               /*_format*/) const;

    /// \brief Callback that publishes a received Color Frame as an ImageStamped
    /// message.
    public:
    virtual void OnNewColorFrame(const unsigned char *_image,
                                 unsigned int _width, unsigned int _height,
                                 unsigned int /*_depth*/, const std::string &
                                 /*_format*/) const;

    /// \brief Pointer to the model containing the plugin.
    protected:
    physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    protected:
    physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    protected:
    rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    protected:
    rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    protected:
    rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    protected:
    rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the world's Sensor Manager.
    protected:
    sensors::SensorManager *smanager;

    /// \brief Pointer to the transport Node.
    protected:
    transport::NodePtr node;

    /// \brief Pointer to the Depth Publisher.
    protected:
    transport::PublisherPtr depthPub;

    /// \brief Pointer to the DepthView Publisher.
    protected:
    transport::PublisherPtr depthViewPub;

    /// \brief Pointer to the Color Publisher.
    protected:
    transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    protected:
    transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    protected:
    transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    protected:
    event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    protected:
    event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    protected:
    event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    protected:
    event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    protected:
    event::ConnectionPtr updateConnection;

    /// \brief Array with converted Depth Map data.
    protected:
    uint16_t *depthMap = NULL;

    /// \brief Callback for the World Update event.
    protected:
    void OnUpdate();
  };
}
#endif

