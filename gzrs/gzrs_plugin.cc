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

#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include "gzrs_plugin.hh"

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RealSensePlugin)

RealSensePlugin::RealSensePlugin()
{
    this->depth_cam = NULL;
    this->ired1_cam = NULL;
    this->ired2_cam = NULL;
}

RealSensePlugin::~RealSensePlugin()
{
    delete [] depth_map;
}

void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Output the name of the model
    std::cout << "\nThe rs_camera plugin is attach to model \""
              << _model->GetName() << "\"\n";

    // Store a pointer to the this model
    this->rs_model = _model;

    // Store a pointer to the world
    this->world = this->rs_model->GetWorld();

    // Sensors Manager
    this->smanager = sensors::SensorManager::Instance();

    // Get Cameras Renderers
    this->depth_cam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                          this->smanager->GetSensor("depth"))
                          ->DepthCamera();

    // TODO: Infrared Cameras Renderers

    // Check if camera renderers have been found successfuly
    if (!this->depth_cam) {
        std::cerr << "Depth Camera has not been found\n";
    }

    // TODO: Check Infrared Camera Renderers

    // Allocate Memory for the real sense depth map
    this->depth_map =
        new uint16_t[depth_cam->ImageWidth() * depth_cam->ImageHeight()];

    // Setup Publishers
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->GetName());

    this->depth_view_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs_stream_depth_view", 1,
        DEPTH_PUB_FREQ_HZ);

    this->depth_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs_stream_depth", 1,
        DEPTH_PUB_FREQ_HZ);

    // TODO: Infrared Camera and Color Camera Publishers

    // Listen to depth camera new frame event
    this->new_depth_frame_conn =
        this->depth_cam->ConnectNewDepthFrame(std::bind(
            &RealSensePlugin::OnNewDepthFrame, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

    // TODO: Infrared Camera Listeners

    // Listen to the update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RealSensePlugin::OnUpdate, this));
}

void RealSensePlugin::OnNewDepthFrame(const float *_image, unsigned int _width,
                                      unsigned int _height,
                                      unsigned int /*_depth*/,
                                      const std::string & /*_format*/)
{
    msgs::ImageStamped msg;

    // Pack viewable image message
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());
    msg.mutable_image()->set_width(this->depth_cam->ImageWidth());
    msg.mutable_image()->set_height(this->depth_cam->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);
    msg.mutable_image()->set_step(this->depth_cam->ImageWidth() *
                                  this->depth_cam->ImageDepth());
    msg.mutable_image()->set_data(this->depth_cam->DepthData(),
                                  sizeof(float) * msg.image().width() *
                                      msg.image().height());

    // Publish viewable image
    this->depth_view_pub->Publish(msg);

    // Convert depth data to realsense scaled depth map
    unsigned int depth_map_dim =
        this->depth_cam->ImageWidth() * this->depth_cam->ImageHeight();

    for (unsigned int i = 0; i < depth_map_dim; i++) {
        // Convert from meters to 0 - UINT16_MAX
        if (_image[i] <= this->depth_cam->NearClip() ||
            _image[i] >= this->depth_cam->FarClip()) {
            depth_map[i] = 0;
        } else {
            depth_map[i] =
                (_image[i] - this->depth_cam->NearClip()) * UINT16_MAX /
                (this->depth_cam->FarClip() - this->depth_cam->NearClip());
        }
    }

    // Pack realsense scaled depth map 
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());
    msg.mutable_image()->set_width(this->depth_cam->ImageWidth());
    msg.mutable_image()->set_height(this->depth_cam->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
    msg.mutable_image()->set_step(this->depth_cam->ImageWidth() *
                                  this->depth_cam->ImageDepth());
    msg.mutable_image()->set_data(this->depth_map, sizeof(uint16_t) *
                                                       msg.image().width() *
                                                       msg.image().height());

    // Publish realsense scaled depth map
    this->depth_pub->Publish(msg);

    // TODO: Publish Color Camera
}

void RealSensePlugin::OnUpdate()
{
    return;
}
