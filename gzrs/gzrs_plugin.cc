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

#include "gzrs_plugin.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(RealSensePlugin)

RealSensePlugin::RealSensePlugin()
{
    this->depth_cam = NULL;
    this->ired1_cam = NULL;
    this->ired2_cam = NULL;
    this->color_cam = NULL;
}

RealSensePlugin::~RealSensePlugin()
{
    delete[] depth_map;
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
                          this->smanager->GetSensor(DEPTH_CAMERA_NAME))
                          ->DepthCamera();
    this->ired1_cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                          this->smanager->GetSensor(IRED1_CAMERA_NAME))
                          ->Camera();
    this->ired2_cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                          this->smanager->GetSensor(IRED2_CAMERA_NAME))
                          ->Camera();
    this->color_cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                          this->smanager->GetSensor(COLOR_CAMERA_NAME))
                          ->Camera();

    // Check if camera renderers have been found successfuly
    if (!this->depth_cam) {
        std::cerr << "Depth Camera has not been found\n";
        return;
    }
    if (!this->ired1_cam) {
        std::cerr << "InfraRed Camera 1 has not been found\n";
        return;
    }
    if (!this->ired2_cam) {
        std::cerr << "InfraRed Camera 2 has not been found\n";
        return;
    }
    if (!this->color_cam) {
        std::cerr << "Color Camera has not been found\n";
        return;
    }

    // Setup Transport Node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->GetName());

    // Setup Publishers
    this->depth_view_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs/stream/" + DEPTH_CAMERA_TOPIC +
            "_view",
        1, DEPTH_PUB_FREQ_HZ);
    this->depth_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs/stream/" + DEPTH_CAMERA_TOPIC,
        1, DEPTH_PUB_FREQ_HZ);
    this->ired1_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs/stream/" + IRED1_CAMERA_TOPIC,
        1, DEPTH_PUB_FREQ_HZ);
    this->ired2_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs/stream/" + IRED2_CAMERA_TOPIC,
        1, DEPTH_PUB_FREQ_HZ);
    this->color_pub = this->node->Advertise<msgs::ImageStamped>(
        "~/" + this->rs_model->GetName() + "/rs/stream/" + COLOR_CAMERA_TOPIC,
        1, DEPTH_PUB_FREQ_HZ);

    // TODO: Publish Depth Scale

    // Listen to depth camera new frame event
    this->new_depth_frame_conn =
        this->depth_cam->ConnectNewDepthFrame(std::bind(
            &RealSensePlugin::OnNewDepthFrame, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

    this->new_ired1_frame_conn = this->ired1_cam->ConnectNewImageFrame(
        std::bind(&RealSensePlugin::OnNewIR1Frame, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5));

    this->new_ired2_frame_conn = this->ired2_cam->ConnectNewImageFrame(
        std::bind(&RealSensePlugin::OnNewIR2Frame, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4, std::placeholders::_5));

    this->new_color_frame_conn =
        this->ired1_cam->ConnectNewImageFrame(std::bind(
            &RealSensePlugin::OnNewColorFrame, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

    // Listen to the update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RealSensePlugin::OnUpdate, this));
}

void RealSensePlugin::OnNewIR1Frame(const unsigned char *_image,
                                    unsigned int _width, unsigned int _height,
                                    unsigned int /*_depth*/,
                                    const std::string & /*_format*/)
{
    msgs::ImageStamped msg;

    // Set Simulation Time
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());

    // Set Image Dimensions
    msg.mutable_image()->set_width(this->ired1_cam->ImageWidth());
    msg.mutable_image()->set_height(this->ired1_cam->ImageHeight());

    // Set Image Pixel Format
    msg.mutable_image()->set_pixel_format(
        common::Image::ConvertPixelFormat(this->ired1_cam->ImageFormat()));

    // Set Image Data
    msg.mutable_image()->set_step(this->ired1_cam->ImageWidth() *
                                  this->ired1_cam->ImageDepth());
    msg.mutable_image()->set_data(this->ired1_cam->ImageData(),
                                  this->ired1_cam->ImageDepth() *
                                      this->ired1_cam->ImageWidth() *
                                      this->ired1_cam->ImageHeight());

    // Publish realsense infrared stream
    this->ired1_pub->Publish(msg);
}

void RealSensePlugin::OnNewIR2Frame(const unsigned char *_image,
                                    unsigned int _width, unsigned int _height,
                                    unsigned int /*_depth*/,
                                    const std::string & /*_format*/)
{
    msgs::ImageStamped msg;

    // Set Simulation Time
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());

    // Set Image Dimensions
    msg.mutable_image()->set_width(this->ired2_cam->ImageWidth());
    msg.mutable_image()->set_height(this->ired2_cam->ImageHeight());

    // Set Image Pixel Format
    msg.mutable_image()->set_pixel_format(
        common::Image::ConvertPixelFormat(this->ired2_cam->ImageFormat()));

    // Set Image Data
    msg.mutable_image()->set_step(this->ired2_cam->ImageWidth() *
                                  this->ired2_cam->ImageDepth());
    msg.mutable_image()->set_data(this->ired2_cam->ImageData(),
                                  this->ired2_cam->ImageDepth() *
                                      this->ired2_cam->ImageWidth() *
                                      this->ired2_cam->ImageHeight());

    // Publish realsense infrared2 stream
    this->ired2_pub->Publish(msg);
}

void RealSensePlugin::OnNewColorFrame(const unsigned char *_image,
                                      unsigned int _width, unsigned int _height,
                                      unsigned int /*_depth*/,
                                      const std::string & /*_format*/)
{
    msgs::ImageStamped msg;

    // Set Simulation Time
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());

    // Set Image Dimensions
    msg.mutable_image()->set_width(this->color_cam->ImageWidth());
    msg.mutable_image()->set_height(this->color_cam->ImageHeight());

    // Set Image Pixel Format
    msg.mutable_image()->set_pixel_format(
        common::Image::ConvertPixelFormat(this->color_cam->ImageFormat()));

    // Set Image Data
    msg.mutable_image()->set_step(this->color_cam->ImageWidth() *
                                  this->color_cam->ImageDepth());
    msg.mutable_image()->set_data(this->color_cam->ImageData(),
                                  this->color_cam->ImageDepth() *
                                      this->color_cam->ImageWidth() *
                                      this->color_cam->ImageHeight());

    // Publish realsense color stream
    this->color_pub->Publish(msg);
}

void RealSensePlugin::OnNewDepthFrame(const float *_image, unsigned int _width,
                                      unsigned int _height,
                                      unsigned int /*_depth*/,
                                      const std::string & /*_format*/)
{
    // Allocate Memory for the real sense depth map
    // TODO: Allocate this memory more intelligently. Also make sure you free it :)
    if (!this->depth_map) {
        this->depth_map =
            new uint16_t[depth_cam->ImageWidth() * depth_cam->ImageHeight()];
    }

    msgs::ImageStamped msg;

    const float *depth_data_float = this->depth_cam->DepthData();

    // Pack viewable image message
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());
    msg.mutable_image()->set_width(this->depth_cam->ImageWidth());
    msg.mutable_image()->set_height(this->depth_cam->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);
    msg.mutable_image()->set_step(this->depth_cam->ImageWidth() *
                                  this->depth_cam->ImageDepth());
    msg.mutable_image()->set_data(depth_data_float, sizeof(*depth_data_float) *
                                                        msg.image().width() *
                                                        msg.image().height());

    // Publish viewable image
    this->depth_view_pub->Publish(msg);

    // Convert depth data to realsense scaled depth map
    unsigned int depth_map_dim =
        this->depth_cam->ImageWidth() * this->depth_cam->ImageHeight();

    for (unsigned int i = 0; i < depth_map_dim; i++) {

        // Check clipping and overflow
        if (depth_data_float[i] < DEPTH_NEAR_CLIP_M ||
            depth_data_float[i] > DEPTH_FAR_CLIP_M ||
            depth_data_float[i] > DEPTH_SCALE_M * UINT16_MAX ||
            depth_data_float[i] < 0) {
            depth_map[i] = 0;
        } else {
            depth_map[i] = (uint16_t)(depth_data_float[i] / DEPTH_SCALE_M);
        }
    }

    // Pack realsense scaled depth map
    msgs::Set(msg.mutable_time(), this->world->GetSimTime());
    msg.mutable_image()->set_width(this->depth_cam->ImageWidth());
    msg.mutable_image()->set_height(this->depth_cam->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
    msg.mutable_image()->set_step(this->depth_cam->ImageWidth() *
                                  this->depth_cam->ImageDepth());
    msg.mutable_image()->set_data(this->depth_map, sizeof(*this->depth_map) *
                                                       msg.image().width() *
                                                       msg.image().height());

    // Publish realsense scaled depth map
    this->depth_pub->Publish(msg);
}

void RealSensePlugin::OnUpdate()
{
    return;
}
