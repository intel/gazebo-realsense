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

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(RealSensePlugin)

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin()
{
  this->depthCam = NULL;
  this->ired1Cam = NULL;
  this->ired2Cam = NULL;
  this->colorCam = NULL;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin()
{
  delete[] depthMap;
}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Output the name of the model
  std::cout << "\nThe rs_camera plugin is attach to model \""
            << _model->GetName() << "\"\n";

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  this->smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                       this->smanager->GetSensor(DEPTH_CAMERA_NAME))
                       ->DepthCamera();
  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       this->smanager->GetSensor(IRED1_CAMERA_NAME))
                       ->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       this->smanager->GetSensor(IRED2_CAMERA_NAME))
                       ->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       this->smanager->GetSensor(COLOR_CAMERA_NAME))
                       ->Camera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam)
  {
    std::cerr << "Depth Camera has not been found\n";
    return;
  }
  if (!this->ired1Cam)
  {
    std::cerr << "InfraRed Camera 1 has not been found\n";
    return;
  }
  if (!this->ired2Cam)
  {
    std::cerr << "InfraRed Camera 2 has not been found\n";
    return;
  }
  if (!this->colorCam)
  {
    std::cerr << "Color Camera has not been found\n";
    return;
  }

  // Setup Transport Node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  // Setup Publishers
  this->depthViewPub = this->node->Advertise<msgs::ImageStamped>(
      "~/" + this->rsModel->GetName() + "/rs/stream/" + DEPTH_CAMERA_TOPIC +
          "_view",
      1, DEPTH_PUB_FREQ_HZ);
  this->depthPub = this->node->Advertise<msgs::ImageStamped>(
      "~/" + this->rsModel->GetName() + "/rs/stream/" + DEPTH_CAMERA_TOPIC, 1,
      DEPTH_PUB_FREQ_HZ);
  this->ired1Pub = this->node->Advertise<msgs::ImageStamped>(
      "~/" + this->rsModel->GetName() + "/rs/stream/" + IRED1_CAMERA_TOPIC, 1,
      DEPTH_PUB_FREQ_HZ);
  this->ired2Pub = this->node->Advertise<msgs::ImageStamped>(
      "~/" + this->rsModel->GetName() + "/rs/stream/" + IRED2_CAMERA_TOPIC, 1,
      DEPTH_PUB_FREQ_HZ);
  this->colorPub = this->node->Advertise<msgs::ImageStamped>(
      "~/" + this->rsModel->GetName() + "/rs/stream/" + COLOR_CAMERA_TOPIC, 1,
      DEPTH_PUB_FREQ_HZ);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(
      std::bind(&RealSensePlugin::OnNewIR1Frame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(
      std::bind(&RealSensePlugin::OnNewIR2Frame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  this->newColorFrameConn = this->ired1Cam->ConnectNewImageFrame(
      std::bind(&RealSensePlugin::OnNewColorFrame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewIR1Frame(const unsigned char *_image,
                                    unsigned int _width, unsigned int _height,
                                    unsigned int /*_depth*/,
                                    const std::string & /*_format*/) const
{
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());

  // Set Image Dimensions
  msg.mutable_image()->set_width(this->ired1Cam->ImageWidth());
  msg.mutable_image()->set_height(this->ired1Cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(this->ired1Cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(this->ired1Cam->ImageWidth() *
                                this->ired1Cam->ImageDepth());
  msg.mutable_image()->set_data(this->ired1Cam->ImageData(),
                                this->ired1Cam->ImageDepth() *
                                    this->ired1Cam->ImageWidth() *
                                    this->ired1Cam->ImageHeight());

  // Publish realsense infrared stream
  this->ired1Pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewIR2Frame(const unsigned char *_image,
                                    unsigned int _width, unsigned int _height,
                                    unsigned int /*_depth*/,
                                    const std::string & /*_format*/) const
{
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());

  // Set Image Dimensions
  msg.mutable_image()->set_width(this->ired2Cam->ImageWidth());
  msg.mutable_image()->set_height(this->ired2Cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(this->ired2Cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(this->ired2Cam->ImageWidth() *
                                this->ired2Cam->ImageDepth());
  msg.mutable_image()->set_data(this->ired2Cam->ImageData(),
                                this->ired2Cam->ImageDepth() *
                                    this->ired2Cam->ImageWidth() *
                                    this->ired2Cam->ImageHeight());

  // Publish realsense infrared2 stream
  this->ired2Pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewColorFrame(const unsigned char *_image,
                                      unsigned int _width, unsigned int _height,
                                      unsigned int /*_depth*/,
                                      const std::string & /*_format*/) const
{
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());

  // Set Image Dimensions
  msg.mutable_image()->set_width(this->colorCam->ImageWidth());
  msg.mutable_image()->set_height(this->colorCam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(this->colorCam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(this->colorCam->ImageWidth() *
                                this->colorCam->ImageDepth());
  msg.mutable_image()->set_data(this->colorCam->ImageData(),
                                this->colorCam->ImageDepth() *
                                    this->colorCam->ImageWidth() *
                                    this->colorCam->ImageHeight());

  // Publish realsense color stream
  this->colorPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame(const float *_image, unsigned int _width,
                                      unsigned int _height,
                                      unsigned int /*_depth*/,
                                      const std::string & /*_format*/)
{
  // Allocate Memory for the real sense depth map
  // TODO: Allocate this memory more intelligently. Also make sure you free it
  // :)
  if (!this->depthMap)
  {
    this->depthMap = new uint16_t[this->depthCam->ImageWidth() *
                                  this->depthCam->ImageHeight()];
  }

  msgs::ImageStamped msg;

  const float *depth_data_float = this->depthCam->DepthData();

  // Pack viewable image message
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(
      depth_data_float,
      sizeof(*depth_data_float) * msg.image().width() * msg.image().height());

  // Publish viewable image
  this->depthViewPub->Publish(msg);

  // Convert depth data to realsense scaled depth map
  unsigned int depthMap_dim =
      this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  for (unsigned int i = 0; i < depthMap_dim; ++i)
  {
    // Check clipping and overflow
    if (depth_data_float[i] < DEPTH_NEAR_CLIP_M ||
        depth_data_float[i] > DEPTH_FAR_CLIP_M ||
        depth_data_float[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depth_data_float[i] < 0)
    {
      depthMap[i] = 0;
    }
    else
    {
      depthMap[i] = (uint16_t)(depth_data_float[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->world->GetSimTime());
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(
      this->depthMap,
      sizeof(*this->depthMap) * msg.image().width() * msg.image().height());

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate()
{
  return;
}

