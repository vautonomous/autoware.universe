// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "emergency.hpp"

#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <X11/Xlib.h>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>

namespace rviz_plugins
{
EmergencyDisplay::EmergencyDisplay()
{
  const Screen * screen_info = DefaultScreenOfDisplay(XOpenDisplay(NULL));

  constexpr float hight_4k = 2160.0;
  const float scale = static_cast<float>(screen_info->height) / hight_4k;
  const auto left = static_cast<int>(std::round(128 * scale));
  const auto top = static_cast<int>(std::round(128 * scale));
  const auto length = static_cast<int>(std::round(256 * scale));

  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(25, 255, 240), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", left, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", top, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz_common::properties::IntProperty(
    "Length", length, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_value_height_offset_ = new rviz_common::properties::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_value_scale_ = new rviz_common::properties::FloatProperty(
    "Value Scale", 1.0 / 6.667, "Value scale", this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
  property_handle_angle_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 3.0, "Scale is steering angle to handle angle ", this, SLOT(updateVisualization()),
    this);
  property_handle_angle_scale_->setMin(0.1);
}

EmergencyDisplay::~EmergencyDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void EmergencyDisplay::onInitialize()
{
  RTDClass::onInitialize();
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "EmergencyDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

//  overlay_->show();

  updateVisualization();
}

void EmergencyDisplay::onEnable()
{
  subscribe();
//  overlay_->show();

}

void EmergencyDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();

}

void EmergencyDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  bool emergency = false;
  bool emergency_hold = false;
  int emergency_level = 0;

    std::ostringstream text;

    std::lock_guard<std::mutex> message_lock(mutex_);
    if (last_msg_ptr_) {
      emergency = last_msg_ptr_->status.emergency;
      emergency_hold = last_msg_ptr_->status.emergency_holding;
      emergency_level = last_msg_ptr_->status.level;
      text << "Emergency: " << emergency << "\nEmergency hold: " << emergency_hold
           << "\nEmergency level: " << emergency_level;
      for (const auto & diag : last_msg_ptr_->status.diag_latent_fault) {
        text << "\nName: " << diag.name << ", Hardware ID: " << diag.hardware_id
             << ", Message: " << diag.message;
      }
      for (const auto & diag : last_msg_ptr_->status.diag_single_point_fault) {
        text << "\nName: " << diag.name << ", Hardware ID: " << diag.hardware_id
             << ", Message: " << diag.message;
      }
    }

    QColor background_color;
    background_color.setAlpha(0);
    jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(*overlay_);
    hud.fill(background_color);

    QPainter painter(&hud);
    painter.setRenderHint(QPainter::Antialiasing, true);


    int w = overlay_->getTextureWidth();
    int h = overlay_->getTextureHeight();

    // draw sign
    // QColor white_color(Qt::red);
    // white_color.setAlpha(255);
    // const double min_range_theta = 2.0 * M_PI + M_PI_2;
    // const double max_range_theta = 0.0 + M_PI_2;
    // painter.setPen(QPen(white_color, line_width, Qt::SolidLine));
    // painter.drawLine(
    //   (w / 2) + (line_width * 0.5) + ((double)w / 2.0 - (line_width * 0.5)) * std::cos(M_PI_4),
    //   (h / 2) + (line_width * 0.5) - ((double)h / 2.0 - (line_width * 0.5)) * std::sin(M_PI_4),
    //   (w / 2) + (line_width * 0.5) - ((double)w / 2.0 - (line_width * 0.5)) * std::cos(M_PI_4),
    //   (h / 2) + (line_width * 0.5) + ((double)h / 2.0 - (line_width * 0.5)) * std::sin(M_PI_4));
    // painter.drawArc(
    //   line_width * 0.5, line_width * 0.5, w, h, 16 * ((min_range_theta - M_PI) * 180.0 / M_PI),
    //   16 * ((max_range_theta - min_range_theta) * 180.0 / M_PI));

    // text
    QColor text_color(property_text_color_->getColor());
    text_color.setAlpha(255);
    painter.setPen(QPen(text_color, static_cast<int>(2), Qt::SolidLine));

    painter.drawText(
      0, 0, w,
      std::max(h, 1), Qt::AlignLeft | Qt::AlignBottom, text.str().c_str());
    painter.end();

}

void EmergencyDisplay::processMessage(
  const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) {
    return;
  }

  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    last_msg_ptr_ = msg_ptr;
  }
  if(msg_ptr->status.emergency || msg_ptr->status.emergency_holding){
    overlay_->show();
  } else {
    overlay_->hide();
  }

  queueRender();
}

void EmergencyDisplay::updateVisualization()
{
  if (last_msg_ptr_ != nullptr) {
    processMessage(last_msg_ptr_);
  }
  overlay_->updateTextureSize(3 * property_length_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());


}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EmergencyDisplay, rviz_common::Display)
