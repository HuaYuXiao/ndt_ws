#include "emat_thickness_gauge/rviz_emat_panel.h"

#include <QVBoxLayout>
#include <ros/ros.h>

namespace emat {

RvizEmatPanel::RvizEmatPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    _widget = new WaveformWidget(this);

    auto* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(_widget);
    setLayout(layout);
}

void RvizEmatPanel::onInitialize() {
    ros::NodeHandle nh;
    _sub = nh.subscribe("/emat/waveform", 10, &RvizEmatPanel::waveformCb, this);
    ROS_INFO("EMAT Waveform Panel: subscribed to /emat/waveform");
}

void RvizEmatPanel::waveformCb(const EmatWaveform::ConstPtr& msg) {
    WaveformFrame f;
    f.raw_data = msg->raw_data;
    f.material = msg->material;
    f.speed_of_voice = msg->speed_of_voice;
    _widget->pushFrame(f);
}

void RvizEmatPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void RvizEmatPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

} // namespace emat

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(emat::RvizEmatPanel, rviz::Panel)
