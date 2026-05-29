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
    _env_sub = nh.subscribe("/emat/envelope", 10, &RvizEmatPanel::envelopeCb, this);
    _feat_sub = nh.subscribe("/emat/features", 10, &RvizEmatPanel::featuresCb, this);
    ROS_INFO("EMAT Waveform Panel: subscribed to /emat/waveform, /emat/envelope, /emat/features");
}

void RvizEmatPanel::waveformCb(const EmatWaveform::ConstPtr& msg) {
    WaveformFrame f;
    f.raw_data = msg->raw_data;
    f.speed_of_voice = msg->speed_of_voice;
    _widget->pushFrame(f);
}

void RvizEmatPanel::envelopeCb(const EmatEnvelope::ConstPtr& msg) {
    std::vector<float> env(msg->envelope.begin(), msg->envelope.end());
    _widget->pushEnvelope(env, msg->sampling_rate);
}

void RvizEmatPanel::featuresCb(const EmatFeatures::ConstPtr& msg) {
    _widget->setThickness(msg->thickness_estimate);
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
