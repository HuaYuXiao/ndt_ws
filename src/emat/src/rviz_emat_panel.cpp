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

    // Read slice parameters (shared with emat_feature_extractor)
    int slice_start = 0, slice_end = 0;
    nh.param("emat_feature_extractor/slice_start", slice_start, 0);
    nh.param("emat_feature_extractor/slice_end", slice_end, 0);
    if (slice_start > 0 || slice_end > 0) {
        _widget->setSlice(slice_start, slice_end);
        ROS_INFO("EMAT Waveform Panel: slice [%d, %d]", slice_start, slice_end);
    }

    _sub = nh.subscribe("/emat/waveform", 10, &RvizEmatPanel::waveformCb, this);
    _env_sub = nh.subscribe("/emat/envelope", 10, &RvizEmatPanel::envelopeCb, this);
    ROS_INFO("EMAT Waveform Panel: subscribed to /emat/waveform, /emat/envelope");
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

void RvizEmatPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void RvizEmatPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

} // namespace emat

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(emat::RvizEmatPanel, rviz::Panel)
