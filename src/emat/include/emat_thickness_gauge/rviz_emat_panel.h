#pragma once

#include <rviz/panel.h>
#include <ros/ros.h>

#include "emat_thickness_gauge/waveform_widget.h"
#include "emat/EmatWaveform.h"

namespace emat {

class RvizEmatPanel : public rviz::Panel {
    Q_OBJECT

public:
    explicit RvizEmatPanel(QWidget* parent = nullptr);
    ~RvizEmatPanel() override = default;

    void onInitialize() override;
    void save(rviz::Config config) const override;
    void load(const rviz::Config& config) override;

private:
    void waveformCb(const EmatWaveform::ConstPtr& msg);

    WaveformWidget* _widget;
    ros::Subscriber _sub;
};

} // namespace emat
