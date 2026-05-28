#pragma once

#include <rviz/panel.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <QLabel>
#include <QImage>
#include <QMutex>
#include <QWidget>

namespace ndt {

class RvizTargetPanel : public rviz::Panel {
    Q_OBJECT

public:
    explicit RvizTargetPanel(QWidget* parent = nullptr);
    ~RvizTargetPanel() override = default;

    void onInitialize() override;
    void save(rviz::Config config) const override;
    void load(const rviz::Config& config) override;

protected:
    void mousePressEvent(QMouseEvent* event) override;

private:
    void rgbCb(const sensor_msgs::Image::ConstPtr& msg);

    QLabel* _image_label;

    ros::Subscriber _rgb_sub;
    ros::Publisher _click_pub;

    mutable QMutex _mx;
    QImage _current_qimage;
    int _img_width = 0;
    int _img_height = 0;
};

} // namespace ndt
