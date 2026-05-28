#include "ndt/rviz_target_panel.h"

#include <QVBoxLayout>
#include <QMouseEvent>
#include <ros/ros.h>

namespace ndt {

RvizTargetPanel::RvizTargetPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    _image_label = new QLabel(this);
    _image_label->setFixedSize(640, 480);
    _image_label->setAlignment(Qt::AlignCenter);
    _image_label->setText("Waiting for D435 image...");
    _image_label->setStyleSheet("QLabel { background-color: #1e1e1e; color: #aaa; }");
    _image_label->setScaledContents(true);

    auto* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(_image_label);
    layout->setAlignment(_image_label, Qt::AlignCenter);
    setLayout(layout);

    setFixedSize(640, 480);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void RvizTargetPanel::onInitialize() {
    ros::NodeHandle nh;
    _rgb_sub = nh.subscribe("/d435/color/image_raw", 2, &RvizTargetPanel::rgbCb, this);

    ros::NodeHandle pnh("~");
    _click_pub = pnh.advertise<geometry_msgs::PointStamped>("click_point", 1);

    ROS_INFO("RvizTargetPanel: subscribed to /d435/color/image_raw, publishing to ~click_point");
}

void RvizTargetPanel::rgbCb(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat rgb_mat;
        cv::cvtColor(cv_ptr->image, rgb_mat, cv::COLOR_BGR2RGB);

        QImage qimg(rgb_mat.data, rgb_mat.cols, rgb_mat.rows,
                    static_cast<int>(rgb_mat.step), QImage::Format_RGB888);

        QMutexLocker lk(&_mx);
        _current_qimage = qimg.copy();
        _img_width = rgb_mat.cols;
        _img_height = rgb_mat.rows;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(10, "RvizTargetPanel cv_bridge error: %s", e.what());
        return;
    }

    QMetaObject::invokeMethod(this, [this]() {
        QMutexLocker lk(&_mx);
        if (!_current_qimage.isNull()) {
            _image_label->setPixmap(QPixmap::fromImage(_current_qimage));
        }
    });
}

void RvizTargetPanel::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        QPoint label_pos = _image_label->mapFromParent(event->pos());
        if (_image_label->rect().contains(label_pos)) {
            QMutexLocker lk(&_mx);
            if (_img_width > 0 && _img_height > 0) {
                double sx = static_cast<double>(_img_width) / _image_label->width();
                double sy = static_cast<double>(_img_height) / _image_label->height();
                int img_x = static_cast<int>(label_pos.x() * sx);
                int img_y = static_cast<int>(label_pos.y() * sy);

                img_x = std::max(0, std::min(img_x, _img_width - 1));
                img_y = std::max(0, std::min(img_y, _img_height - 1));

                geometry_msgs::PointStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "d435_color_optical_frame";
                msg.point.x = static_cast<double>(img_x);
                msg.point.y = static_cast<double>(img_y);
                msg.point.z = 0.0;
                _click_pub.publish(msg);

                ROS_WARN("RvizTargetPanel: click at (%d, %d)", img_x, img_y);
            }
        }
    }
    rviz::Panel::mousePressEvent(event);
}

void RvizTargetPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void RvizTargetPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

} // namespace ndt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ndt::RvizTargetPanel, rviz::Panel)
