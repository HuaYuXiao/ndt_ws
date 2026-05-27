#include <QApplication>
#include <thread>
#include <ros/ros.h>

#include "emat_thickness_gauge/waveform_widget.h"
#include "emat/EmatWaveform.h"

namespace {

emat::WaveformWidget* g_widget = nullptr;

void waveformCb(const emat::EmatWaveform::ConstPtr& msg) {
    if (!g_widget) return;
    emat::WaveformFrame f;
    f.raw_data = msg->raw_data;
    f.material = msg->material;
    f.speed_of_voice = msg->speed_of_voice;
    g_widget->pushFrame(f);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "emat_waveform_viz");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    emat::WaveformWidget widget;
    g_widget = &widget;
    widget.show();

    ros::Subscriber sub = nh.subscribe("/emat/waveform", 10, waveformCb);

    ROS_INFO("EMAT Waveform Visualizer (Qt5)");
    ROS_INFO("Subscribing to /emat/waveform");

    // ROS spin in background thread
    std::thread spinThread([]() { ros::spin(); });

    int ret = app.exec();

    g_widget = nullptr;
    ros::shutdown();
    if (spinThread.joinable())
        spinThread.join();

    return ret;
}
