#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <mavros_msgs/PositionTarget.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <ctime>

class Recorder
{
public:
    Recorder()
    {
        ros::NodeHandle nh;

        odom_sub_ = nh.subscribe(
            "/mavros/local_position/odom", 10,
            &Recorder::odomCallback, this);

        image_sub_ = nh.subscribe(
            "/d435/color/image_raw", 10,
            &Recorder::imageCallback, this);

        cloud_sub_ = nh.subscribe(
            "/d435/aligned_depth_to_color/image_raw", 10,
            &Recorder::cloudCallback, this);

        pos_target_sub_ = nh.subscribe(
            "/mavros/setpoint_raw/local", 10,
            &Recorder::posTargetCallback, this);

        // ===== 创建实验目录 =====
        std::string base_dir =
            "/home/cwkj/ndt_ws/src/ndt/runs/";

        std::string date_dir = base_dir + getTodayDate();
        std::filesystem::create_directories(date_dir);

        int run_id = 0;
        std::string run_dir;
        while (true) {
            run_dir = date_dir + "/" + std::to_string(run_id);
            if (!std::filesystem::exists(run_dir))
                break;
            ++run_id;
        }
        std::filesystem::create_directories(run_dir);

        // ===== 打开 CSV =====
        logfile_.open(run_dir + "/record_log.csv");
        logfile_ << "t,"
                 << "odom_x,odom_y,odom_z,"
                 << "odom_roll,odom_pitch,odom_yaw,"
                 << "target_x,target_y,target_z,target_yaw\n";

        // ===== 打开视频写入器（延迟初始化）=====
        run_dir_ = run_dir;

        ROS_INFO("Recording experiment to: %s", run_dir_.c_str());
    }

    ~Recorder()
    {
        if (rgb_writer_.isOpened())
            rgb_writer_.release();
        if (depth_writer_.isOpened())
            depth_writer_.release();
        logfile_.close();
    }

private:
    // ===== ROS =====
    ros::Subscriber odom_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber pos_target_sub_;

    // ===== 数据缓存 =====
    nav_msgs::Odometry last_odom_;
    sensor_msgs::Image last_image_;
    sensor_msgs::Image last_cloud_;
    mavros_msgs::PositionTarget last_pos_target_;

    bool have_odom_ = false;
    bool have_image_ = false;
    bool have_cloud_ = false;
    bool have_pos_target_ = false;

    std::mutex mtx_;
    std::ofstream logfile_;

    // ===== 视频 =====
    cv::VideoWriter rgb_writer_;
    cv::VideoWriter depth_writer_;
    std::string run_dir_;

    // ===== 工具函数 =====
    std::string getTodayDate()
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d");
        return oss.str();
    }

    // ===== 回调 =====
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_odom_ = *msg;
        have_odom_ = (last_odom_.pose.pose.position.z > 0.5);
        tryRecord();
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_image_ = *msg;
        have_image_ = true;
        tryRecord();
    }

    void cloudCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_cloud_ = *msg;
        have_cloud_ = true;
        tryRecord();
    }

    void posTargetCallback(
        const mavros_msgs::PositionTarget::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_pos_target_ = *msg;
        have_pos_target_ = true;
    }

    // ===== 统一记录入口 =====
    void tryRecord()
    {
        if (!have_odom_ || !have_image_ || !have_cloud_){
            return;
        }

        // ---------- 位姿 ----------
        const auto& p = last_odom_.pose.pose.position;
        const auto& q = last_odom_.pose.pose.orientation;

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        logfile_
            << ros::Time::now().toSec() << ","
            << p.x << "," << p.y << "," << p.z << ","
            << roll << "," << pitch << "," << yaw << ",";

        if (have_pos_target_) {
            double target_yaw = std::nan("");
            if (!(last_pos_target_.type_mask &
                  mavros_msgs::PositionTarget::IGNORE_YAW)) {
                target_yaw = last_pos_target_.yaw;
            }

            logfile_
                << last_pos_target_.position.x << ","
                << last_pos_target_.position.y << ","
                << last_pos_target_.position.z << ","
                << target_yaw << "\n";
        } else {
            logfile_ << "nan,nan,nan,nan\n";
        }

        logfile_.flush();

        // ---------- RGB ----------
        cv::Mat rgb =
            cv_bridge::toCvCopy(last_image_, "bgr8")->image;

        if (!rgb_writer_.isOpened()) {
            rgb_writer_.open(
                run_dir_ + "/rgb.mp4",
                cv::VideoWriter::fourcc('m','p','4','v'),
                30,
                rgb.size(),
                true);
        }
        rgb_writer_.write(rgb);

        // ---------- Depth ----------
        cv::Mat depth16 =
            cv_bridge::toCvCopy(
                last_cloud_,
                sensor_msgs::image_encodings::TYPE_16UC1)->image;

        cv::Mat depth8;
        depth16.convertTo(depth8, CV_8U, 255.0 / 5000.0);

        if (!depth_writer_.isOpened()) {
            depth_writer_.open(
                run_dir_ + "/depth.mp4",
                cv::VideoWriter::fourcc('m','p','4','v'),
                30,
                depth8.size(),
                false);
        }
        depth_writer_.write(depth8);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_record");
    Recorder recorder;
    ros::spin();
    return 0;
}
