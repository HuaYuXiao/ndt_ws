#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <emat/EmatWaveform.h>
#include <emat/EmatFeatures.h>

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

class MultimodalRecorder
{
public:
    MultimodalRecorder()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Parameters
        std::string output_dir;
        pnh.param<std::string>("output_dir", output_dir,
            "/home/cwkj/ndt_ws/src/record/datasets");

        // Subscribers
        odom_sub_ = nh.subscribe(
            "/mavros/local_position/odom", 10,
            &MultimodalRecorder::odomCallback, this);

        image_sub_ = nh.subscribe(
            "/d435/color/image_raw", 10,
            &MultimodalRecorder::imageCallback, this);

        depth_sub_ = nh.subscribe(
            "/d435/aligned_depth_to_color/image_raw", 10,
            &MultimodalRecorder::depthCallback, this);

        pos_target_sub_ = nh.subscribe(
            "/mavros/setpoint_raw/local", 10,
            &MultimodalRecorder::posTargetCallback, this);

        emat_sub_ = nh.subscribe(
            "/emat/waveform", 10,
            &MultimodalRecorder::ematCallback, this);

        // 可选话题：EMAT 特征、法向量、接触概率
        emat_features_sub_ = nh.subscribe(
            "/emat/features", 10,
            &MultimodalRecorder::ematFeaturesCallback, this);

        normal_sub_ = nh.subscribe(
            "/ndt_normal/target_pose_d435", 10,
            &MultimodalRecorder::normalCallback, this);

        contact_prob_sub_ = nh.subscribe(
            "/ndt/contact_probability", 10,
            &MultimodalRecorder::contactProbCallback, this);

        // Create run directory
        std::string date_dir = output_dir + "/run_" + getTodayDate();
        std::filesystem::create_directories(date_dir);

        int run_id = 0;
        while (true) {
            run_dir_ = date_dir + "/" + std::to_string(run_id);
            if (!std::filesystem::exists(run_dir_))
                break;
            ++run_id;
        }
        std::filesystem::create_directories(run_dir_);
        std::filesystem::create_directories(run_dir_ + "/depth");

        // Open CSV files
        logfile_.open(run_dir_ + "/record_log.csv");
        logfile_ << "t,"
                 << "odom_x,odom_y,odom_z,"
                 << "odom_roll,odom_pitch,odom_yaw,"
                 << "target_x,target_y,target_z,target_yaw,"
                 << "emat_stamp,emat_sample_count,"
                 << "emat_speed_of_voice,emat_average_count\n";

        emat_logfile_.open(run_dir_ + "/emat_waveform.csv");
        emat_logfile_ << "stamp,sample_count,raw_data_hex\n";

        // 帧索引 CSV：对齐所有模态数据
        frame_index_.open(run_dir_ + "/frame_index.csv");
        frame_index_ << "frame_idx,stamp,"
                     << "pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,"
                     << "normal_x,normal_y,normal_z,"
                     << "emat_energy,emat_peak_amplitude,emat_arrival_time,"
                     << "emat_spectral_centroid,emat_kurtosis,emat_phase,"
                     << "emat_band0,emat_band1,emat_band2,emat_band3,"
                     << "emat_band4,emat_band5,emat_band6,emat_band7,"
                     << "emat_thickness,contact_prob\n";

        ROS_INFO("Multimodal recording to: %s", run_dir_.c_str());
    }

    ~MultimodalRecorder()
    {
        if (rgb_writer_.isOpened()) rgb_writer_.release();
        if (depth_writer_.isOpened()) depth_writer_.release();
        logfile_.close();
        emat_logfile_.close();
        frame_index_.close();
    }

private:
    // ROS subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber pos_target_sub_;
    ros::Subscriber emat_sub_;
    ros::Subscriber emat_features_sub_;
    ros::Subscriber normal_sub_;
    ros::Subscriber contact_prob_sub_;

    // Cached messages
    nav_msgs::Odometry last_odom_;
    sensor_msgs::Image last_image_;
    sensor_msgs::Image last_depth_;
    mavros_msgs::PositionTarget last_pos_target_;
    emat::EmatWaveform last_emat_;
    emat::EmatFeatures last_emat_features_;
    geometry_msgs::PoseStamped last_normal_;
    std_msgs::Float32MultiArray last_contact_prob_;

    // Flags
    bool have_odom_ = false;
    bool have_image_ = false;
    bool have_depth_ = false;
    bool have_pos_target_ = false;
    bool have_emat_ = false;
    bool have_emat_features_ = false;
    bool have_normal_ = false;
    bool have_contact_prob_ = false;

    std::mutex mtx_;
    std::ofstream logfile_;
    std::ofstream emat_logfile_;
    std::ofstream frame_index_;

    // Video writers
    cv::VideoWriter rgb_writer_;
    cv::VideoWriter depth_writer_;
    std::string run_dir_;
    int frame_idx_ = 0;


    // ===== Utility =====
    std::string getTodayDate()
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d");
        return oss.str();
    }

    static double extractStamp(const std_msgs::Header& header)
    {
        return header.stamp.toSec();
    }

    // ===== Callbacks =====
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_odom_ = *msg;
        have_odom_ = true;
        tryRecord();
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_image_ = *msg;
        have_image_ = true;
        tryRecord();
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_depth_ = *msg;
        have_depth_ = true;
        tryRecord();
    }

    void posTargetCallback(
        const mavros_msgs::PositionTarget::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_pos_target_ = *msg;
        have_pos_target_ = true;
    }

    void ematCallback(const emat::EmatWaveform::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_emat_ = *msg;
        have_emat_ = true;
        tryRecord();
    }

    void ematFeaturesCallback(const emat::EmatFeatures::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_emat_features_ = *msg;
        have_emat_features_ = true;
    }

    void normalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_normal_ = *msg;
        have_normal_ = true;
    }

    void contactProbCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        last_contact_prob_ = *msg;
        have_contact_prob_ = true;
    }

    // ===== Unified recording entry =====
    void tryRecord()
    {
        if (!have_odom_ || !have_image_ || !have_depth_)
            return;

        // ---------- Pose ----------
        const auto& p = last_odom_.pose.pose.position;
        const auto& q = last_odom_.pose.pose.orientation;

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Use odometry message stamp
        double odom_stamp = extractStamp(last_odom_.header);

        logfile_
            << std::fixed << std::setprecision(6)
            << odom_stamp << ","
            << p.x << "," << p.y << "," << p.z << ","
            << roll << "," << pitch << "," << yaw << ",";

        // ---------- Target setpoint ----------
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
                << target_yaw << ",";
        } else {
            logfile_ << "nan,nan,nan,nan,";
        }

        // ---------- EMAT metadata in main CSV ----------
        if (have_emat_) {
            logfile_
                << last_emat_.stamp.toSec() << ","
                << last_emat_.sample_count << ","
                << last_emat_.speed_of_voice << ","
                << (int)last_emat_.average_count << "\n";

            // Full waveform to separate CSV
            emat_logfile_ << std::fixed << std::setprecision(6)
                          << last_emat_.stamp.toSec() << ","
                          << last_emat_.sample_count;
            for (auto byte : last_emat_.raw_data) {
                emat_logfile_ << "," << std::hex
                              << std::setw(2) << std::setfill('0')
                              << (int)byte;
            }
            emat_logfile_ << std::dec << "\n";
            emat_logfile_.flush();
        } else {
            logfile_ << "nan,nan,nan,nan\n";
        }

        logfile_.flush();

        // ---------- RGB video ----------
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

        // ---------- Depth video ----------
        cv::Mat depth16 =
            cv_bridge::toCvCopy(
                last_depth_,
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

        // ---------- 16-bit Depth PNG（训练用，保留原始精度）----------
        std::ostringstream depth_name;
        depth_name << run_dir_ << "/depth/"
                   << std::setw(6) << std::setfill('0')
                   << frame_idx_ << ".png";
        cv::imwrite(depth_name.str(), depth16);

        // ---------- frame_index.csv（对齐所有模态）----------
        double depth_stamp = extractStamp(last_depth_.header);

        // 法向量：从四元数提取 X 轴方向（与 normal_ros.py 一致）
        double nx = std::nan(""), ny = std::nan(""), nz = std::nan("");
        if (have_normal_) {
            const auto& nq = last_normal_.pose.orientation;
            tf2::Quaternion n_tf_q(nq.x, nq.y, nq.z, nq.w);
            tf2::Matrix3x3 n_R(n_tf_q);
            // X 轴 = 法线方向
            tf2::Vector3 x_axis = n_R.getColumn(0);
            nx = x_axis.x(); ny = x_axis.y(); nz = x_axis.z();
        }

        // 接触概率：取窗口最后一个元素（当前帧）
        double contact_prob = std::nan("");
        if (have_contact_prob_ && last_contact_prob_.data.size() > 0) {
            contact_prob = last_contact_prob_.data.back();
        }

        frame_index_
            << std::fixed << std::setprecision(6)
            << frame_idx_ << "," << depth_stamp << ","
            << p.x << "," << p.y << "," << p.z << ","
            << roll << "," << pitch << "," << yaw << ","
            << nx << "," << ny << "," << nz << ",";

        // EMAT features（可选）
        if (have_emat_features_) {
            frame_index_
                << last_emat_features_.energy << ","
                << last_emat_features_.peak_amplitude << ","
                << last_emat_features_.arrival_time << ","
                << last_emat_features_.spectral_centroid << ","
                << last_emat_features_.kurtosis << ","
                << last_emat_features_.phase << ",";
            for (int i = 0; i < 8; ++i) {
                frame_index_ << last_emat_features_.band_energies[i] << ",";
            }
            frame_index_ << last_emat_features_.thickness_estimate << ",";
        } else {
            frame_index_ << "nan,nan,nan,nan,nan,nan,"
                         << "nan,nan,nan,nan,nan,nan,nan,nan,nan,";
        }

        frame_index_ << contact_prob << "\n";
        frame_index_.flush();

        ++frame_idx_;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multimodal_recorder");
    MultimodalRecorder recorder;
    ros::spin();
    return 0;
}
