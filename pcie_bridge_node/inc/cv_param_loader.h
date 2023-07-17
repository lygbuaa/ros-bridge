#pragma once

#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


class CvParamLoader : public rclcpp::Node
{
public:
    static constexpr double PI_ = 3.141592741;
    static constexpr double DEG2RAD = 180.0 / PI_;

    std::string image_front_topic_;
    std::string image_left_topic_;
    std::string image_rear_topic_;
    std::string image_right_topic_;

    int image_width_;
    int image_height_;
    int fps_;

    int image_front_chn_;
    int image_left_chn_;
    int image_rear_chn_;
    int image_right_chn_;

private:
    std::string yaml_path_;
    cv::FileStorage fs_;

public:
    CvParamLoader(std::string yaml_path) : Node("CvParamLoader")
    {
        yaml_path_ = yaml_path;
        fs_.open(yaml_path, cv::FileStorage::READ);
        if(!fs_.isOpened()){
            RLOGE("open config file failed: %s\n", yaml_path.c_str());
        }
        assert(fs_.isOpened());
        load_params();
    }

    ~CvParamLoader(){
        if(fs_.isOpened()){
            fs_.release();
        }
    }

    /* this is project source code path */
    std::string get_package_src_path(const std::string& pkg_name = "se_apa_node"){
        std::string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
        // return pkg_path + "/../";
        return pkg_path + "/../../../../ros-bridge/";
    }

    /* this is project source code path */
    std::string get_package_share_path(const std::string& pkg_name = "se_apa_node"){
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory(pkg_name);
        return pkg_share_path + "/";
    }

    void load_params(){
        fs_["image_front_topic"] >> image_front_topic_;
        fs_["image_left_topic"] >> image_left_topic_;
        fs_["image_rear_topic"] >> image_rear_topic_;
        fs_["image_right_topic"] >> image_right_topic_;

        fs_["image_front_chn"] >> image_front_chn_;
        fs_["image_left_chn"] >> image_left_chn_;
        fs_["image_rear_chn"] >> image_rear_chn_;
        fs_["image_right_chn"] >> image_right_chn_;

        RLOGI("image_front_topic[%d]: %s, image_left_topic[%d]: %s, image_rear_topic[%d]: %s, image_right_topic[%d]: %s\n", \
            image_front_chn_, image_front_topic_.c_str(), image_left_chn_, image_left_topic_.c_str(), image_rear_chn_, image_rear_topic_.c_str(), image_right_chn_, image_right_topic_.c_str());

        fs_["image_width"] >> image_width_;
        fs_["image_height"] >> image_height_;
        fs_["fps"] >> fps_;
        RLOGI("image_width: %d, image_height: %d, fps: %d", image_width_, image_height_, fps_);
    }

};