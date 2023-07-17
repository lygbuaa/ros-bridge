#ifndef __CAN_BRIDGE_NODE_H__
#define __CAN_BRIDGE_NODE_H__

#include <cv_bridge/cv_bridge.h>
#include "aumo_s2_sdk/controlplay.h"
#include "logging_utils.h"
#include "cv_param_loader.h"
#include "aumo_device.h"

class PcieBridgeNode : public rclcpp::Node
{
private:
    rclcpp::Time t_;
    std::shared_ptr<CvParamLoader> param_loader_ = nullptr;
    std::shared_ptr<AumoDevice> aumo_device_ = nullptr;
    std::vector<aumo_channel_info_t> aumo_infos_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_front_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_left_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_rear_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_right_;

public:
    PcieBridgeNode() : Node("PcieBridgeNode")
    {}
    ~PcieBridgeNode()
    {}

    bool init(const std::string config_path)
    {
        param_loader_ = std::make_shared<CvParamLoader>(config_path);
        for(int i=0; i<AUMO_CH_NUM_; i++)
        {
            aumo_channel_info_t info;
            info.ch = i;
            info.width = param_loader_->image_width_;
            info.height = param_loader_->image_height_;
            info.h_total = info.width;
            info.v_total = info.height;
            info.fps = param_loader_->fps_;
            info.frame_period_nsec = 1e9 / info.fps;
            info.video_type = FB_YUYV8_422;
            info.gmsl_speed = GMSL_SPEED_3G;

            float stride = (info.video_type==FB_RAW12) ? 1.5f : 2.0f;
            info.stride = (int)round(info.width * stride);

            aumo_infos_.emplace_back(info);
        }

        aumo_device_ = std::make_shared<AumoDevice>();
        if(!aumo_device_->init_dev(aumo_infos_))
        {
            RLOGE("pcie card init error, quit!");
            abort();
        }

        rclcpp::QoS qos = rclcpp::QoS(
            rclcpp::KeepLast(10)
        );
        //qos.best_effort();
        qos.reliable();
        sub_img_front_ = this->create_subscription<sensor_msgs::msg::Image>(param_loader_->image_front_topic_, qos, std::bind(&PcieBridgeNode::img_front_callback, this, std::placeholders::_1));
        sub_img_left_ = this->create_subscription<sensor_msgs::msg::Image>(param_loader_->image_left_topic_, qos, std::bind(&PcieBridgeNode::img_left_callback, this, std::placeholders::_1));
        sub_img_rear_ = this->create_subscription<sensor_msgs::msg::Image>(param_loader_->image_rear_topic_, qos, std::bind(&PcieBridgeNode::img_rear_callback, this, std::placeholders::_1));
        sub_img_right_ = this->create_subscription<sensor_msgs::msg::Image>(param_loader_->image_right_topic_, qos, std::bind(&PcieBridgeNode::img_right_callback, this, std::placeholders::_1));

        return true;
    }

    void destroy()
    {
        // th_recv_ -> join();
    }

    void img_front_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {       
        send_pcie_image(img_msg, param_loader_->image_front_chn_);
    }

    void img_left_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        send_pcie_image(img_msg, param_loader_->image_left_chn_);
    }

    void img_rear_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        send_pcie_image(img_msg, param_loader_->image_rear_chn_);
    }

    void img_right_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        send_pcie_image(img_msg, param_loader_->image_right_chn_);
    }

    cv::Mat get_img_from_msg(const sensor_msgs::msg::Image::SharedPtr& img_msg){
        cv_bridge::CvImagePtr ptr;
        std::string encoding = (img_msg->encoding=="8UC1") ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::RGB8;
        ptr = cv_bridge::toCvCopy(img_msg, encoding);
        return ptr->image;
    }

    void YUVToYUYV(const cv::Mat &yuv, std::vector<uint8_t> &yuyv)
    {
        int width = yuv.cols;
        int height = yuv.rows;

        // 分配YUYV422缓冲区大小（每个像素占2字节）
        yuyv.resize(width * height * 2);

        int yuyvIndex = 0;
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x += 2)
            {
                // 获取相邻的两个像素
                cv::Vec3b yuvPixel1 = yuv.at<cv::Vec3b>(y, x);
                cv::Vec3b yuvPixel2 = yuv.at<cv::Vec3b>(y, x + 1);

                // 提取Y、U、V分量
                uint8_t y1 = yuvPixel1[0];
                uint8_t u = yuvPixel1[1];
                uint8_t y2 = yuvPixel2[0];
                uint8_t v = yuvPixel2[2];

                // 将Y、U、Y、V交错排列成YUYV形式
                yuyv[yuyvIndex++] = y1;
                yuyv[yuyvIndex++] = u;
                yuyv[yuyvIndex++] = y2;
                yuyv[yuyvIndex++] = v;
            }
        }
    }

    bool send_pcie_image(const sensor_msgs::msg::Image::SharedPtr img_msg, int chn)
    {
        cv::Mat rgbImage = get_img_from_msg(img_msg);
        rclcpp::Time timestamp = img_msg->header.stamp;

        cv::Mat yuvImage;
        yuvImage.create(rgbImage.rows, rgbImage.cols*2, CV_8UC2);
        cv::cvtColor(rgbImage, yuvImage, cv::COLOR_RGB2YUV);

        aumo_channel_info_t& info = aumo_infos_[chn];

        info.yuyvImage.clear();
        YUVToYUYV(yuvImage, info.yuyvImage);

        cv::Size newSize(rgbImage.cols, rgbImage.rows);
        cv::Mat tmp_img(yuvImage.rows, yuvImage.cols, CV_8UC2, info.yuyvImage.data());
        cv::resize(tmp_img, info.yuyvImageMat, newSize);
        char* imageData = reinterpret_cast<char *>(info.yuyvImageMat.data);

        int64_t timestampNs = timestamp.nanoseconds();
        
        /* attach timestamp in nano-seconds */
        if(imageData)
        {
            std::memcpy(imageData, &timestampNs, sizeof(timestampNs));
            RLOGD("chn[%d] attach timestampNs: %ld", chn, timestampNs);
        }

        
        info.pdata = imageData;
        return aumo_device_ -> inject_image(info);
    }

    void run_test_loop()
    {}


};

#endif