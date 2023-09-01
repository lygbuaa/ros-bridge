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
    std::vector<aumo_video_channel_info_t> aumo_video_infos_;
    std::vector<aumo_canfd_channel_info_t> aumo_canfd_infos_;
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
        for(int i=0; i<AUMO_VIDEO_CH_NUM_; i++)
        {
            aumo_video_channel_info_t info;
            info.ch = i;
            info.width = param_loader_->image_width_;
            info.height = param_loader_->image_height_;
            info.h_total = param_loader_->signal_width_;
            info.v_total = param_loader_->signal_height_;
            info.fps = param_loader_->image_fps_;
            info.frame_period_nsec = 1e9 / info.fps;
            info.video_type = param_loader_->image_format_;     //FB_YUYV8_422
            info.gmsl_speed = param_loader_->gmsl_speed_;       //GMSL_SPEED_3G
            info.trigger_mode = param_loader_->trigger_mode_;   //PLAY_TIMESTAMP

            float stride = (info.video_type==FB_RAW12) ? 1.5f : 2.0f;
            info.stride = (int)round(info.width * stride);

            aumo_video_infos_.emplace_back(info);
        }

        for (int i = 0; i < AUMO_CANFD_CH_NUM_; i++)
        {
            aumo_canfd_channel_info_t info;
            info.ch = i;
            info.is_canfd = (param_loader_->is_canfd_>0);
            info.arbi_baud = param_loader_->canfd_arbi_baudrate_;
            info.data_baud = param_loader_->canfd_data_baudrate_;
            info.fps = param_loader_->canfd_fps_;
            info.msg_period_nsec = 1e9 / info.fps;
            aumo_canfd_infos_.emplace_back(info);
        }

        aumo_device_ = std::make_shared<AumoDevice>();
        if(!aumo_device_->init_dev(param_loader_->pcie_card_id_, aumo_video_infos_, aumo_canfd_infos_))
        {
            RLOGE("pcie card [%d] init error, quit!", param_loader_->pcie_card_id_);
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

        aumo_video_channel_info_t& info = aumo_video_infos_[chn];

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

    void test_canfd()
    {
        unsigned int msg_data[16]={0x11223344,0x55667788,0x99AABBCC,0xDDEEFF00, 0x11223344,0x55667788,0x99AABBCC,0xDDEEFF00, 0x11223344,0x55667788,0x99AABBCC,0xDDEEFF00, 0x11223344,0x55667788,0x99AABBCC,0xDDEEFF00};
        int canid = 0x100;
        for(int j=0; j<100; j++)
        {

            for (int i = 0; i < AUMO_CANFD_CH_NUM_; i++)
            {
                aumo_canfd_channel_info_t& info = aumo_canfd_infos_[i];
                info.id = canid;
                info.pdata = msg_data;
                if(info.is_canfd)
                {
                    info.len = 64;
                }
                else
                {
                    info.len = 8;
                }
                aumo_device_ -> inject_canfd_msg(info);
                RLOGI("chn[%d] inject canfd msg: 0x%x", i, canid);
            }

            canid += 1;
            if(canid > 0x8ff)
            {
                canid = 0x100;
            }
            // usleep(100*1000);
        }
    }


};

#endif