#ifndef __CAN_BRIDGE_NODE_H__
#define __CAN_BRIDGE_NODE_H__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include "cxkj_canalyst2_wrapper.h"
#include "itekon_x200_wrapper.h"

typedef union
{
	struct
	{
		int32_t sec;
		uint32_t nanosec;
		float throttle;
		float steer;
		float brake;
		int32_t gear;
	} __attribute__((packed)) msg;

	uint8_t buffer[24];
} SpiCarlaControl_t;

class CanBridgeNode : public rclcpp::Node
{
private:
    static constexpr unsigned int CAN_RECV_CHANNEL_ = 0;
    static constexpr unsigned int CAN_SEND_CHANNEL_ = 1;
    static constexpr unsigned int CARLA_CONTROL_CMD_CANID_0_ = 0x240;
    static constexpr unsigned int CARLA_CONTROL_CMD_CANID_1_ = 0x250;
    static constexpr unsigned int CARLA_CONTROL_CMD_CANID_2_ = 0x260;
    std::shared_ptr<UsbCanBase> usbcan_wrapper_ = nullptr;
    std::unique_ptr<std::thread> th_recv_ = nullptr;
    SpiCarlaControl_t spi_frame_;
    SpiCarlaControl_t spi_frame_last_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_control_;
    rclcpp::Time t_;

public:
    CanBridgeNode() : Node("CanBridgeNode")
    {
        LOGPF("system is %s-endian.\n", is_big_endian() ? "big" : "little");
    }
    ~CanBridgeNode()
    {}

    static int is_big_endian(void)
    {
        union {
            uint32_t i;
            char c[4];
        } e = {0x01000000};

        return e.c[0];
    }

    bool init(void)
    {
        memset(spi_frame_.buffer, 0, 24);
        memset(spi_frame_last_.buffer, 0, 24);
        pub_control_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);

#if USBCAN_TYPE == 0
        usbcan_wrapper_ = std::make_shared<UsbCanClassicCxkj> ();
#elif USBCAN_TYPE == 1
        usbcan_wrapper_ = std::make_shared<UsbCanFDItekon> ();
#endif
        if(!usbcan_wrapper_ -> init_usbcan()){
            RLOGF("failed to init usbcan!");
        }
        bool ret = (usbcan_wrapper_->open_usbcan(CAN_RECV_CHANNEL_))&&(usbcan_wrapper_->open_usbcan(CAN_SEND_CHANNEL_));
        if(ret){
            th_recv_ = std::unique_ptr<std::thread> (new std::thread(&CanBridgeNode::recv_loop, this));
        }
        return ret;
    }

    void destroy()
    {
        th_recv_ -> join();
        usbcan_wrapper_ -> close_usbcan();
    }

    bool process_can_frame_0(const CanFrame_t& frame)
    {
        t_ = rclcpp::Clock().now();
        memset(spi_frame_.buffer, 0, 24);
        memcpy(spi_frame_.buffer, frame.data, 8);
        return true;
    }

    bool process_can_frame_1(const CanFrame_t& frame)
    {
        memcpy(spi_frame_.buffer+8, frame.data, 8);
        return true;
    }

    bool process_can_frame_2(const CanFrame_t& frame)
    {
        static uint32_t counter = 0;
        memcpy(spi_frame_.buffer+16, frame.data, 8);
#if 0
        char tmp[128] = {0};
        for(int i=0; i<24; i++){
            sprintf(tmp+i*3, "%02x,", spi_frame_.buffer[i]);
        }
        RLOGI("spi bufffer: %s", tmp);
#endif
        if(counter % 100 == 0){
            RLOGI("spi frame [%d], sec: %d, nanosec: %d, throttle: %.3f, steer: %.3f, brake: %.3f, gear: %d", \
                counter, spi_frame_.msg.sec, spi_frame_.msg.nanosec, spi_frame_.msg.throttle, spi_frame_.msg.steer, spi_frame_.msg.brake, spi_frame_.msg.gear);
        }
        counter += 1;
        /* publish CarlaEgoVehicleControl, only if new timestamp arrival */
        if((spi_frame_.msg.sec != spi_frame_last_.msg.sec) || (spi_frame_.msg.nanosec != spi_frame_last_.msg.nanosec)){
            static double sum_latency = 0.0f;
            static uint32_t idx = 0;
            
            const double ts_msg = (double)(spi_frame_.msg.sec + spi_frame_.msg.nanosec*1e-9);
            const double ts_now = t_.seconds();
            idx += 1;
            const double latency = ts_now - ts_msg;
            sum_latency += latency;
            RLOGI("spi frame [%d] latency: %.6f, average latency: %.6f", idx, latency, sum_latency/idx);
            publish_carla_control(spi_frame_);
        }

        spi_frame_last_ = spi_frame_;

        return true;
    }

    void publish_carla_control(const SpiCarlaControl_t& sf)
    {
        /* publish control */
        carla_msgs::msg::CarlaEgoVehicleControl cmd;
        std_msgs::msg::Header header;
        header.frame_id = "world";
        header.stamp = rclcpp::Time(sf.msg.sec, sf.msg.nanosec);
        cmd.header = header;
        cmd.throttle = sf.msg.throttle;
        cmd.steer = sf.msg.steer;
        cmd.brake = sf.msg.brake;
        cmd.hand_brake = false;
        cmd.gear = sf.msg.gear;
        cmd.manual_gear_shift = false;
        cmd.reverse = (cmd.gear < 0);
        pub_control_->publish(cmd);
    }

    void recv_loop()
    {
        RLOGI("start recv thread.");
        CanFrameList_t frames;

        while(rclcpp::ok())
        {
            if(usbcan_wrapper_->recv_frame(frames, CAN_RECV_CHANNEL_) > 0){
                while(!frames.empty()){
                    CanFrame_t frame = frames.front();
                    /* process frame */
                    switch(frame.can_id){
                        case CARLA_CONTROL_CMD_CANID_0_:
                            process_can_frame_0(frame);
                            break;
                        case CARLA_CONTROL_CMD_CANID_1_:
                            process_can_frame_1(frame);
                            break;
                        case CARLA_CONTROL_CMD_CANID_2_:
                            process_can_frame_2(frame);
                            break;
                        default:
                            RLOGD("unsupported canid: 0x%03X", frame.can_id);
                    }
                    frames.pop();
                }
            }else{
                // std::chrono::milliseconds dura(10);
                continue;
            }
        }
        RLOGI("recv thread exit.");
    }

    SpiCarlaControl_t make_spi_frame(int i){
        SpiCarlaControl_t spi_frame;
        rclcpp::Time t = rclcpp::Clock().now();
        const double t_sec = t.seconds();
        spi_frame.msg.sec = (int32_t)(floor(t_sec));
        spi_frame.msg.nanosec = uint32_t((t_sec - spi_frame.msg.sec)*1e9);
        spi_frame.msg.throttle = i+2;
        spi_frame.msg.steer = i+3;
        spi_frame.msg.brake = i+4;
        spi_frame.msg.gear = i+5;
        return spi_frame;
    }

    void send_spi_frame(const SpiCarlaControl_t& spi_frame){
        CanFrame_t can_frame;
        can_frame.can_dlc = 8;

        can_frame.can_id = CARLA_CONTROL_CMD_CANID_0_;
        memcpy(can_frame.data, spi_frame.buffer, can_frame.can_dlc);
        usbcan_wrapper_->send_frame(can_frame, CAN_SEND_CHANNEL_);

        can_frame.can_id = CARLA_CONTROL_CMD_CANID_1_;
        memcpy(can_frame.data, spi_frame.buffer+8, can_frame.can_dlc);
        usbcan_wrapper_->send_frame(can_frame, CAN_SEND_CHANNEL_);

        can_frame.can_id = CARLA_CONTROL_CMD_CANID_2_;
        memcpy(can_frame.data, spi_frame.buffer+16, can_frame.can_dlc);
        usbcan_wrapper_->send_frame(can_frame, CAN_SEND_CHANNEL_);
    }

    void run_test_loop()
    {
        for(int i=0; i<100; i++){
            SpiCarlaControl_t spi_frame = make_spi_frame(i);
            send_spi_frame(spi_frame);
            usleep(100*1000);
        }
    }

};

#endif