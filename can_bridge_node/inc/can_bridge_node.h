#ifndef __CAN_BRIDGE_NODE_H__
#define __CAN_BRIDGE_NODE_H__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include "usbcan_wrapper.h"
#include "logging_utils.h"

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
    std::shared_ptr<UsbCanClassical> usbcan_wrapper_ = nullptr;
    std::unique_ptr<std::thread> th_recv_ = nullptr;
    SpiCarlaControl_t spi_frame_;

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
        usbcan_wrapper_ = std::make_shared<UsbCanClassical>();
        th_recv_ = std::unique_ptr<std::thread> (new std::thread(&CanBridgeNode::recv_loop, this));
        bool ret = (usbcan_wrapper_->open_usbcan(CAN_RECV_CHANNEL_))&&(usbcan_wrapper_->open_usbcan(CAN_SEND_CHANNEL_));
        return ret;
    }

    bool process_can_frame_0(const CanFrameClassical_t& frame)
    {
        memset(spi_frame_.buffer, 0, 24);
        memcpy(spi_frame_.buffer, frame.data, 8);
        return true;
    }

    bool process_can_frame_1(const CanFrameClassical_t& frame)
    {
        memcpy(spi_frame_.buffer+8, frame.data, 8);
        return true;
    }

    bool process_can_frame_2(const CanFrameClassical_t& frame)
    {
        memcpy(spi_frame_.buffer+16, frame.data, 8);
        char tmp[128] = {0};
        for(int i=0; i<24; i++){
            sprintf(tmp+i*3, "%02x,", spi_frame_.buffer[i]);
        }
        RLOGI("spi bufffer: %s", tmp);
        RLOGI("spi frame, sec: %d, nanosec: %d, throttle: %.3f, steer: %.3f, brake: %.3f, gear: %d", \
               spi_frame_.msg.sec, spi_frame_.msg.nanosec, spi_frame_.msg.throttle, spi_frame_.msg.steer, spi_frame_.msg.brake, spi_frame_.msg.gear);
        /* publish CarlaEgoVehicleControl */

        return true;
    }

    void recv_loop()
    {
        RLOGI("start recv thread.");
        FrameList_t frames;

        while(rclcpp::ok())
        {
            if(usbcan_wrapper_->recv_frame(frames, CAN_RECV_CHANNEL_) > 0){
                while(!frames.empty()){
                    CanFrameClassical_t frame = frames.front();
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
                            RLOGI("unsupported canid: 0x%03X", frame.can_id);
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
        spi_frame.msg.sec = i;
        spi_frame.msg.nanosec = i+1;
        spi_frame.msg.throttle = i+2;
        spi_frame.msg.steer = i+3;
        spi_frame.msg.brake = i+4;
        spi_frame.msg.gear = i+5;
        return spi_frame;
    }

    void send_spi_frame(const SpiCarlaControl_t& spi_frame){
        CanFrameClassical_t can_frame;
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
        for(int i=0; i<10; i++){
            SpiCarlaControl_t spi_frame = make_spi_frame(i);
            send_spi_frame(spi_frame);
            sleep(1);
        }
    }

};

#endif