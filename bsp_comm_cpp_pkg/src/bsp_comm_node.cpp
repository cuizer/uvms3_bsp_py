#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "hal/msg/hal_inertialnavi.hpp"
#include "hal/msg/hal_dvl.hpp"
#include "hal/msg/hal_depthsensor.hpp"
#include "hal/msg/hal_mainthruster.hpp"
#include "hal/msg/hal_auxithruster.hpp"
#include "hal/msg/hal_battery.hpp"
#include "hal/msg/hal_tailservo.hpp"
#include "hal/msg/hal_antenna.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <thread>
#include <cstring>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BspCommNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BspCommNode(): LifecycleNode("bsp_comm_node")
    {
        this->declare_parameter<std::string>("udp_ip", "127.0.0.1");
        this->declare_parameter<int>("udp_port", 5000);
    }

    // ================= 生命周期 =================
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        auto qos = rclcpp::QoS(10);

        // 创建HAL节点消息订阅
        inertial_sub_     = this->create_subscription<hal::msg::HalInertialnavi>("/hal/inertialnavi",qos,std::bind(&BspCommNode::inertial_callback, this, std::placeholders::_1));
        dvl_sub_          = this->create_subscription<hal::msg::HalDvl>("/hal/dvl",qos,std::bind(&BspCommNode::dvl_callback, this, std::placeholders::_1));
        depthsensor_sub_  = this->create_subscription<hal::msg::HalDepthsensor>("/hal/depthsensor",qos,std::bind(&BspCommNode::depthsensor_callback, this, std::placeholders::_1));
        mainthruster_sub_ = this->create_subscription<hal::msg::HalMainthruster>("/hal/mainthruster",qos,std::bind(&BspCommNode::mainthruster_callback, this, std::placeholders::_1));
        auxithruster_sub_ = this->create_subscription<hal::msg::HalAuxithruster>("/hal/auxithruster",qos,std::bind(&BspCommNode::auxithruster_callback, this, std::placeholders::_1));
        battery_sub_      = this->create_subscription<hal::msg::HalBattery>("/hal/battery",qos,std::bind(&BspCommNode::battery_callback, this, std::placeholders::_1));
        tailservo_sub_    = this->create_subscription<hal::msg::HalTailservo>("/hal/tailservo",qos,std::bind(&BspCommNode::tailservo_callback, this, std::placeholders::_1));
        antenna_sub_      = this->create_subscription<hal::msg::HalAntenna>("/hal/antenna",qos,std::bind(&BspCommNode::antenna_callback, this, std::placeholders::_1));
 
        udp_ip_ = this->get_parameter("udp_ip").as_string();
        udp_port_ = this->get_parameter("udp_port").as_int();

        // 创建UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(get_logger(), "Socket create failed");
            return CallbackReturn::FAILURE;
        }

        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(udp_port_);
        inet_pton(AF_INET, udp_ip_.c_str(), &target_addr_.sin_addr);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&BspCommNode::udp_send, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        active_ = true;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        active_ = false;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        inertial_sub_.reset();
        dvl_sub_.reset();
        depthsensor_sub_.reset();
        mainthruster_sub_.reset();
        auxithruster_sub_.reset();
        battery_sub_.reset();
        tailservo_sub_.reset();
        antenna_sub_.reset();
        timer_.reset();

        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }

        return CallbackReturn::SUCCESS;
    }

private:

    // ================= 回调函数 =================
    void inertial_callback(const hal::msg::HalInertialnavi::SharedPtr msg)
    {
        if (!active_) return;
        inertial_data_ = *msg;
    }
    
    void dvl_callback(const hal::msg::HalDvl::SharedPtr msg)
    {
        if (!active_) return;
        dvl_data_ = *msg;
    }
    
    void depthsensor_callback(const hal::msg::HalDepthsensor::SharedPtr msg)
    {
        if (!active_) return;
        depthsensor_data_ = *msg;
    }
    
    void mainthruster_callback(const hal::msg::HalMainthruster::SharedPtr msg)
    {
        if (!active_) return;
        mainthruster_data_ = *msg;
    }
    
    void auxithruster_callback(const hal::msg::HalAuxithruster::SharedPtr msg)
    {
        if (!active_) return;
        auxithruster_data_ = *msg;
    }
    
    void battery_callback(const hal::msg::HalBattery::SharedPtr msg)
    {
        if (!active_) return;
        battery_data_ = *msg;
    }
    
    void tailservo_callback(const hal::msg::HalTailservo::SharedPtr msg)
    {
        if (!active_) return;
        tailservo_data_ = *msg;
    }
    
    void antenna_callback(const hal::msg::HalAntenna::SharedPtr msg)
    {
        if (!active_) return;
        antenna_data_ = *msg;
    }

// ================= 打包函数 =================
    std::vector<uint8_t> pack_inertial(const hal::msg::HalInertialnavi & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 8*sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.yaw, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.pitch, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.roll, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.latitude, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.longitude, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.east_velocity, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.north_velocity, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.sky_velocity, sizeof(float));

        return buf;
    }
    
    std::vector<uint8_t> pack_dvl(const hal::msg::HalDvl & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 3*sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.velocity_x, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.velocity_y, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.velocity_z, sizeof(float)); p += sizeof(float);

        return buf;
    }
    
    std::vector<uint8_t> pack_depthsensor(const hal::msg::HalDepthsensor & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(float) + 2*sizeof(uint16_t) + sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.depth_1, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.temp_1, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.depth_2, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.temp_2, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.depth_avg, sizeof(float));

        return buf;
    }
    
    std::vector<uint8_t> pack_mainthruster(const hal::msg::HalMainthruster & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 3*sizeof(int16_t) + sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.rpm, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.current, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.voltage, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.fault_status, sizeof(uint8_t)); p += sizeof(uint8_t);

        return buf;
    }
    
    std::vector<uint8_t> pack_auxithruster(const hal::msg::HalAuxithruster & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 6 * sizeof(int16_t) + 6 * sizeof(int16_t) + 6 * sizeof(uint16_t) + 6 * sizeof(uint8_t) + 6 * sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t));
        p += sizeof(int64_t);

        memcpy(p, msg.rpm.data(), 6 * sizeof(int16_t));
        p += 6 * sizeof(int16_t);

        memcpy(p, msg.current.data(), 6 * sizeof(int16_t));
        p += 6 * sizeof(int16_t);

        memcpy(p, msg.voltage.data(), 6 * sizeof(uint16_t));
        p += 6 * sizeof(uint16_t);

        memcpy(p, msg.esc_status.data(), 6 * sizeof(uint8_t));
        p += 6 * sizeof(uint8_t);

        memcpy(p, msg.fault_status.data(), 6 * sizeof(uint8_t));
        p += 6 * sizeof(uint8_t);

        return buf;
    }
    
    std::vector<uint8_t> pack_battery(const hal::msg::HalBattery & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(uint8_t) + 2*sizeof(uint16_t) + 2*sizeof(int16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 3*sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.battery_status_48v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.battery_status_72v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.battery_voltage_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_voltage_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_current_48v, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.battery_current_72v, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.cycle_count_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.cycle_count_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_temperature_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_temperature_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.remain_capacity_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.remain_capacity_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.total_capacity_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.total_capacity_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.switch_state_12v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.switch_state_24v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.switch_state_72v, sizeof(uint8_t));
        
        return buf;
    }
    
    std::vector<uint8_t> pack_tailservo(const hal::msg::HalTailservo & msg)
    {
        std::vector<uint8_t> buf( sizeof(int64_t) + 4 * sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, msg.position.data(), 4 * sizeof(float)); p += 4 * sizeof(float);

        return buf;
    }
    
    std::vector<uint8_t> pack_antenna(const hal::msg::HalAntenna & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(uint8_t) + sizeof(double));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.brake_status, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.running_status, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.total_angle, sizeof(double));

        return buf;
    }


// ================= 打印函数 =================
    void print_inertial(const hal::msg::HalInertialnavi & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Inertial]\n""timestamp: %ld | yaw: %.2f | pitch: %.2f | roll: %.2f\n""latitude: %.2f | longitude: %.2f\n""east_velocity: %.2f | north_velocity: %.2f | sky_velocity: %.2f",
        msg.timestamp,
        msg.yaw,
        msg.pitch,
        msg.roll,
        msg.latitude,
        msg.longitude,
        msg.east_velocity,
        msg.north_velocity,
        msg.sky_velocity
    );
    }
    
    void print_dvl(const hal::msg::HalDvl & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Dvl]\n""timestamp: %ld | velocity_x: %.2f | velocity_y: %.2f | velocity_z: %.2f",
        msg.timestamp,
        msg.velocity_x,
        msg.velocity_y,
        msg.velocity_z
    );
    }
    
    void print_depthsensor(const hal::msg::HalDepthsensor & msg)
    {
    RCLCPP_INFO(this->get_logger(), 
        "[Depthsensor]\n"
        "timestamp: %ld\n"
        "depth_1: %.3f m | temp_1: %u\n"
        "depth_2: %.3f m | temp_2: %u\n"
        "depth_avg: %.3f m",
        msg.timestamp,
        msg.depth_1,
        msg.temp_1,
        msg.depth_2,
        msg.temp_2,
        msg.depth_avg
    );
    }
    
    void print_mainthruster(const hal::msg::HalMainthruster & msg)
    {
    RCLCPP_INFO(this->get_logger(),
        "[MainThruster]\n"
        "timestamp: %ld\n"
        "rpm: %d\n"
        "current: %d\n"
        "voltage: %d\n"
        "fault_status: %u",
        msg.timestamp,
        msg.rpm,
        msg.current,
        msg.voltage,
        msg.fault_status
    );
    }
    
    void print_auxithruster(const hal::msg::HalAuxithruster & msg)
    {
    RCLCPP_INFO(this->get_logger(),"timestamp: %ld", msg.timestamp);

    for (size_t i = 0; i < 6; ++i)
    {
    RCLCPP_INFO(this->get_logger(), "[Thruster %zu] rpm:%d | current:%d | voltage:%u | esc:%u | fault:%u", i,msg.rpm[i],msg.current[i],msg.voltage[i],msg.esc_status[i],msg.fault_status[i]);
    }
    }
    
    void print_battery(const hal::msg::HalBattery & msg)
    {
    RCLCPP_INFO(this->get_logger(),
        "[Battery]\n"
        "ts: %ld\n"
        "status: 48V=%u | 72V=%u\n"
        "voltage: 48V=%.1fV | 72V=%.1fV\n"
        "current: 48V=%.1fA | 72V=%.1fA\n"
        "cycle: 48V=%u | 72V=%u\n"
        "temp: 48V=%uC | 72V=%uC\n"
        "remain: 48V=%.1fAh | 72V=%.1fAh\n"
        "total: 48V=%.1fAh | 72V=%.1fAh\n"
        "switch: 12V=%u | 24V=%u | 72V=%u",
        msg.timestamp,
        msg.battery_status_48v,
        msg.battery_status_72v,
        msg.battery_voltage_48v * 0.1,
        msg.battery_voltage_72v * 0.1,
        msg.battery_current_48v * 0.1,
        msg.battery_current_72v * 0.1,
        msg.cycle_count_48v,
        msg.cycle_count_72v,
        msg.battery_temperature_48v,
        msg.battery_temperature_72v,
        msg.remain_capacity_48v * 0.1,
        msg.remain_capacity_72v * 0.1,
        msg.total_capacity_48v * 0.1,
        msg.total_capacity_72v * 0.1,
        msg.switch_state_12v,
        msg.switch_state_24v,
        msg.switch_state_72v
    );
    }
    
    void print_tailservo(const hal::msg::HalTailservo & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[TailServo] timestamp: %ld", msg.timestamp);

    for (size_t i = 0; i < 4; ++i)
    {
        RCLCPP_INFO( this->get_logger(), "[TailServo %zu] position: %.2f", i, msg.position[i]);
    }
    }
    
    void print_antenna(const hal::msg::HalAntenna & msg)
    {
    RCLCPP_INFO(this->get_logger(),
        "[Antenna]\n"
        "timestamp: %ld\n"
        "brake_status: %u\n"
        "run_status: %u\n"
        "total_angle: %.3f deg",
        msg.timestamp,
        msg.brake_status,
        msg.running_status,
        msg.total_angle
    );
    }
    
    std::vector<uint8_t> build_packet(uint8_t msg_id, const std::vector<uint8_t>& payload)
    {
        uint16_t header = 0x55AA;
        uint16_t len = payload.size();

        std::vector<uint8_t> packet(sizeof(header)+1+sizeof(len)+len);

        uint8_t* p = packet.data();

        memcpy(p, &header, 2); p += 2;
        memcpy(p, &msg_id, 1); p += 1;
        memcpy(p, &len, 2); p += 2;
        memcpy(p, payload.data(), len);

        return packet;
    }
    
    // ================= UDP发送 =================
    void udp_send()
    {
        if (!active_) return;

        static int count = 0;
        bool do_print = (++count % 50 == 0);

        // ---------- Inertialnavi ----------
        if (inertial_data_.has_value())
        {
            const auto & msg = inertial_data_.value();
            if (do_print)
            {
                print_inertial(msg);
            }

            auto payload = pack_inertial(msg);
            auto packet = build_packet(0x01, payload);

            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        } 

    // ---------- DVL ----------
        if (dvl_data_.has_value())
        {
            const auto & msg = dvl_data_.value();
            if (do_print)
            {
                print_dvl(msg);
            }

            auto payload = pack_dvl(msg);
            auto packet = build_packet(0x02, payload);

            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }

    // ---------- Depthsensor ----------
        if (depthsensor_data_.has_value())
        {
            const auto & msg = depthsensor_data_.value();
            if (do_print)
            {
                print_depthsensor(msg);
            }

            auto payload = pack_depthsensor(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Mainthruster ----------
        if (mainthruster_data_.has_value())
        {
            const auto & msg = mainthruster_data_.value();
            if (do_print)
            {
                print_mainthruster(msg);
            }

            auto payload = pack_mainthruster(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Auxithruster ----------
        if (auxithruster_data_.has_value())
        {
            const auto & msg = auxithruster_data_.value();
            if (do_print)
            {
                print_auxithruster(msg);
            }

            auto payload = pack_auxithruster(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Battery ----------
        if (battery_data_.has_value())
        {
            const auto & msg = battery_data_.value();
            if (do_print)
            {
                print_battery(msg);
            }

            auto payload = pack_battery(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Tailservo ----------
        if (tailservo_data_.has_value())
        {
            const auto & msg = tailservo_data_.value();
            if (do_print)
            {
                print_tailservo(msg);
            }

            auto payload = pack_tailservo(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Antenna ----------
        if (antenna_data_.has_value())
        {
            const auto & msg = antenna_data_.value();
            if (do_print)
            {
                print_antenna(msg);
            }

            auto payload = pack_antenna(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
        
    }

private:
    rclcpp::Subscription<hal::msg::HalInertialnavi>::SharedPtr inertial_sub_;
    rclcpp::Subscription<hal::msg::HalDvl>::SharedPtr dvl_sub_;
    rclcpp::Subscription<hal::msg::HalDepthsensor>::SharedPtr depthsensor_sub_;
    rclcpp::Subscription<hal::msg::HalMainthruster>::SharedPtr mainthruster_sub_;
    rclcpp::Subscription<hal::msg::HalAuxithruster>::SharedPtr auxithruster_sub_;
    rclcpp::Subscription<hal::msg::HalBattery>::SharedPtr battery_sub_;
    rclcpp::Subscription<hal::msg::HalTailservo>::SharedPtr tailservo_sub_;
    rclcpp::Subscription<hal::msg::HalAntenna>::SharedPtr antenna_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<hal::msg::HalInertialnavi> inertial_data_;
    std::optional<hal::msg::HalDvl> dvl_data_;
    std::optional<hal::msg::HalDepthsensor> depthsensor_data_;
    std::optional<hal::msg::HalMainthruster> mainthruster_data_;
    std::optional<hal::msg::HalAuxithruster> auxithruster_data_;
    std::optional<hal::msg::HalBattery> battery_data_;
    std::optional<hal::msg::HalTailservo> tailservo_data_;
    std::optional<hal::msg::HalAntenna> antenna_data_;

    int sock_{-1};
    std::string udp_ip_;
    int udp_port_;
    struct sockaddr_in target_addr_;

    bool active_{false};
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BspCommNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
