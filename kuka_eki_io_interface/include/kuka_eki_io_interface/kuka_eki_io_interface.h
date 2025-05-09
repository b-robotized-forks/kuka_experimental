#ifndef ROS2_CONTROL_KUKA_EKI_IO_HPP_
#define ROS2_CONTROL_KUKA_EKI_IO_HPP_

#include <vector>
#include <string>
#include <map>

#include <boost/asio.hpp>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"



namespace kuka_eki_io_interface
{
    //#define USE_EXPORT_OVERRIDES

    using Socket = boost::asio::ip::udp::socket;
    using SocketPtr = std::unique_ptr<boost::asio::ip::udp::socket>;
    using Endpoint = boost::asio::ip::udp::endpoint;
    using IoService = boost::asio::io_service;
    using Resolver = boost::asio::ip::udp::resolver;
    using SystemErrorCode = boost::system::error_code;
    using DeadlineTimer = boost::asio::deadline_timer;
    using DeadlineTimerPtr = std::unique_ptr<boost::asio::deadline_timer>;
    using Udp = boost::asio::ip::udp;
    using Seconds = boost::posix_time::seconds;

    const std::string LOGGER_NAME = "KukaEkiIoInterface";
    const std::string ioNames[] = {"IO1", "IO2", "IO3", "IO4", "IO5", "IO6", "IO7", "IO8"};
    const int __maxIoNumber = 8;
    const int __myCustomTemporaryDefaultValue = -42069;
    const int __pinNumberOffset = 500;
    const int __defaultNumberOfIos = 8;
    const int __ekiModeWrite = 2;
    const int __ekiModeRead = 1;

    struct GpioPinInfo
    {
        std::string name; // e.g., "cmd_gripper_open"
        int pin_number;
        std::string command_interface_name; // e.g., "cmd_gripper_open/set_value"
        std::string state_interface_name;   // e.g., "state_gripper_opened/get_value" or "cmd_gripper_open/commanded_value"
        hardware_interface::CommandInterface::SharedPtr command_interface;
        hardware_interface::StateInterface::SharedPtr state_interface;
    };

    class KukaEkiIoInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(KukaEkiIoInterface)
            virtual ~KukaEkiIoInterface();

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

            // std::vector<hardware_interface::StateInterface> export_state_interfaces() final;                             // pk // DEPRECATED USE on_export_state_interfaces instead
            // std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;                         // pk // DEPRECATED USE on_export_command_interfaces instead
            virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() final;     // pk // !! If hardware_interface::SystemInterface is implemented correctly and used as intended you do not need to override this function !!
            virtual std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() final;      // pk // !! If hardware_interface::SystemInterface is implemented correctly and used as intended you do not need to override this function !!

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

            bool eki_read_state(std::vector<bool>& io_states, std::vector<int>& io_pins);
            bool eki_write_command(const std::vector<int>& io_pins, const std::vector<bool>& target_ios);
    
            KukaEkiIoInterface(const std::string& eki_server_address, const std::string& eki_server_port, int n_io);
        private:
            int numberOfIos_;

            // Store the command for the simulated robot
            //std::vector<bool> ioStates_;
            //std::vector<bool> ioCommands_;
            //std::vector<int> ioPins_;

            std::unordered_map<int, GpioPinInfo> gpioInfo_;
            
            std::string eki_server_address_;
            std::string eki_server_port_;

            int eki_cmd_buff_len_;
            int eki_max_cmd_buff_len_ = 5; // by default, limit command buffer to 5 (size of advance run in KRL)

            // EKI socket read/write
            int eki_read_state_timeout_ = 100;  // [s]; settable by parameter (default = 5)
            IoService ios_;
            DeadlineTimerPtr deadline_;
            Endpoint eki_server_endpoint_;
            SocketPtr eki_server_socket_;

            void eki_check_read_state_deadline();
            static void eki_handle_receive(const boost::system::error_code& ec, size_t length, boost::system::error_code*  out_ec, size_t*  out_length);
            bool eki_read_state(std::vector<bool>& ioStates, std::vector<int>& ioPins);
            bool eki_write_command(const std::vector<double>& joint_position);

            static int ekiCommandBufferSize_;

    };
} // namespace kuka_eki_io_interface

#endif // ROS2_CONTROL_KUKA_EKI_IO_HPP_