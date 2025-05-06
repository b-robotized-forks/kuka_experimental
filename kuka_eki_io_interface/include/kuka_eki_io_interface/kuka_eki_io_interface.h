#ifndef ROS2_CONTROL_KUKA_EKI_IO_HPP_
#define ROS2_CONTROL_KUKA_EKI_IO_HPP_

#include <vector>
#include <string>

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
    class KukaEkiIoInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(KukaEkiIoInterface)
            virtual ~KukaEkiIoInterface();

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

            bool eki_read_state(std::vector<bool>& io_states, std::vector<int>& io_pins, std::vector<int>& io_types, int& cmd_buff_len);
            bool eki_write_command(const std::vector<int>& io_pins, const std::vector<int>& io_modes,const std::vector<bool>& target_ios);

            virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() final;
            virtual std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() final;
    
            KukaEkiIoInterface(const std::string& eki_server_address, const std::string& eki_server_port, int n_io);
        private:
            int numberOfIos_ = 8;

            // Store the command for the simulated robot
            std::vector<double> hw_commands_;
            std::vector<double> hw_states_;

            std::string eki_server_address_;
            std::string eki_server_port_;

            int eki_cmd_buff_len_;
            int eki_max_cmd_buff_len_ = 5; // by default, limit command buffer to 5 (size of advance run in KRL)

            // EKI socket read/write
            int eki_read_state_timeout_ = 100;  // [s]; settable by parameter (default = 5)
            boost::asio::io_service ios_;
            std::unique_ptr<boost::asio::deadline_timer> deadline_;
            boost::asio::ip::udp::endpoint eki_server_endpoint_;
            std::unique_ptr<boost::asio::ip::udp::socket> eki_server_socket_;

            void eki_check_read_state_deadline();
            static void eki_handle_receive(const boost::system::error_code& ec, size_t length, boost::system::error_code*  out_ec, size_t*  out_length);
            bool eki_read_state(std::vector<double>& joint_position, std::vector<double>& joint_velocity, std::vector<double>& joint_effort, int& cmd_buff_len);
            bool eki_write_command(const std::vector<double>& joint_position);

    };
} // namespace kuka_eki_io_interface

#endif // ROS2_CONTROL_KUKA_EKI_IO_HPP_