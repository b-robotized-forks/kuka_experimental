#ifndef ROS2_CONTROL_KUKA_EKI_IO_HPP__
#define ROS2_CONTROL_KUKA_EKI_IO_HPP__

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
    // pk // This struct is used to store the information of the GPIOs defined in the URDF and KUKA EKI configuration
    // pk // It contains the interface name, command interface name, state interface name and the pin number
    // pk // The pin number is the actual pin number defined in the URDF and KUKA EKI configuration
    // pk // The idea is to use this helper struct to map the actual pin number to the name of the gpio interface
    // pk // This implementation doesn't allow for multiple command and state interfaces per GPIO and needs to be refactored if that is an issue.
    // pk // EXAMPLE URDF: /home/ws_infrastructure/src/maurob_gripper/maurob_gripper_description/urdf/maurob_steingreifer_v2.ros2_control.xacro
    // pk // This example URDF is how I think the URDF could(!) look like for the KUKA EKI IO interface (probably needs to be refactored)
    struct GpioPinInfo
    {
        std::string InterfaceName;
        std::string CommandInterfaceName;
        std::string StateInterfaceName;
        int PinNumber;

        std::string GetCommandInterfaceName() const;
        std::string GetStateInterfaceName() const;
    };

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
    using Milliseconds = boost::posix_time::milliseconds;

    const std::string LOGGER_NAME = "KukaEkiIoInterface";
    const int __maxIoNumber = 8;
    const int __myCustomTemporaryDefaultValue = -42069;
    const int __ekiModeWrite = 2;
    const int __ekiModeRead = 1;

    bool isValidIPv4(const std::string& ipString);
    bool isInteger(const std::string& s);
    const std::string getInElementNameByIndex(int index);
    const std::string getOutElementNameByIndex(int index);

    class KukaEkiIoInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(kuka_eki_io_interface::KukaEkiIoInterface)
            // virtual ~KukaEkiIoInterface();

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;
            // pk // on_configure and on_cleanup are currently not used, but could be used in the future to configure and cleanup the interface

            // std::vector<hardware_interface::StateInterface> export_state_interfaces() final;                                 // pk // DEPRECATED USE on_export_state_interfaces instead
            // std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;                             // pk // DEPRECATED USE on_export_command_interfaces instead
            // virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() final;      // pk // !! If hardware_interface::SystemInterface is implemented correctly and used as intended you do not need to override this function !!
            // virtual std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() final;       // pk // !! If hardware_interface::SystemInterface is implemented correctly and used as intended you do not need to override this function !!

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;
        private:
            int numberOfIos_;

            std::unordered_map<int, GpioPinInfo> gpioInfos_;        // pk // This maps the actual pin number defined in the URDF and KUKA EKI configuration to the name of the gpio interface
            
            std::string eki_server_address_;
            std::string eki_server_port_;

            int eki_cmd_buff_len_;
            int eki_max_cmd_buff_len_ = 5; // by default, limit command buffer to 5 (size of advance run in KRL)

            // EKI socket read/write
            int eki_read_state_timeout_ = 5;  // [ms]; settable by parameter (default = 5)
            IoService ios_;
            DeadlineTimerPtr deadline_;
            Endpoint eki_server_endpoint_;
            SocketPtr eki_server_socket_;

            void eki_check_read_state_deadline();
            //static void eki_handle_receive(const boost::system::error_code& ec, size_t length, boost::system::error_code*  out_ec, size_t*  out_length);
            bool eki_read_state(std::vector<int>& inKeys, std::vector<bool>& inValues, std::vector<int>& outKeys, std::vector<bool>& outValues);
            bool eki_write_command(const std::vector<int>& io_pins, const std::vector<bool>& target_ios);
            static int ekiCommandBufferSize_;

            bool setInternalStates(const std::string& interfaceName, const std::vector<int>& inKeys, const std::vector<bool>& inValues);
            bool getInternalCommands(); //std::vector<int>& ioPins, std::vector<bool>& ioStates);

            bool parseKeyAndValue(tinyxml2::XMLElement* xmlIoState, const std::string& xmlChildElementName, int& key, bool& value);
    };
} // namespace kuka_eki_io_interface

#endif // ROS2_CONTROL_KUKA_EKI_IO_HPP__