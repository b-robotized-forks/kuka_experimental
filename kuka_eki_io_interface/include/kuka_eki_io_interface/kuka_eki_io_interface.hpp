#ifndef ROS2_CONTROL_KUKA_EKI_IO_HPP__
#define ROS2_CONTROL_KUKA_EKI_IO_HPP__

#include <vector>
#include <string>
#include <map>
#include <boost/asio.hpp>
#include <tinyxml2.h>

// ROS2
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ROS2 // ros2_control
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace kuka_eki_io_interface
{
    // pk // EXAMPLE URDF: /home/ws_infrastructure/src/maurob_gripper/maurob_gripper_description/urdf/maurob_steingreifer_v2.ros2_control.xacro
    // pk // This example URDF is how I think the URDF could(!) look like for the KUKA EKI IO interface (probably needs to be refactored)
    
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

    const std::string EKI_XML_ELEMENT_STATE = "IoState";
    const std::string EKI_XML_ELEMENT_REQUEST = "IoRequest";
    const std::string EKI_XML_ELEMENT_REQUEST_ID = "RequestId";
    const std::string EKI_XML_ELEMENT_PREFIX_IN = "In";
    const std::string EKI_XML_ELEMENT_PREFIX_OUT = "Out";
    const std::string EKI_XML_ATTRIBUTE_KEY = "Key";
    const std::string EKI_XML_ATTRIBUTE_VALUE = "Value";
    const std::string LOGGER_NAME = "KukaEkiIoInterface";
    const int __maxIoNumber = 8;

    class KukaEkiIoInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(kuka_eki_io_interface::KukaEkiIoInterface)
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;
        private:
            // Data Members // Tests
            const std::string XML_READ_EXAMPLE = "<IoState StateId=\"697\"><In0 Key=\"500\" Value=\"1\"></In0><In1 Key=\"501\" Value=\"0\"></In1><In2 Key=\"502\" Value=\"1\"></In2><In3 Key=\"503\" Value=\"0\"></In3><In4 Key=\"504\" Value=\"0\"></In4><In5 Key=\"505\" Value=\"0\"></In5><In6 Key=\"506\" Value=\"0\"></In6><In7 Key=\"507\" Value=\"0\"></In7 RequestId=\"11\"><Out0 Key=\"500\" Value=\"0\"></Out0><Out1 Key=\"501\" Value=\"0\"></Out1><Out2 Key=\"502\" Value=\"0\"></Out2><Out3 Key=\"503\" Value=\"1\"></Out3><Out4 Key=\"504\" Value=\"0\"></Out4><Out5 Key=\"505\" Value=\"0\"></Out5><Out6 Key=\"506\" Value=\"0\"></Out6><Out7 Key=\"507\" Value=\"0\"></Out7></IoState>";
            const std::string XML_READ_EXAMPLE_OUTS_ARE_ZERO = "<IoState StateId=\"697\"><In0 Key=\"500\" Value=\"1\"></In0><In1 Key=\"501\" Value=\"0\"></In1><In2 Key=\"502\" Value=\"1\"></In2><In3 Key=\"503\" Value=\"0\"></In3><In4 Key=\"504\" Value=\"0\"></In4><In5 Key=\"505\" Value=\"0\"></In5><In6 Key=\"506\" Value=\"0\"></In6><In7 Key=\"507\" Value=\"0\"></In7 RequestId=\"11\"><Out0 Key=\"500\" Value=\"0\"></Out0><Out1 Key=\"501\" Value=\"0\"></Out1><Out2 Key=\"502\" Value=\"0\"></Out2><Out3 Key=\"503\" Value=\"0\"></Out3><Out4 Key=\"504\" Value=\"0\"></Out4><Out5 Key=\"505\" Value=\"0\"></Out5><Out6 Key=\"506\" Value=\"0\"></Out6><Out7 Key=\"507\" Value=\"0\"></Out7></IoState>";
            const std::string XML_WRITE_EXAMPLE = "<IoRequest RequestId=\"11\"><Out0 Key=\"500\" Value=\"0\"/><Out1 Key=\"501\" Value=\"0\"/><Out2 Key=\"502\" Value=\"0\"/><Out3 Key=\"503\" Value=\"0\"/><Out4 Key=\"504\" Value=\"0\"/><Out5 Key=\"505\" Value=\"0\"/><Out6 Key=\"506\" Value=\"0\"/><Out7 Key=\"507\" Value=\"0\"/><In0 Key=\"500\"/><In1 Key=\"501\"/><In2 Key=\"502\"/><In3 Key=\"503\"/><In4 Key=\"504\"/><In5 Key=\"505\"/><In6 Key=\"506\"/><In7 Key=\"507\"/></IoRequest>";

            // Data Members
            std::string eki_server_address_;
            std::string eki_server_port_;
            std::string eki_io_port_;
            std::vector<std::string> orderedCommandFullNames_;
            std::vector<std::string> orderedCommandNames_;
            std::vector<std::string> orderedCommandStateFullNames_;
            std::vector<std::string> orderedCommandStateNames_;
            std::vector<std::string> orderedCommandStateKeys_;
            std::vector<std::string> orderedNonCommandStateFullNames_;
            std::vector<std::string> orderedNonCommandStateNames_;
            std::vector<std::string> orderedNonCommandStateKeys_;
            int lastRequestId_;

            // EKI socket read/write
            int eki_read_state_timeout_ = 5;  // [ms], settable by parameter (default = 5)
            IoService ios_;
            DeadlineTimerPtr deadline_;
            Endpoint eki_server_endpoint_;
            SocketPtr eki_server_socket_;

            // Setup
            void eki_check_read_state_deadline();
            static void eki_handle_receive(const boost::system::error_code& systemErrorCode, size_t length, boost::system::error_code* out_ec, size_t* out_length);
            bool isValidIPv4(const std::string& ipString);
            hardware_interface::return_type assignOrderedInterfaceNames();
            hardware_interface::return_type assignEkiConfiguration();
            std::string removeInterfacePrefix(const std::string& interfaceName, const std::string& prefixName);

            // Read states
            hardware_interface::return_type eki_read_state();
            hardware_interface::return_type readIoValuesFromXmlIo(tinyxml2::XMLElement* xmlIo, int& key, bool& value);
            hardware_interface::return_type getNonCommandStateFullNameByKey(const std::string& key, std::string& fullname);
            hardware_interface::return_type getCommandStateFullNameByKey(const std::string& key, std::string& fullname);

            // Write commands
            bool isCommandUpdateRequired();
            hardware_interface::return_type eki_write_command();
            int getCommandAsValidDigit(const std::string& commandName);
            const std::string getInElementNameByIndex(int index);
            const std::string getOutElementNameByIndex(int index);
    };
} // namespace kuka_eki_io_interface

#endif // ROS2_CONTROL_KUKA_EKI_IO_HPP__