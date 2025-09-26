#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>
#include <boost/chrono.hpp>
#include <tinyxml2.h>
#include <regex>

#include <kuka_eki_io_interface/kuka_eki_io_interface.hpp>

using namespace boost::placeholders;

namespace kuka_eki_io_interface
{
    hardware_interface::CallbackReturn KukaEkiIoInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_init() called.");

        promise_write_ = boost::promise<hardware_interface::return_type>();
        future_write_ = promise_write_.get_future();
        promise_write_.set_value(hardware_interface::return_type::OK);


        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        
        if (info_.gpios.size() > __maxIoNumber) {
            RCLCPP_FATAL(logger, "on_init() called. Detected %ld GPIOs. Hardware Interface supports only %i GPIOs.", info_.gpios.size() , __maxIoNumber);
            return hardware_interface::CallbackReturn::ERROR;
        }      

        RCLCPP_INFO(logger, "on_init() called. Detected %ld GPIOs.", info_.gpios.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_configure() called. Previous state was [ %i, %s ]", previous_state.id(), previous_state.label().c_str());

        if (configureInterfaces()==hardware_interface::return_type::ERROR)
            return hardware_interface::CallbackReturn::ERROR;

        if (assignEkiConfiguration()==hardware_interface::return_type::ERROR)
            return hardware_interface::CallbackReturn::ERROR;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_activate() called. Previous state was [ %i, %s ]", previous_state.id(), previous_state.label().c_str());        

        deadline_.reset(new boost::asio::deadline_timer(ios_));
        eki_server_socket_.reset(new boost::asio::ip::udp::socket(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)));

        boost::asio::ip::udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_io_port_});

        // Initiate contact to start server. Do nothing until a read is invoked (deadline_ = +inf)
        boost::array<char, 1> ini_buf = {0};
        eki_server_socket_->send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);

        // Start persistent actor to check for eki_read_state timeouts.
        deadline_->expires_at(boost::posix_time::pos_infin);  // 
        eki_check_read_state_deadline();

        RCLCPP_INFO(logger, "KUKA EKI IO interface activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_deactivate() called. Previous state was [ %i, %s ]", previous_state.id(), previous_state.label().c_str());

        // Stop the deadline timer
        deadline_->cancel();

        // Close the socket
        eki_server_socket_->close();

        // Reset ordered interface names.
        command_interfaces_info_.clear();
        command_state_interfaces_info_.clear();
        non_command_state_interfaces_info_.clear();

        RCLCPP_INFO(logger, "KUKA EKI IO interface deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void KukaEkiIoInterface::eki_check_read_state_deadline() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Check, if deadline has already passed.
        if (deadline_->expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
            deadline_->expires_at(boost::posix_time::pos_infin);
            eki_server_socket_->cancel();
        }

        // Sleep until deadline exceeded
        deadline_->async_wait(boost::bind(&KukaEkiIoInterface::eki_check_read_state_deadline, this));
    }

    void KukaEkiIoInterface::eki_handle_receive(const boost::system::error_code& systemErrorCode, size_t length, boost::system::error_code* out_ec, size_t* out_length) {
        *out_ec = systemErrorCode;
        *out_length = length;
    }

    bool KukaEkiIoInterface::isValidIPv4(const std::string& ipString) {
        static const std::regex ipv4Regex(
            "^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
            "(:(6553[0-5]|655[0-2][0-9]|65[0-4][0-9]{2}|6[0-4][0-9]{3}|[1-5][0-9]{4}|[1-9][0-9]{0,3}))?$"
        );
        
        return std::regex_match(ipString, ipv4Regex);
    }

    hardware_interface::return_type KukaEkiIoInterface::configureInterfaces() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Reset ordered interface names.
        command_interfaces_info_.clear();
        command_state_interfaces_info_.clear();
        non_command_state_interfaces_info_.clear();

        // Reset Id tracking.
        lastRequestId_ = 0;

        RCLCPP_INFO(logger, "--- Configuring EKI Interfaces from URDF ---");

        for (const auto& gpio : info_.gpios) {
            RCLCPP_INFO(logger, "Processing GPIO component: %s", gpio.name.c_str());

            for (const auto& command_if : gpio.command_interfaces) {
                EkiInterfaceInfo info;
                info.name = command_if.name;
                if (command_if.parameters.find("EKI_key") == command_if.parameters.end()) {
                    RCLCPP_FATAL(logger, "Command interface '%s' is missing the 'EKI_key' parameter.", info.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }
                info.key = command_if.parameters.at("EKI_key");

                try {
                    std::stoi(info.key);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_FATAL(
                        logger, "EKI_key '%s' for command interface '%s' is not a valid integer.",
                        info.key.c_str(), info.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }

                command_interfaces_info_.push_back(info);

                RCLCPP_INFO(logger, "  [Command] '%s' -> Key: %s (Type: %s)", info.name.c_str(), info.key.c_str(), command_if.data_type.c_str());
            }

            for (const auto& state_if : gpio.state_interfaces) {
                EkiInterfaceInfo info;
                info.name = state_if.name;

                if (state_if.parameters.find("EKI_key") == state_if.parameters.end()) {
                    RCLCPP_FATAL(logger, "State interface '%s' is missing the 'EKI_key' parameter.", info.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }
                info.key = state_if.parameters.at("EKI_key");

                try {
                    std::stoi(info.key);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_FATAL(
                        logger, "EKI_key '%s' for command interface '%s' is not a valid integer.",
                        info.key.c_str(), info.name.c_str());
                    return hardware_interface::return_type::ERROR;
                }

                // Check if this state interface has a corresponding command interface
                auto it = std::find_if(command_interfaces_info_.begin(), command_interfaces_info_.end(),
                                       [&info](const EkiInterfaceInfo& cmd_info) {
                                           return cmd_info.name == info.name;
                                       });

                if (it != command_interfaces_info_.end()) {
                    command_state_interfaces_info_.push_back(info);
                    RCLCPP_INFO(logger, "  [Cmd State] '%s' -> Key: %s (Type: %s)", info.name.c_str(), info.key.c_str(), state_if.data_type.c_str());
                } else {
                    non_command_state_interfaces_info_.push_back(info);
                    RCLCPP_INFO(logger, "  [State] '%s' -> Key: %s (Type: %s)", info.name.c_str(), info.key.c_str(), state_if.data_type.c_str());
                }
            }
        }

        // Sort by the 'key' member of the struct to ensure the XML is generated in a consistent order.
        auto sort_by_key = [](const EkiInterfaceInfo& a, const EkiInterfaceInfo& b) {
            return std::stoi(a.key) < std::stoi(b.key);
        };
        std::sort(command_interfaces_info_.begin(), command_interfaces_info_.end(), sort_by_key);
        std::sort(command_state_interfaces_info_.begin(), command_state_interfaces_info_.end(), sort_by_key);
        std::sort(non_command_state_interfaces_info_.begin(), non_command_state_interfaces_info_.end(), sort_by_key);

        RCLCPP_INFO(logger, "--- Interface configuration finished ---");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::assignEkiConfiguration() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        eki_server_address_ = info_.hardware_parameters["robot_ip"];
        eki_io_port_ = info_.hardware_parameters["eki_io_port"]; //"54601";

        std::string completeAddress = eki_server_address_ + ":" + eki_io_port_;
        RCLCPP_INFO(logger, "Configured EKI Server Url: %s", completeAddress.c_str());
        if (eki_server_address_.empty() || eki_io_port_.empty()) {
            RCLCPP_FATAL(logger, "robot_ip or eki_io_port cannot be empty");
            return hardware_interface::return_type::ERROR;
        }
        if (!isValidIPv4(completeAddress)) {
            RCLCPP_FATAL(logger, "Invalid IP address: %s", completeAddress.c_str());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_DEBUG(logger, "write() called. Time=[%f , %li] Duration=[ %f , %li ]", time.seconds(), time.nanoseconds(), period.seconds(), period.nanoseconds());

        if (eki_read_state() == hardware_interface::return_type::ERROR) {
            std::string msg = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure kuka_eki_io_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            RCLCPP_ERROR(logger, "Configured EKI Server Address: %s", eki_server_address_.c_str());
            RCLCPP_ERROR(logger, "Configured EKI Server Port   : %s", eki_io_port_.c_str());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::eki_read_state() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Declarations & Allocations
        static boost::array<char, 2048> inBuffer;

        // Read socket buffer (with timeout) // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_->expires_from_now(boost::posix_time::milliseconds(eki_read_state_timeout_));
        boost::system::error_code systemErrorCode = boost::asio::error::would_block;
        size_t receivedMessageLength = 0;

        eki_server_socket_->async_receive(boost::asio::buffer(inBuffer), boost::bind(&KukaEkiIoInterface::eki_handle_receive, _1, _2, &systemErrorCode, &receivedMessageLength));
        // eki_server_socket_->async_receive(
        //     boost::asio::buffer(inBuffer),
        //     [&systemErrorCode, &receivedMessageLength](const boost::system::error_code& errorCode, std::size_t length) {
        //         systemErrorCode = errorCode;
        //         receivedMessageLength = length;
        //     }
        // );

        do
            ios_.run_one();
        while (systemErrorCode == boost::asio::error::would_block);

        deadline_->expires_at(boost::posix_time::pos_infin);

        // KUKAEKIIO_00001 // KUKAEKIIO_00002 // Log warning when errorcode is set and do not continue processing.
        if (systemErrorCode) {
            RCLCPP_DEBUG(logger, "communication error code: %s", systemErrorCode.message().c_str());
            return hardware_interface::return_type::OK;
        }

        // KUKAEKIIO_00003 // KUKAEKIIO_00004 // Log warning when packages with zero lenght are received and do not continue processing.
        if (receivedMessageLength == 0) {
            RCLCPP_WARN(logger, "message of length 0 received.");
            return hardware_interface::return_type::OK;
        }

        // KUKAEKIIO_00006 // Materialize incoming c-string as XML DOM.  
        tinyxml2::XMLDocument xmlDocument;
        tinyxml2::XMLError xmlDocumentParseError = xmlDocument.Parse(inBuffer.data(), receivedMessageLength);
        // tinyxml2::XMLError xmlDocumentParseError = xmlDocument.Parse(XML_READ_EXAMPLE_OUTS_ARE_ZERO.c_str(), XML_READ_EXAMPLE_OUTS_ARE_ZERO.size());
        // TODO // ReH -> FoN // END // Change from test xml to real xml.

        // TODO // HAndle parse error.
        if (xmlDocumentParseError != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(logger, "Error parsing incoming XML telegram: %s", xmlDocument.ErrorStr());
            return hardware_interface::return_type::ERROR;
        }

        // KUKAEKIIO_00009 // Log debug the XML DOM.
        tinyxml2::XMLPrinter printer(0, true, 0);
        xmlDocument.Print(&printer);
        RCLCPP_DEBUG(logger, "Received EKI XML: %s", printer.CStr());

        tinyxml2::XMLElement* xmlIoState = xmlDocument.FirstChildElement(EKI_XML_ELEMENT_STATE.c_str());

        // KUKAEKIIO_00007 // KUKAEKIIO_00008 // Log warning when no "IOState" is child-element contained and do not continue processing. 
        if (!xmlIoState) {
            RCLCPP_ERROR(logger, "no %s-element found in XML.", EKI_XML_ELEMENT_STATE.c_str());
            return hardware_interface::return_type::ERROR;
        }

        // Transform data from XML DOM into an c-typed structure.
        int key = 0;
        bool value = false;

        for (tinyxml2::XMLElement* xmlIo = xmlIoState->FirstChildElement(); xmlIo != nullptr; xmlIo = xmlIo->NextSiblingElement()) {
            std::string name = xmlIo->Name();

            if (readIoValuesFromXmlIo(xmlIo, key, value) != hardware_interface::return_type::OK) {
                RCLCPP_ERROR(logger, "Failed to read values from %s.", name.c_str());
                continue; // Try next element
            }

            std::string key_str = std::to_string(key);
            
            if (name.find(EKI_XML_ELEMENT_PREFIX_IN.c_str()) != std::string::npos) {
                auto it = std::find_if(non_command_state_interfaces_info_.begin(), non_command_state_interfaces_info_.end(),
                                       [&key_str](const EkiInterfaceInfo& info) { return info.key == key_str; });

                if (it != non_command_state_interfaces_info_.end()) {
                    set_state(it->name, value ? 1.0 : 0.0);
                    RCLCPP_DEBUG(logger, "Setting state '%s' (Key: %s) to %f.", it->name.c_str(), it->key.c_str(), value ? 1.0 : 0.0);
                } else {
                    RCLCPP_ERROR(logger, "Received state for unconfigured INPUT Key: %s", key_str.c_str());
                    return hardware_interface::return_type::ERROR;
                }

            } else if (name.find(EKI_XML_ELEMENT_PREFIX_OUT.c_str()) != std::string::npos) {
                 auto it = std::find_if(command_state_interfaces_info_.begin(), command_state_interfaces_info_.end(),
                                       [&key_str](const EkiInterfaceInfo& info) { return info.key == key_str; });
                if (it != command_state_interfaces_info_.end()) {
                    set_state(it->name, value ? 1.0 : 0.0);
                     RCLCPP_DEBUG(logger, "Setting state '%s' (Key: %s) to %f.", it->name.c_str(), it->key.c_str(), value ? 1.0 : 0.0);
                } else {
                    RCLCPP_ERROR(logger, "Received state for unconfigured OUTPUT Key: %s", key_str.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::readIoValuesFromXmlIo(tinyxml2::XMLElement* xmlIo, int& key, bool& value) {
        if (xmlIo->QueryIntAttribute(EKI_XML_ATTRIBUTE_KEY.c_str(), &key) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to query '%s' attribute or attribute is not an int for %s.", EKI_XML_ATTRIBUTE_KEY.c_str(), xmlIo->Name());
            return hardware_interface::return_type::ERROR;
        }
        if (xmlIo->QueryBoolAttribute(EKI_XML_ATTRIBUTE_VALUE.c_str(), &value) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to query '%s' attribute or attribute is not an bool for %s.", EKI_XML_ATTRIBUTE_VALUE.c_str(), xmlIo->Name());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_DEBUG(logger, "write() called. Time=[%f , %li] Duration=[ %f , %li ]", time.seconds(), time.nanoseconds(), period.seconds(), period.nanoseconds());

        if (mutex_write_.try_lock()) {
            if (future_write_.is_ready() && isCommandUpdateRequired()) {
                RCLCPP_DEBUG(logger, "Is ready.");
                future_write_ = boost::async([this]() { return write_throttle(); });
            }

            mutex_write_.unlock();
        }
       
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::write_throttle() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_DEBUG(logger, "WRITE / Before sleep.");
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        RCLCPP_DEBUG(logger, "WRITE / After sleep.");
        if (eki_write_command() == hardware_interface::return_type::ERROR) {
            std::string msg = "Failed to write to robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_io_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            return hardware_interface::return_type::ERROR;
        }
        RCLCPP_DEBUG(logger, "WRITE / OKAY.");

        return  hardware_interface::return_type::OK;
    }

    bool KukaEkiIoInterface::isCommandUpdateRequired() {
        for (const auto& info : command_state_interfaces_info_) {
            try {
                if (std::isnan(get_command(info.name))) {
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Command update not required. Command '%s' was NaN.", info.name.c_str());
                    return false;
                }
                if (get_command(info.name) != get_state(info.name)) {
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Command update required for '%s': command=%.1f, state=%.1f",
                                 info.name.c_str(), get_command(info.name), get_state(info.name));
                    return true;
                }
            } catch (const std::runtime_error& e) {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Error accessing interface '%s' in isCommandUpdateRequired(): %s", info.name.c_str(), e.what());
            }
        }
        return false;
    }

    hardware_interface::return_type KukaEkiIoInterface::eki_write_command() {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Temporary unique request id rotates between 1 and 60000.
        lastRequestId_ = lastRequestId_ > 60000 ? 1 : lastRequestId_ + 1;

        // Building XML DOM.
        tinyxml2::XMLDocument xmlCommand;
        auto ioRequest = xmlCommand.NewElement(EKI_XML_ELEMENT_REQUEST.c_str());
        ioRequest->SetAttribute(EKI_XML_ELEMENT_REQUEST_ID.c_str(), lastRequestId_);
        
        for (std::size_t index = 0; index < command_interfaces_info_.size(); ++index) {
            const auto& info = command_interfaces_info_[index];
            tinyxml2::XMLElement* outElement = xmlCommand.NewElement(getOutElementNameByIndex(index).c_str());
            outElement->SetAttribute(EKI_XML_ATTRIBUTE_KEY.c_str(), info.key.c_str());
            outElement->SetAttribute(EKI_XML_ATTRIBUTE_VALUE.c_str(), getCommandAsValidDigit(info.name));
            ioRequest->InsertEndChild(outElement);
        }
        for (std::size_t index = 0; index < non_command_state_interfaces_info_.size(); ++index) {
            const auto& info = non_command_state_interfaces_info_[index];
            tinyxml2::XMLElement* inElement = xmlCommand.NewElement(getInElementNameByIndex(index).c_str());
            inElement->SetAttribute(EKI_XML_ATTRIBUTE_KEY.c_str(), info.key.c_str());
            ioRequest->InsertEndChild(inElement);
        }

        xmlCommand.InsertEndChild(ioRequest);

        // Building XML String.
        tinyxml2::XMLPrinter printer(0, true, 0);
        xmlCommand.Print(&printer);
        
        // Send XML String to EKI Server.
        try {
            eki_server_socket_->send_to(boost::asio::buffer(printer.CStr(), printer.CStrSize()), eki_server_endpoint_);
            RCLCPP_DEBUG(logger, "Sending EKI XML: %s", printer.CStr());
            return hardware_interface::return_type::OK;
        } catch(const std::exception& e) {
            RCLCPP_ERROR(logger, "Sending EKI XML: %s resulted in exception: %s", printer.CStr(), e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    int KukaEkiIoInterface::getCommandAsValidDigit(const std::string& commandName) {
        double noDigit = get_command(commandName);
        return std::isnan(noDigit) || noDigit == 0.0 ? 0 : 1;
    }

    const std::string KukaEkiIoInterface::getInElementNameByIndex(int index) {
        return EKI_XML_ELEMENT_PREFIX_IN + std::to_string(index);
    }

    const std::string KukaEkiIoInterface::getOutElementNameByIndex(int index) {
        return EKI_XML_ELEMENT_PREFIX_OUT + std::to_string(index);
    }

}  // namespace kuka_eki_io_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kuka_eki_io_interface::KukaEkiIoInterface, hardware_interface::SystemInterface)