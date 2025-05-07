#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tinyxml2.h>
#include <regex>

#include <kuka_eki_io_interface/kuka_eki_io_interface.h>

namespace kuka_eki_io_interface
{
    KukaEkiIoInterface::~KukaEkiIoInterface() 
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Destructor called. Cleaning up ...");
        if (eki_server_socket_)
        {
            eki_server_socket_->close();
            eki_server_socket_.reset();
        }
        on_deactivate(rclcpp_lifecycle::State());
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_init() called");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        info_ = info; // pk // Is this even necessary?

        if (__maxIoNumber < info_.gpios.size())
        {
            RCLCPP_FATAL(logger, "KUKA EKI IO interface only supports a maximum of %d GPIOs.", __maxIoNumber);
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Check if the number of command interfaces and state interfaces is correct
        for (const hardware_interface::ComponentInfo& gpio : info_.gpios)
        {
            if (gpio.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(logger, "KUKA EKI IO interface only supports one command interface per GPIO.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (gpio.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(logger, "KUKA EKI IO interface only supports one state interface per GPIO.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (gpio.parameters.find("pin") == gpio.parameters.end())
            {
                RCLCPP_FATAL(logger, "KUKA EKI IO interface requires a pin parameter for each GPIO.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else 
            {
                if (!isInteger(gpio.parameters.at("pin")))
                {
                    RCLCPP_FATAL(logger, "KUKA EKI IO interface pin parameter must be an integer.");
                    return hardware_interface::CallbackReturn::ERROR;
                }

                int pin = std::stoi(gpio.parameters.at("pin"));
                ioPins_.push_back(pin);     // pk // This looks kinda hacky but may not be required if hardware_interface::SystemInterface is implemented correctly. 
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_activate() called");

        eki_server_address_ = info_.hardware_parameters["robot_ip"];
        eki_server_port_ = info_.hardware_parameters["eki_robot_port"];
        numberOfIos_ = info_.gpios.size();

        std::string completeAddress = eki_server_address_ + ":" + eki_server_port_;
        RCLCPP_INFO(logger, "using IP and port: %s", completeAddress.c_str());
        if (eki_server_address_.empty() || eki_server_port_.empty())
        {
            RCLCPP_FATAL(logger, "robot_ip or eki_robot_port cannot be empty");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!isValidIPv4(completeAddress))
        {
            RCLCPP_FATAL(logger, "Invalid IP address: %s", completeAddress.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        deadline_.reset(new DeadlineTimer(ios_));
        eki_server_socket_.reset(new Socket(ios_, Endpoint(Udp::v4(), 0)));

        Udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({Udp::v4(), eki_server_address_, eki_server_port_});

        boost::array<char, 1> ini_buf = {0};
        eki_server_socket_->send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);  // initiate contact to start server

        // Start persistent actor to check for eki_read_state timeouts
        deadline_->expires_at(boost::posix_time::pos_infin);  // do nothing until a read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();

        std::vector<bool> ioStates;
        std::vector<int> ioPins;
        if (!eki_read_state(ioStates, ioPins))
        {
            std::string errorMessage = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_FATAL(logger, errorMessage.c_str());
            throw std::runtime_error(errorMessage);
        }

        ioStates_ = ioStates;
        ioCommands_ = ioStates;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        info_ = info; // pk // Where does this come from? Probably inherited from SystemInterface.
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void KukaEkiIoInterface::eki_check_read_state_deadline()
    {
        // Check if deadline has already passed
        if (deadline_->expires_at() <= boost::asio::deadline_timer::traits_type::now())
        {
            eki_server_socket_->cancel();
            deadline_->expires_at(boost::posix_time::pos_infin);
        }
        // Sleep until deadline exceeded
        deadline_->async_wait(boost::bind(&KukaEkiIoInterface::eki_check_read_state_deadline, this));
    }

    void KukaEkiIoInterface::eki_handle_receive(const SystemErrorCode& systemErrorCode, size_t length, SystemErrorCode* out_ec, size_t* out_length)
    {
        *out_ec = systemErrorCode;
        *out_length = length;
    }

    bool KukaEkiIoInterface::eki_read_state(std::vector<bool>& ioStates, std::vector<int>& ioPins)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Declarations & Allocations
        ioStates.resize(numberOfIos_);
        ioPins.resize(numberOfIos_);
        static boost::array<char, 2048> inBuffer;

        // Read socket buffer (with timeout) // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_->expires_from_now(Seconds(eki_read_state_timeout_));
        SystemErrorCode systemErrorCode = boost::asio::error::would_block;
        size_t len = 0;

        eki_server_socket_->async_receive(boost::asio::buffer(inBuffer), boost::bind(&KukaEkiIoInterface::eki_handle_receive, _1, _2, &systemErrorCode, &len));

        do
            ios_.run_one();
        while (systemErrorCode == boost::asio::error::would_block);

        // KUKAEKIIO_00001 // KUKAEKIIO_00002 // Log warning when errorcode is set and do not continue processing.
        if (systemErrorCode)
        {
            RCLCPP_WARN(logger, " communication error code: %s", systemErrorCode.message().c_str());
            return false;
        }

        // KUKAEKIIO_00003 // KUKAEKIIO_00004 // Log warning when packages with zero lenght are received and do not continue processing.
        if (len == 0)
        {
            RCLCPP_WARN(logger, " packet of len 0 received.");
            return false;
        }

        // KUKAEKIIO_00005 // Ensure null-terminated data buffer for parsing it as c-string.
        inBuffer[len] = '\0';

        // KUKAEKIIO_00006 // Materialize incoming c-string as XML DOM. 
        tinyxml2::XMLDocument xmlDocument;
        tinyxml2::XMLError xmlDocumentParseError = xmlDocument.Parse(inBuffer.data());

        // TODO // HAndle parse error.
        if (xmlDocumentParseError != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(logger, "Error parsing incoming XML telegram: %s", xmlDocument.ErrorStr());
            return false;
        }

        tinyxml2::XMLElement* robotState = xmlDocument.FirstChildElement("IOState");

        // KUKAEKIIO_00007 // KUKAEKIIO_00008 // Log warning when no "IOState" is child-element contained and do not continue processing. 
        if (!robotState)
        {
            RCLCPP_ERROR(logger, " no IOState-element found in XML.");
            return false;
        }
        
        // KUKAEKIIO_00009 // Log debug the XML DOM.
        tinyxml2::XMLPrinter printer;
        xmlDocument.Print(&printer);
        RCLCPP_DEBUG(logger, " received XML: %s", printer.CStr());
        
        // Transform data from XML DOM into an c-typed structure.
        // 
        for (int i = 0; i < numberOfIos_; i++)
        {
            tinyxml2::XMLElement* state = robotState->FirstChildElement(ioNames[i].c_str());
            if (!state)
            {
                RCLCPP_ERROR(logger, " no %s-element found in XML.", ioNames[i].c_str());
                return false;
            }
            
            int ioState = myCustomTemporaryDefaultValue;
            int ioPin = myCustomTemporaryDefaultValue;
            int ioMode = myCustomTemporaryDefaultValue;

            state->QueryIntAttribute("State", &ioState);
            state->QueryIntAttribute("Pin", &ioPin);
            state->QueryIntAttribute("Mode", &ioMode);

            if (ioState == myCustomTemporaryDefaultValue || ioPin == myCustomTemporaryDefaultValue || ioMode != __ekiModeRead_)
            {
                RCLCPP_ERROR(logger, " invalid %s-element found in XML.", ioNames[i].c_str());
                return false;
            }

            ioStates[i] = ioState;
            ioPins[i] = ioPin;
        }
        return true;
    }

    bool KukaEkiIoInterface::eki_write_command(const std::vector<int> &ioPins, const std::vector<bool> &targetIos)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // TODO: assert vectors' lengths
        // TODO: extend vector if length < n_io_ // pk // Doesn't make sense.
        if (ioPins.size() != numberOfIos_ || targetIos.size() != numberOfIos_)
        {
            RCLCPP_ERROR(logger, "Invalid command: size of ioPins, ioModes and targetIos must be equal to numberOfIos_=%d", numberOfIos_);
            return false;
        }

        tinyxml2::XMLDocument xmlCommand;
        tinyxml2::XMLElement* ioCommand = xmlCommand.NewElement("IOCommand");
        for (int i = 0; i < numberOfIos_; i++)
        {
            tinyxml2::XMLElement* ioElement = xmlCommand.NewElement(ioNames[i].c_str());
            ioCommand->InsertEndChild(ioElement);

            // pk // What was the purpose of this??
            // TiXmlText* empty_text = new TiXmlText("");
            // io_element->LinkEndChild(empty_text);

            ioElement->SetAttribute("Pin", ioPins[i]);
            ioElement->SetAttribute("Mode", __ekiModeWrite_);
            ioElement->SetAttribute("Value", (int)targetIos[i]);
        }
        xmlCommand.InsertEndChild(ioCommand);

        tinyxml2::XMLPrinter printer;
        xmlCommand.Print(&printer);
        RCLCPP_DEBUG(logger, " sending XML: %s", printer.CStr());

        eki_server_socket_->send_to(boost::asio::buffer(printer.CStr(), printer.CStrSize()), eki_server_endpoint_);
        return true;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> KukaEkiIoInterface::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> stateInterfaces;
        for (int i = 0; i < numberOfIos_; i++)
            stateInterfaces.push_back(std::make_shared<hardware_interface::StateInterface>(ioNames[i], "eki_io", &ioStates_[i]));
        return stateInterfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> KukaEkiIoInterface::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> commandInterfaces;
        for (int i = 0; i < numberOfIos_; i++)
            commandInterfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(ioNames[i], "eki_io", &ioCommands_[i]));
        return commandInterfaces;
    }

    hardware_interface::return_type KukaEkiIoInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        std::vector<int> ioPins;
        std::vector<bool> ioStates;

        ioPins.resize(numberOfIos_);
        ioStates.resize(numberOfIos_);

        if (!eki_read_state(ioStates, ioPins))
        {
            std::string msg = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            throw std::runtime_error(msg);
        }

        ioStates_ = ioStates;
        ioPins_ = ioPins;
    }

    hardware_interface::return_type KukaEkiIoInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        if (!eki_write_command(ioPins_, ioCommands_))
        {
            std::string msg = "Failed to write to robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            throw std::runtime_error(msg);
        }
        return hardware_interface::return_type::OK;
    }

    bool isValidIPv4(const std::string& ipString) {
        // IPv4 pattern with optional port
        // Matches: 192.168.1.1, 192.168.1.1:8080, 0.0.0.0, 255.255.255.255:65535
        static const std::regex ipv4Regex(
            "^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\."
            "(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"
            "(:(6553[0-5]|655[0-2][0-9]|65[0-4][0-9]{2}|6[0-4][0-9]{3}|[1-5][0-9]{4}|[1-9][0-9]{0,3}))?$"
        );
        
        return std::regex_match(ipString, ipv4Regex);
    }

    bool isInteger(const std::string& s) {
        try {
            std::stoi(s);
            return true;
        } catch (const std::invalid_argument& e) {
            return false;
        } catch (const std::out_of_range& e) {
            return true; // It's a number, but too large for int
        }
    }

}  // namespace kuka_eki_io_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kuka_eki_io_interface::KukaEkiIoInterface, hardware_interface::SystemInterface)