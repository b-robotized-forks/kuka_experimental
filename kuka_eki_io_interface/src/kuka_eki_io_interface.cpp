#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tinyxml2.h>
#include <regex>

#include <kuka_eki_io_interface/kuka_eki_io_interface.hpp>

using namespace boost::placeholders;

namespace kuka_eki_io_interface
{
    const std::string XML_READ_EXAMPLE = "<IoState StateId=\"697\"><In0 Key=\"500\" Value=\"1\"></In0><In1 Key=\"501\" Value=\"0\"></In1><In2 Key=\"502\" Value=\"1\"></In2><In3 Key=\"503\" Value=\"0\"></In3><In4 Key=\"504\" Value=\"0\"></In4><In5 Key=\"505\" Value=\"0\"></In5><In6 Key=\"506\" Value=\"0\"></In6><In7 Key=\"507\" Value=\"0\"></In7 RequestId=\"11\"><Out0 Key=\"500\" Value=\"0\"></Out0><Out1 Key=\"501\" Value=\"0\"></Out1><Out2 Key=\"502\" Value=\"0\"></Out2><Out3 Key=\"503\" Value=\"1\"></Out3><Out4 Key=\"504\" Value=\"0\"></Out4><Out5 Key=\"505\" Value=\"0\"></Out5><Out6 Key=\"506\" Value=\"0\"></Out6><Out7 Key=\"507\" Value=\"0\"></Out7></IoState>";
    const std::string XML_WRITE_EXAMPLE = "<IoRequest RequestId=\"11\"><Out0 Key=\"500\" Value=\"False\" /><Out1 Key=\"501\" Value=\"False\" /><Out2 Key=\"502\" Value=\"False\" /><Out3 Key=\"503\" Value=\"True\" /><Out4 Key=\"504\" Value=\"False\" /><Out5 Key=\"505\" Value=\"False\" /><Out6 Key=\"506\" Value=\"False\" /><Out7 Key=\"507\" Value=\"False\" /><In0 Key=\"500\" /><In1 Key=\"501\" /><In2 Key=\"502\" /><In3 Key=\"503\" /><In4 Key=\"504\" /><In5 Key=\"505\" /><In6 Key=\"506\" /><In7 Key=\"507\" /></IoRequest>";
    // pk // Deconstructor most likely not required anymore since this class is not run as a separate node anymore and will be handled by the resource manager.
    // KukaEkiIoInterface::~KukaEkiIoInterface() 
    // {
    //     RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Destructor called. Cleaning up ...");
    //     on_deactivate(rclcpp_lifecycle::State());
    // }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_deactivate() called");

        // Stop the deadline timer
        deadline_->cancel();

        // Close the socket
        eki_server_socket_->close();

        RCLCPP_INFO(logger, "KUKA EKI IO interface deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "on_init() called");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        info_ = info; // pk // Is this even necessary?

        if (info_.gpios.size() > __maxIoNumber)
        {
            RCLCPP_FATAL(logger, "KUKA EKI IO interface only supports a maximum of %d GPIOs.", __maxIoNumber);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger, "KUKA EKI IO interface initialized with %d GPIOs.", info_.gpios.size());
        
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

            if (!isInteger(gpio.parameters.at("pin")))
            {
                RCLCPP_FATAL(logger, "KUKA EKI IO interface pin parameter must be an integer.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            // pk // Map KUKA EKI GPIO pin number to the command and state interface names
            int pinNumber = std::stoi(gpio.parameters.at("pin"));
            gpioInfos_[pinNumber] = {
                gpio.name,
                gpio.command_interfaces[0].name,
                gpio.state_interfaces[0].name,
                pinNumber
            };

            RCLCPP_DEBUG(logger, "GPIO %d: %s", pinNumber, gpio.name.c_str());
            RCLCPP_DEBUG(logger, "Command interface: %s", gpio.command_interfaces[0].name.c_str());
            RCLCPP_DEBUG(logger, "State interface: %s", gpio.state_interfaces[0].name.c_str());
            RCLCPP_DEBUG(logger, "Pin: %d", pinNumber);
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

        std::vector<int> inKeys;
        std::vector<bool> inValues;
        std::vector<int> outKeys;
        std::vector<bool> outValues;

        inKeys.resize(numberOfIos_);
        inValues.resize(numberOfIos_);
        outKeys.resize(numberOfIos_);
        outValues.resize(numberOfIos_);
        
        if (!eki_read_state(inKeys, inValues, outKeys, outValues))
        {
            std::string errorMessage = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " ms. Make sure eki_io_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_WARN(logger, errorMessage.c_str());
            //throw std::runtime_error(errorMessage); // pk // Comment back in
        }
        setInternalStates(inKeys, inValues);

        RCLCPP_INFO(logger, "KUKA EKI IO interface activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // pk // No idea what this function does.
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

    // void KukaEkiIoInterface::eki_handle_receive(const SystemErrorCode& systemErrorCode, size_t length, SystemErrorCode* out_ec, size_t* out_length)
    // {
    //     *out_ec = systemErrorCode;
    //     *out_length = length;
    // }

    bool KukaEkiIoInterface::eki_read_state(std::vector<int>& inKeys, std::vector<bool>& inValues, std::vector<int>& outKeys, std::vector<bool>& outValues)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Declarations & Allocations
        inValues.resize(numberOfIos_);
        inKeys.resize(numberOfIos_);
        outKeys.resize(numberOfIos_);
        outValues.resize(numberOfIos_);
        static boost::array<char, 2048> inBuffer;

        // Read socket buffer (with timeout) // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_->expires_from_now(Milliseconds(eki_read_state_timeout_));
        SystemErrorCode systemErrorCode = boost::asio::error::would_block;
        size_t receivedMessageLength = 0;

        //eki_server_socket_->async_receive(boost::asio::buffer(inBuffer), boost::bind(&KukaEkiIoInterface::eki_handle_receive, _1, _2, &systemErrorCode, &len));
        // eki_server_socket_->async_receive(
        //     boost::asio::buffer(inBuffer),
        //     [&systemErrorCode, &receivedMessageLength](const boost::system::error_code& errorCode, std::size_t length) {
        //         systemErrorCode = errorCode;
        //         receivedMessageLength = length;
        //     }
        // );

        // do
        //     ios_.run_one();
        // while (systemErrorCode == boost::asio::error::would_block);

        // // KUKAEKIIO_00001 // KUKAEKIIO_00002 // Log warning when errorcode is set and do not continue processing.
        // if (systemErrorCode)
        // {
        //     RCLCPP_WARN(logger, "communication error code: %s", systemErrorCode.message().c_str());
        //     return false;
        // }

        // // KUKAEKIIO_00003 // KUKAEKIIO_00004 // Log warning when packages with zero lenght are received and do not continue processing.
        // if (receivedMessageLength == 0)
        // {
        //     RCLCPP_WARN(logger, "message of length 0 received.");
        //     return false;
        // }

        // KUKAEKIIO_00005 // Ensure null-terminated data buffer for parsing it as c-string.
        // pk // Don't need to manually null-terminate the buffer, since tinyxml2::XMLDocument::Parse() does that for us. (receivedMessageLength parameter)
        //inBuffer[receivedMessageLength] = '\0';

        // KUKAEKIIO_00006 // Materialize incoming c-string as XML DOM. 
        tinyxml2::XMLDocument xmlDocument;
        // tinyxml2::XMLError xmlDocumentParseError = xmlDocument.Parse(inBuffer.data(), receivedMessageLength);
        tinyxml2::XMLError xmlDocumentParseError = xmlDocument.Parse(XML_READ_EXAMPLE.c_str(), XML_READ_EXAMPLE.size());

        // TODO // HAndle parse error.
        if (xmlDocumentParseError != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(logger, "Error parsing incoming XML telegram: %s", xmlDocument.ErrorStr());
            return false;
        }

        // KUKAEKIIO_00009 // Log debug the XML DOM.
        tinyxml2::XMLPrinter printer;
        xmlDocument.Print(&printer);
        // RCLCPP_INFO(logger, "received XML: %s", printer.CStr()); // TODO // ReH // Set to debug later.

        tinyxml2::XMLElement* xmlIoState = xmlDocument.FirstChildElement("IoState");

        // KUKAEKIIO_00007 // KUKAEKIIO_00008 // Log warning when no "IOState" is child-element contained and do not continue processing. 
        if (!xmlIoState)
        {
            RCLCPP_ERROR(logger, "no IoState-element found in XML.");
            return false;
        }
        
        // Transform data from XML DOM into an c-typed structure.
        for (int i = 0; i < numberOfIos_; i++)
        {
            int key = 0;
            bool value = false;

            parseKeyAndValue(xmlIoState, getInElementNameByIndex(i), key, value);
            inKeys[i] = key;
            inValues[i] = value;

            parseKeyAndValue(xmlIoState, getOutElementNameByIndex(i), key, value);
            outKeys[i] = key;
            outValues[i] = value;            
        }

        // RCLCPP_DEBUG(logger, "read %d IOs from robot EKI server.", numberOfIos_);
        // for (int i = 0; i < numberOfIos_; i++)
        // {
        //     RCLCPP_INFO(logger, " %s: [ %d, %d ]", getInElementNameByIndex(i).c_str(), inKeys[i], (int)inValues[i]);
        //     RCLCPP_INFO(logger, " %s: [ %d, %d ]", getOutElementNameByIndex(i).c_str(), outKeys[i], (int)outValues[i]);
        // }
        return true;
    }

    bool KukaEkiIoInterface::parseKeyAndValue(tinyxml2::XMLElement* xmlIoState, const std::string& xmlChildElementName, int& key, bool& value)
    {
        tinyxml2::XMLElement* xmlElement = xmlIoState->FirstChildElement(xmlChildElementName.c_str());
        if (!xmlElement)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "no %s-element found in XML.", xmlChildElementName.c_str());
            return false;
        }

        if (xmlElement->QueryIntAttribute("Key", &key) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to query 'Key' attribute or attribute is not an int for %s.", xmlElement->Name());
            return false;
        }
        if (xmlElement->QueryBoolAttribute("Value", &value) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to query 'Value' attribute or attribute is not an int for %s.", xmlElement->Name());
            return false;
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
        auto ioCommand = xmlCommand.NewElement("IOCommand");
        for (int i = 0; i < numberOfIos_; i++)
        {
            tinyxml2::XMLElement* ioElement = xmlCommand.NewElement(getInElementNameByIndex(i).c_str());
            ioCommand->InsertEndChild(ioElement);

            // pk // What was the purpose of this??
            // TiXmlText* empty_text = new TiXmlText("");
            // io_element->LinkEndChild(empty_text);

            ioElement->SetAttribute("Pin", ioPins[i]);
            ioElement->SetAttribute("Mode", __ekiModeWrite);
            ioElement->SetAttribute("Value", (int)targetIos[i]);
        }
        xmlCommand.InsertEndChild(ioCommand);

        tinyxml2::XMLPrinter printer;
        xmlCommand.Print(&printer);
        
        eki_server_socket_->send_to(boost::asio::buffer(printer.CStr(), printer.CStrSize()), eki_server_endpoint_);
        
        RCLCPP_DEBUG(logger, " sending XML: %s", printer.CStr());
        return true;
    }

    hardware_interface::return_type KukaEkiIoInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        std::vector<int> inKeys;
        std::vector<bool> inValues;
        std::vector<int> outKeys;
        std::vector<bool> outValues;

        if (!eki_read_state(inKeys, inValues, outKeys, outValues))
        {
            std::string msg = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            //return hardware_interface::return_type::ERROR;
        }
        if (!setInternalStates(inKeys, inValues))
        {
            std::string msg = "Failed to set state values from robot EKI server.";
            RCLCPP_ERROR(logger, msg.c_str());
            //return hardware_interface::return_type::ERROR;
        }

        RCLCPP_DEBUG(logger, "read %d IOs from robot EKI server.", numberOfIos_);
        for (int i = 0; i < numberOfIos_; i++)
            RCLCPP_DEBUG(logger, "Pin[%s]: %d", inKeys[i], inValues[i]);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiIoInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        auto ioCommands = std::vector<bool>();
        auto ioPins = std::vector<int>();
        if (!getInternalCommands(ioPins, ioCommands))
        {
            std::string msg = "Failed to get command values from robot EKI server.";
            RCLCPP_ERROR(logger, msg.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if (!eki_write_command(ioPins, ioCommands))
        {
            std::string msg = "Failed to write to robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(logger, msg.c_str());
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }

    bool KukaEkiIoInterface::setInternalStates(const std::vector<int>& ioPins, const std::vector<bool>& targetIos)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        int i = 0;
        try
        {
            for (int pinNumber : ioPins)
            {
                auto gpioInfo = gpioInfos_.find(pinNumber);
                if (gpioInfo == gpioInfos_.end())
                {
                    RCLCPP_ERROR(logger, "Invalid pin number %d", pinNumber);
                    return false;
                }
                auto gpio = gpioInfo->second;
                const std::string interfaceName = gpio.GetStateInterfaceName();
                set_state<double>(interfaceName, targetIos[i] ? 1.0 : 0.0);
                RCLCPP_INFO(logger, "Set state of %s[pin=%d] to %d", interfaceName.c_str(), pinNumber, targetIos[i]);
                i++;
            }
            RCLCPP_INFO(logger, "-------------------------------------------------------");
        }
        catch (const std::runtime_error& e)
        {
            RCLCPP_ERROR(logger, "Failed to set state value: %s", e.what());
            return false;
        }

        return true;
    }

    bool KukaEkiIoInterface::getInternalCommands(std::vector<int>& ioPins, std::vector<bool>& ioStates)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        ioPins.reserve(numberOfIos_);
        ioStates.reserve(numberOfIos_);

        try
        {
            for (const auto& [pinNumber, gpio] : gpioInfos_)
            {
                const std::string interfaceName = gpio.GetCommandInterfaceName();
                ioPins.push_back(pinNumber);
                ioStates.push_back(get_command<bool>(interfaceName));
            }
        }
        catch (const std::runtime_error& e)
        {
            RCLCPP_ERROR(logger, "Failed to get command value: %s", e.what());
            return false;
        }

        return true;
    }

    std::string GpioPinInfo::GetCommandInterfaceName() const
    {
        return InterfaceName + "/" + CommandInterfaceName;
    }

    std::string GpioPinInfo::GetStateInterfaceName() const
    {
        return InterfaceName + "/" + StateInterfaceName;
    }


    // IPv4 pattern with optional port
    // Matches: 192.168.1.1, 192.168.1.1:8080, 0.0.0.0, 255.255.255.255:65535
    bool isValidIPv4(const std::string& ipString) {
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

    const std::string getInElementNameByIndex(int index)
    {
        static const std::string defaultIoName = "In";
        return defaultIoName + std::to_string(index);
    }

    const std::string getOutElementNameByIndex(int index)
    {
        static const std::string defaultIoName = "Out";
        return defaultIoName + std::to_string(index);
    }

}  // namespace kuka_eki_io_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kuka_eki_io_interface::KukaEkiIoInterface, hardware_interface::SystemInterface)