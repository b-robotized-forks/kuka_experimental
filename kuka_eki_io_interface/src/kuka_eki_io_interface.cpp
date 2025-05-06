#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tinyxml2.h>

#include <kuka_eki_io_interface/kuka_eki_io_interface.h>

// TODOs
// Readability -> "ec" to "errorcode".

#define LOGGER_NAME "KukaEkiIoInterface"
const std::string ioNames[] = {"IO1", "IO2", "IO3", "IO4", "IO5", "IO6", "IO7", "IO8"};
const int myCustomTemporaryDefaultValue = -42069;
const int pinNumberOffset_ = 500;
const int __ekiModeWrite_ = 2;
const int __ekiModeRead_ = 1;

namespace kuka_eki_io_interface
{
    KukaEkiIoInterface::~KukaEkiIoInterface() 
    {
        on_deactivate(rclcpp_lifecycle::State());
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);
        RCLCPP_INFO(logger, "Starting ... please wait...");

        eki_server_address_ = info_.hardware_parameters["robot_ip"];
        eki_server_port_ = info_.hardware_parameters["eki_robot_port"];
        RCLCPP_INFO(logger, "using IP: %s", eki_server_address_.c_str());
        RCLCPP_INFO(logger, "using port: %s", eki_server_port_.c_str());

        if (eki_server_address_.empty() || eki_server_port_.empty())
        {
            RCLCPP_FATAL(logger, "robot_ip or eki_robot_port cannot be empty");
            return hardware_interface::CallbackReturn::ERROR;
        }

        deadline_.reset(new boost::asio::deadline_timer(ios_));
        eki_server_socket_.reset(new boost::asio::ip::udp::socket(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)));

        boost::asio::ip::udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});

        boost::array<char, 1> ini_buf = {0};
        eki_server_socket_->send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);  // initiate contact to start server

        // Start persistent actor to check for eki_read_state timeouts
        deadline_->expires_at(boost::posix_time::pos_infin);  // do nothing until a read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();

        std::vector<bool> ioStates;
        std::vector<int> ioPins;
        std::vector<int> ioModes;
        if (!eki_read_state(ioStates, ioPins, ioModes, 0))
        {
            std::string errorMessage = "Failed to read from robot EKI server within alloted time of " + std::to_string(eki_read_state_timeout_) + " seconds. Make sure eki_hw_interface is running on the robot controller and all configurations are correct.";
            RCLCPP_FATAL(logger, errorMessage.c_str());
            throw std::runtime_error(errorMessage);
        }
    }

    hardware_interface::CallbackReturn KukaEkiIoInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        info_ = info; // pk // Where does this come from? Probably inherited from SystemInterface.
        //status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::CallbackReturn::SUCCESS;
    }



    // PRIVATE // PRIVATE // PRIVATE // PRIVATE // PRIVATE // PRIVATE
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

    void KukaEkiIoInterface::eki_handle_receive(const boost::system::error_code& ec, size_t length, boost::system::error_code* out_ec, size_t* out_length)
    {
        *out_ec = ec;
        *out_length = length;
    }

    bool KukaEkiIoInterface::eki_read_state(std::vector<bool>& ioStates, std::vector<int>& ioPins, std::vector<int>& ioModes, int commandBufferLength)
    {
        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Declarations & Allocations
        ioStates.resize(numberOfIos_);
        ioPins.resize(numberOfIos_);
        ioModes.resize(numberOfIos_);
        static boost::array<char, 2048> inBuffer;

        // Read socket buffer (with timeout) // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_->expires_from_now(boost::posix_time::seconds(eki_read_state_timeout_));
        boost::system::error_code ec = boost::asio::error::would_block;
        size_t len = 0;

        eki_server_socket_->async_receive(boost::asio::buffer(inBuffer), boost::bind(&KukaEkiIoInterface::eki_handle_receive, _1, _2, &ec, &len));

        do
            ios_.run_one();
        while (ec == boost::asio::error::would_block);

        // KUKAEKIIO_00001 // KUKAEKIIO_00002 // Log warning when errorcode is set and do not continue processing.
        if (ec)
        {
            RCLCPP_WARN(logger, " communication error code: %s", ec.message().c_str());
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

            if (ioState == myCustomTemporaryDefaultValue || ioPin == myCustomTemporaryDefaultValue || ioMode == myCustomTemporaryDefaultValue)
            {
                RCLCPP_ERROR(logger, " invalid %s-element found in XML.", ioNames[i].c_str());
                return false;
            }

            ioStates[i] = ioState;
            ioPins[i] = ioPin;
            ioModes[i] = ioMode;
        }
        return true;
    }

    bool KukaEkiIOInterface::eki_write_command(const std::vector<int> &io_pins, const std::vector<int> &io_modes, const std::vector<bool> &target_ios)
    {
        // TODO: assert vectors' lengths
        // TODO: extend vector if length < n_io_
        TiXmlDocument xml_out;
        TiXmlElement* io_command = new TiXmlElement("IOCommand");
        char io_name[] = "IO1";
        for (int i = 0; i < n_io_; i++)
        {
            TiXmlElement* io_element = new TiXmlElement(io_name);
            TiXmlText* empty_text = new TiXmlText("");
            io_command->LinkEndChild(io_element);
            io_element->LinkEndChild(empty_text);

            io_element->SetAttribute("Pin", io_pins[i]);
            io_element->SetAttribute("Mode", io_modes[i]);
            io_element->SetAttribute("Value", (int)target_ios[i]);

            io_name[2]++;
        }
        xml_out.LinkEndChild(io_command);
        xml_out.Print();
        TiXmlPrinter xml_printer;
        xml_printer.SetStreamPrinting();  // no linebreaks
        xml_out.Accept(&xml_printer);

        eki_server_socket_.send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                              eki_server_endpoint_);
        return true;
    }
}  // namespace kuka_eki_io_interface