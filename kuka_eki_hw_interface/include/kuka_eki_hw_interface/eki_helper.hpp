#ifndef EKI_HELPER_HPP
#define EKI_HELPER_HPP

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <tinyxml.h>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>

class EkiHelper {
public:
    static void check_read_state_deadline(
        boost::asio::deadline_timer &deadline,
        std::shared_ptr<boost::asio::ip::udp::socket> eki_server_socket,
        boost::asio::io_service &ios)
    {
        if (deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
            eki_server_socket->cancel();
            deadline.expires_at(boost::posix_time::pos_infin);
        }

        deadline.async_wait(boost::bind(
            &EkiHelper::check_read_state_deadline,
            boost::ref(deadline), eki_server_socket, boost::ref(ios)));
    }

    static void eki_handle_receive(const boost::system::error_code& ec, size_t length,
                               boost::system::error_code* out_ec, size_t* out_length)
    {
        *out_ec = ec;
        *out_length = length;
    }

    static bool eki_read_state(std::vector<double> &joint_position,
                           std::vector<double> &joint_velocity,
                           std::vector<double> &joint_effort,
                           int &cmd_buff_len,
                           boost::asio::deadline_timer &deadline,
                           std::shared_ptr<boost::asio::ip::udp::socket> eki_server_socket,
                           boost::asio::io_service &ios,
                           int eki_read_state_timeout,
                           const std::vector<double> &hw_states)
    {
        static boost::array<char, 2048> in_buffer;
        deadline.expires_from_now(boost::posix_time::seconds(eki_read_state_timeout));
        boost::system::error_code ec = boost::asio::error::would_block;
        size_t len = 0;

        eki_server_socket->async_receive(boost::asio::buffer(in_buffer),
            boost::bind(&eki_handle_receive, _1, _2, &ec, &len));
        do ios.run_one();
        while (ec == boost::asio::error::would_block);

        if (ec) 
        {
            RCLCPP_WARN(rclcpp::get_logger("KukaEkiHardwareInterface"), " communication error code: %s", ec.message().c_str());
            return false;
        }

        // Update joint positions from XML packet (if received)
        if (len == 0) 
        {
            RCLCPP_WARN(rclcpp::get_logger("KukaEkiHardwareInterface"), " packet of len 0 received");
            return false;
        }

        // Parse XML
        TiXmlDocument xml_in;
        in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
        xml_in.Parse(in_buffer.data());
        TiXmlElement* robot_state = xml_in.FirstChildElement("RobotState");
        if (!robot_state) 
            return false;
        TiXmlElement* pos = robot_state->FirstChildElement("Pos");
        TiXmlElement* vel = robot_state->FirstChildElement("Vel");
        TiXmlElement* eff = robot_state->FirstChildElement("Eff");
        TiXmlElement* robot_command = robot_state->FirstChildElement("RobotCommand");
        if (!pos || !vel || !eff || !robot_command) 
        {
            RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "parsing failed, one of pos, vel eff or RobotCommand missing");
            return false;
        }

        // Extract axis positions
        double joint_pos;  // [deg]
        double joint_vel;  // [%max]
        double joint_eff;  // [Nm]
        char axis_name[] = "A1";
        for (long unsigned int i = 0; i < hw_states.size(); ++i) 
        {
            pos->Attribute(axis_name, &joint_pos);
            joint_position[i] = angles::from_degrees(joint_pos);  // convert deg to rad
            vel->Attribute(axis_name, &joint_vel);
            joint_velocity[i] = joint_vel;
            eff->Attribute(axis_name, &joint_eff);
            joint_effort[i] = joint_eff;
            axis_name[1]++;
        }

        // Extract number of command elements buffered on robot
        robot_command->Attribute("Size", &cmd_buff_len);
        return true;
    }

    // static bool write_command(const std::vector<double> &joint_position_command,
    //                           std::shared_ptr<boost::asio::ip::udp::socket> eki_server_socket,
    //                           const boost::asio::ip::udp::endpoint& eki_server_endpoint,
    //                           const std::vector<std::double>& hw_states)
    // {
    //     TiXmlDocument xml_out;
    //     TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
    //     TiXmlElement* pos = new TiXmlElement("Pos");
    //     TiXmlText* empty_text = new TiXmlText("");
    //     robot_command->LinkEndChild(pos);
    //     pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
    //     char axis_name[] = "A1";
    //     for (long unsigned int i = 0; i < hw_states.size(); ++i) {
    //         pos->SetAttribute(axis_name, std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
    //         axis_name[1]++;
    //     }
    //     xml_out.LinkEndChild(robot_command);
    //     TiXmlPrinter xml_printer;
    //     xml_printer.SetStreamPrinting();  // no linebreaks
    //     xml_out.Accept(&xml_printer);

    //     eki_server_socket->send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()), eki_server_endpoint);
    //     return true;
    // }
};

#endif // EKI_HELPER_HPP
