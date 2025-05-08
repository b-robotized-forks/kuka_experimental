// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Students of the Insitute for Robotics and Autonomous Systems (IRAS) 
//          - (Supervisor: Prof. Dr.-Ing. Christian Wurll), 
//          Moritz Weisenböhler, Mathias Fuhrer

#include <eki_communication/Robot.h>

#include <thread>
#include <iostream>

bool rbt::Robot::is_connected()
{
    return (!interface_used_ || interface_.is_connected()) && (!meta_interface_used_ || meta_interface_.is_connected());
}

bool rbt::Robot::connect(const std::string &host, int port, int meta_port)
{
    std::cout << "[Robot] Trying to connect to the host: [" << host << "], port: [" << port << "], meta_port: [" << meta_port << "]" << std::endl;
    interface_used_ = port > 0;
    meta_interface_used_ = meta_port > 0;

    if (interface_used_)
    {
        connect_to(interface_, host, port);
    }

    if (meta_interface_used_)
    {
        connect_to(meta_interface_, host, meta_port);
    }
    std::cout << "[Robot] is_connected: " << is_connected() << std::endl;
    return is_connected();
}

void rbt::Robot::connect_async(const std::string &host, int port, int meta_port)
{
    std::thread thread = std::thread([this, host, port, meta_port]() {
        this->connect(host, port, meta_port);

        spin();
    });

    thread.detach();
}

void rbt::Robot::disconnect()
{
    if (interface_used_)
    {
        std::cout << "[Robot] Disconnecting EKI Interface ..." << std::endl;
        interface_.disconnect();
    }

    if (meta_interface_used_)
    {
        std::cout << "[Robot] Disconnecting Meta EKI Interface ..." << std::endl;
        meta_interface_.disconnect();
    }
}

void rbt::Robot::await_connection()
{
    while (!is_connected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void rbt::Robot::perform(const MoveCommand &move)
{
    perform(Command(move));
}

void rbt::Robot::perform(const GripCommand &grip)
{
    perform(Command(grip));
}

void rbt::Robot::perform(const MoveCommand &move, const GripCommand &grip)
{
    perform(Command(move, grip));
}

void rbt::Robot::perform(const Command &command)
{
    waiting_sequence_.add(command);

    if (auto_run)
    {
        run();
    }
}

void rbt::Robot::pause_commands()
{
    if (!commands_paused_)
    {
        commands_paused_ = true;
        send_meta();
    }
}

void rbt::Robot::continue_commands()
{
    if (commands_paused_)
    {
        commands_paused_ = false;
        send_meta();
    }
}

void rbt::Robot::abort_commands()
{
    send_meta(true);
}

void rbt::Robot::reset_abort_commands()
{
    send_meta(false);
}

void rbt::Robot::set_velocity(float value)
{
    if (velocity_override_ != value)
    {
        velocity_override_ = value;

        if (!commands_paused_)
        {
            send_meta();
        }
    }
}

bool rbt::Robot::run()
{
    if (!is_active() && waiting_sequence_.size() > 0)
    {
        active_sequence_.clear();

        active_sequence_.add(waiting_sequence_);
        waiting_sequence_.clear();

        send_sequence();

        call_listener(RobotEvent::RUN);

        return true;
    }

    return false;
}

void rbt::Robot::send_sequence()
{
    XmlWriter writer;

    writer.line_break = "\n";

    writer.add_prolog();

    active_sequence_.to_xml(writer);

    int size = interface_.send(writer.get_string());
}

void rbt::Robot::send_meta(bool abort_commands)
{
    rbt::MetaCommand command;
    command.velocity_override = commands_paused_ ? 0.0f : velocity_override_;
    command.abort_commands = abort_commands;

    XmlWriter writer;
    writer.add_prolog();

    command.to_xml(writer);

    int size = meta_interface_.send(writer.get_string());
}

void rbt::Robot::connect_to(rbt::EKInterface &interface, const std::string &host, int port)
{
    while (!interface.is_connected())
    {
        if (interface.connect_to(host, port))
        {
            call_listener(RobotEvent::CONNECT);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_));
        }
    }
}

void rbt::Robot::spin()
{
    std::string buffer;
    std::string meta_buffer;

    while (is_connected())
    {
        if (interface_used_)
        {
            std::string xml = collect_state_xml(interface_, buffer, "RobotState");
            update_state(xml, false);
        }

        if (meta_interface_used_)
        {
            std::string meta_xml = collect_state_xml(meta_interface_, meta_buffer, "MetaState");
            update_state(meta_xml, true);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_delay_));
    }
}

std::string rbt::Robot::collect_state_xml(EKInterface &interface, std::string &buffer, const std::string &tag)
{
    buffer += interface.receive();

    int end_index = buffer.find("</" + tag + ">");

    if (end_index > -1)
    {
        end_index += 3 + tag.size();

        std::string xml_message = buffer.substr(0, end_index);
        buffer = buffer.substr(end_index);

        return xml_message;
    }

    return "";
}

void rbt::Robot::update_state(std::string &xml_message, bool is_meta)
{
    // std::cout << "[Robot update_state] " << (is_meta ? "Meta_XML message: " : "XML message: ") << xml_message << std::endl;
    if (xml_message.size() > 0)
    {
        XmlReader reader(xml_message);

        if (!reader.has_error())
        {
            if (is_meta)
            {
                meta_state_.from_xml(reader);
            }
            else
            {
                state_.from_xml(reader);

                active_sequence_.update(state_);

                check_time();
            }

            call_listener(RobotEvent::STATE);

            if (!is_meta && auto_run)
            {
                run();
            }
        }
        else if (reader.error_id() != tinyxml2::XMLError::XML_ERROR_EMPTY_DOCUMENT)
        {
            std::cout << reader.error() << std::endl;
            std::cout << "-> " << xml_message << std::endl;
        }
    }
}

void rbt::Robot::call_listener(RobotEvent event)
{
    if (listener != nullptr)
    {
        listener(event, this);
    }
}

void rbt::Robot::check_time()
{
    // TODO(mathias31415): Check/ Modifiy this?
    if (!chrono_running_tote_ && state_.position_joints.a1 < -5)
    {
        std::cout << "Chrono started" << std::endl;

        start_chrono("perform tote");
        start_chrono("perform all");

        chrono_running_tote_ = true;

        if (remaining_runs < 0)
        {
            remaining_runs = 3;

            start_chrono("perform 3 runs");
        }
    }

    if (!chrono_running_pick_ && state_.position_joints.a1 < -30)
    {
        log_chrono("perform tote");

        start_chrono("perform pick");

        chrono_running_pick_ = true;
    }

    if (chrono_running_pick_ && state_.position_joints.a1 > -30)
    {
        log_chrono("perform pick");

        chrono_running_pick_ = false;

        start_chrono("perform between");

        chrono_running_between_ = true;
    }

    if (chrono_running_between_ && state_.position_joints.a1 > 10)
    {
        log_chrono("perform between");

        chrono_running_between_ = false;

        start_chrono("perform drop");

        chrono_running_drop_ = true;
    }

    if (chrono_running_drop_ && state_.position_joints.a1 < 10)
    {
        log_chrono("perform drop");

        log_chrono("perform all");

        chrono_running_drop_ = false;

        chrono_running_tote_ = false;

        if (--remaining_runs == 0)
        {
            log_chrono("perform 3 runs");
            remaining_runs = -1;
        }
    }
}