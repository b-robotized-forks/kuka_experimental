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
// Authors: Moritz Weisenböhler

#pragma once

#include <functional>

#include <eki_common/Component.h>

#include <eki_communication/core/EKInterface.h>
#include <eki_communication/core/CommandSequence.h>
#include <eki_communication/MetaCommand.h>
#include <eki_communication/RobotState.h>
#include <eki_communication/RobotMetaState.h>

namespace rbt
{
    class Robot : public Component
    {
    private:
        EKInterface interface_;
        EKInterface meta_interface_;

        int reconnect_delay_ = 1000;
        int loop_delay_ = 20;
        bool interface_used_ = false;
        bool meta_interface_used_ = false;

        RobotState state_;
        RobotMetaState meta_state_;
        CommandSequence active_sequence_;
        CommandSequence waiting_sequence_;

        float velocity_override_ = 1.f;
        bool commands_paused_ = false;

        void connect_to(rbt::EKInterface &interface, const std::string &host, int port);
        void spin();
        std::string collect_state_xml(EKInterface &interface, std::string &buffer, const std::string &tag);
        void update_state(std::string &xml_message, bool is_meta);
        void call_listener(RobotEvent event);

        ChronoEntry chrono_;

        bool chrono_running_tote_ = false;
        bool chrono_running_pick_ = false;
        bool chrono_running_between_ = false;
        bool chrono_running_drop_ = false;
        int remaining_runs = -1;

        void perform(const Command &command);
        void send_sequence();
        void send_meta(bool abort_commands = false);
        void check_time();

    public:
        Robot() : Component("rbt::Robot") {}
        ~Robot() {}

        bool is_connected();
        bool connect(const std::string &host, int port, int meta_port = 0);
        void connect_async(const std::string &host, int port, int meta_port = 0);
        void disconnect();
        void await_connection();

        void perform(const MoveCommand &move);
        void perform(const GripCommand &grip);
        void perform(const MoveCommand &move, const GripCommand &grip);

        bool auto_run = false;

        void pause_commands();
        void continue_commands();
        void abort_commands();
        void set_velocity(float value);

        bool run();
        bool is_active() { return !active_sequence_.is_finished(); }
        RobotState get_state() { return state_; }
        RobotMetaState get_meta_state() { return meta_state_; }
        float get_velocity_override() { return velocity_override_; }
        bool commands_paused() { return commands_paused_; }

        std::function<void(RobotEvent event, Robot *robot)> listener = nullptr;
        /*CommandSequence get_active_sequence() { return active_sequence_; }
    CommandSequence get_waiting_sequence() { return waiting_sequence_; }*/
    };
} // namespace rbt