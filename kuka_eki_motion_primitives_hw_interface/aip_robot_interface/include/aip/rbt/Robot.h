#pragma once

#include <functional>

#include <aip/common/Component.h>

#include <aip/rbt/core/EKInterface.h>
#include <aip/rbt/core/CommandSequence.h>
#include <aip/rbt/MetaCommand.h>
#include <aip/rbt/RobotState.h>
#include <aip/rbt/RobotMetaState.h>

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