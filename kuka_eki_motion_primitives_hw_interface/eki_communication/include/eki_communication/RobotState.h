#pragma once

#include <eki_communication/xml/XmlReader.h>

#include <eki_communication/core/Types.h>

namespace rbt
{
    class RobotState
    {
    private:
        static int max_id_;
        int id_ = 0;

    public:
        RobotState() : id_(++max_id_) {}
        ~RobotState() {}

        int id() const { return id_; }

        int command_id = 0;
        int command_step = 0;
        int command_max_step = 0;

        PoseJoints position_joints;
        PoseCartesian position_cartesian;
        PoseJoints velocity;
        PoseJoints torque;

        float gripper_jaw_position = 0.f;
        float gripper_jaw_status = 0.f;
        float gripper_vacuum_suction = 0.f;
        float gripper_vacuum_force1 = 0.f;
        float gripper_vacuum_force2 = 0.f;
        float gripper_vacuum_cylinder = 0.f;

        int info_code = 0;
        std::string info_message;

        int error_code = 0;
        std::string error_message;

        void from_xml(XmlReader &reader);
    };
} // namespace rbt