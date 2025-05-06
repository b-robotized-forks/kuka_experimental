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
//          Moritz Weisenböhler

#pragma once

#include <eki_common/default.h>

namespace rbt
{
    class EKInterface
    {
    private:
        int socket_;
        bool connected_ = false;

        int check_error(int value, std::string msg);

    public:
        EKInterface() {}
        ~EKInterface() {}

        int receive_size = 1024;

        bool is_connected();

        bool connect_to(const std::string &host, int port, bool udp = false);
        void disconnect();

        int send(const std::string &xml);
        std::string receive();
    };
} // namespace rbt
