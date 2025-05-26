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

#include <eki_communication/core/EKInterface.h>

#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <iostream>

int rbt::EKInterface::check_error(int value, std::string msg)
{
    if (value < 0)
    {
        std::cout << msg << std::endl;
        connected_ = false;
        //throw std::runtime_error(msg);
    }

    return value;
}

bool rbt::EKInterface::is_connected()
{
    if (connected_)
    {
        int error_code;
        socklen_t error_code_size = sizeof(error_code);
        getsockopt(socket_, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size);

        connected_ = error_code == 0;
    }

    return connected_;
}

int rbt::EKInterface::send(const std::string &xml)
{
    // std::cout<<"[EKInterface] Sending XML to robot ..."<<std::endl<<xml<<std::endl<<std::endl;
    return check_error(
        write(socket_, xml.c_str(), xml.size()),
        "ERROR writing to socket");
}

std::string rbt::EKInterface::receive()
{
    int count;
    ioctl(socket_, FIONREAD, &count);

    if (count > 0)
    {
        char buffer[receive_size];
        bzero(buffer, receive_size);

        check_error(
            read(socket_, buffer, receive_size - 1),
            "ERROR reading from socket");

        // std::cout << "[EKInterface] Received XML from robot ..." << std::endl << std::string(buffer) << std::endl << std::endl;
        return std::string(buffer);
    }

    return "";
}

bool rbt::EKInterface::connect_to(const std::string &host, int port, bool udp)
{
    std::cout << "[EKInterface] Trying to connect to " << host << ", port: " << port << ", udp: " << (udp ? "true" : "false") << std::endl;

    socket_ = socket(AF_INET, udp ? SOCK_DGRAM : SOCK_STREAM, 0);
    struct hostent *server = gethostbyname(host.c_str());

    struct sockaddr_in serv_addr;

    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);

    serv_addr.sin_port = htons(port);

    int code = check_error(
        connect(socket_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)),
        "ERROR connecting to socket");

    connected_ = code == 0;

    std::cout << "[EKInterface] " << (connected_ ? "Successfully connected to " : "Failed to connect to ") << host << ", port: " << port << std::endl;

    return connected_;
}

void rbt::EKInterface::disconnect()
{
    close(socket_);

    connected_ = false;

    std::cout << "[EKInterface] Closed socket " << socket_ << std::endl;
}