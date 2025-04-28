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
