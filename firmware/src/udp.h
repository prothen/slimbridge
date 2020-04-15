#include "mavesp.h"

#pragma once

namespace mavesp{

class UDP : public CommLink {

#define UDP_BUFFER_SIZE     1024
#define UDP_TIMEOUT_MS      1000
#define HOST_PORT           14550
#define CLIENT_PORT         14555
#define RESET_IP_TIMEOUT_MS 500

public:
    void    link                    (CommLink* forward_to);
    void    forward_messages        ();
    int     send                    (mavlink_message_t* message);
    int     send                    (mavlink_message_t* message, int count);

private:
    bool    _is_mavlink_message  ();
    void    _send_single_udp_message(mavlink_message_t* message);

    WiFiUDP                     udp_driver_;
    IPAddress                   dst_ip_{0};
    CommLink*                   uart_interface_;

    mavlink_message_t           message_;

};

}
