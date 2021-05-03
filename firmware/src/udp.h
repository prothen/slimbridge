#include "mavesp.h"
#include "config.h"

#pragma once

namespace mavesp{

class UDP : public CommLink {

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
