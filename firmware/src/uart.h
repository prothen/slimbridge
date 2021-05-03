#include "mavesp.h"
#include "config.h"

#pragma once

namespace mavesp{

class UART : public CommLink {


public:
    void    link                (CommLink* forward_to);
    void    forward_messages    ();
    int     send                (mavlink_message_t* message);
    int     send                (mavlink_message_t* message, int count);
private:
    bool    _is_mavlink_message  ();

    CommLink*               udp_interface_;

    mavlink_message_t       message_[UART_QUEUE_SIZE];
    int                     n_queue_;
    unsigned long           stamp_queue_;
};

}

