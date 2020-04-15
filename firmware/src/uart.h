#include "mavesp.h"

#pragma once

namespace mavesp{

class UART : public CommLink {

#define QUEUE_SIZE                  2
#define QUEUE_SEND_THRESHOLD        5
#define QUEUE_TIMEOUT_MS            1

#define UART_BAUDRATE 921600
#define UART_BUFFER_SIZE 1024


public:
    void    link                (CommLink* forward_to);
    void    forward_messages    ();
    int     send                (mavlink_message_t* message);
    int     send                (mavlink_message_t* message, int count); 
private:
    bool    _is_mavlink_message  ();

    CommLink*               udp_interface_;

    mavlink_message_t       message_[QUEUE_SIZE];
    int                     n_queue_;
    unsigned long           stamp_queue_;
};

}

