#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#undef F
#include <standard/mavlink.h>

#pragma once

namespace mavesp{

class CommLink{
public:
    virtual void    forward_messages    ();
    virtual int     send                (mavlink_message_t* message);
    virtual int     send                (mavlink_message_t* message, int count);
public:
    mavlink_message_t       rx_msg_;
    mavlink_status_t        rx_status_;
    mavlink_status_t        mav_status_;


};

}
