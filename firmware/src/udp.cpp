#include "udp.h"


void mavesp::UDP::link(mavesp::CommLink* uart_interface){
    uart_interface_ = uart_interface;
    udp_driver_.begin(UDP_CLIENT_PORT);
}


int mavesp::UDP::send(mavlink_message_t* message, int count) {
    if(!dst_ip_){return 0;}
    int sentCount = 0;
    udp_driver_.beginPacket(dst_ip_, UDP_HOST_PORT);
    for(int i = 0; i < count; i++) {
        char buf[300];
        unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message[i]);
        size_t sent = udp_driver_.write((uint8_t*)(void*)buf, len);
        if(sent != len) {
            break;
            udp_driver_.endPacket();
            udp_driver_.beginPacket(dst_ip_, UDP_HOST_PORT);
            udp_driver_.write((uint8_t*)(void*)&buf[sent], len - sent);
            udp_driver_.endPacket();
            return sentCount;
        }
        sentCount++;
    }
    udp_driver_.endPacket();
    return sentCount;
}


int mavesp::UDP::send(mavlink_message_t* message) {
    if(!dst_ip_){return 0;}
    _send_single_udp_message(message);
    return 1;
}


void mavesp::UDP::_send_single_udp_message(mavlink_message_t* msg){
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, msg);
    udp_driver_.beginPacket(dst_ip_, UDP_HOST_PORT);
    size_t sent = udp_driver_.write((uint8_t*)(void*)buf, len);
    udp_driver_.endPacket();
    if(sent != len) {
        delay(1);
        udp_driver_.beginPacket(dst_ip_, UDP_HOST_PORT);
        udp_driver_.write((uint8_t*)(void*)&buf[sent], len - sent);
        udp_driver_.endPacket();
    }
}


void mavesp::UDP::forward_messages(){
    if(_is_mavlink_message()) {
        uart_interface_->send(&message_);
        memset(&message_, 0, sizeof(message_));
    }
}


bool mavesp::UDP::_is_mavlink_message(){
    bool msg_received = false;
    int udp_count = udp_driver_.parsePacket();
    if(udp_count > 0){
        while(udp_count--) {
            int result = udp_driver_.read();
            if (result >= 0) {
                msg_received = mavlink_frame_char_buffer(&rx_msg_,
                                                         &rx_status_,
                                                         result,
                                                         &message_,
                                                         &mav_status_);
                if(msg_received) {
                    dst_ip_ = udp_driver_.remoteIP();
                    return msg_received;
                }
            }
        }
    }
    return msg_received;
}
