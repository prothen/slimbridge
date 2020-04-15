#include "uart.h"


void mavesp::UART::link(mavesp::CommLink* udp_interface){
    udp_interface_ = udp_interface;
    Serial.begin(UART_BAUDRATE);
    Serial.setRxBufferSize(UART_BUFFER_SIZE);
}


void mavesp::UART::forward_messages(){
    if(n_queue_ < QUEUE_SIZE) {
        if(_is_mavlink_message()) {
            n_queue_++;
        }
    }
    // Send messages if queue is full or if some messages have been kept longer than QUEUE_TIMEOUT
    if(n_queue_ && (n_queue_ >= QUEUE_SEND_THRESHOLD || (millis() - stamp_queue_) > QUEUE_TIMEOUT_MS)) {
        int n_sent_messages = udp_interface_->send(message_, n_queue_);
        // Reset message queue for the amount of n_sent_messages messages
        if(n_sent_messages == n_queue_) {
            memset(message_, 0, sizeof(message_));
            n_queue_ = 0;
            stamp_queue_  = millis();
        } else if(n_sent_messages) {
            int n_remaining_messages = n_queue_ - n_sent_messages;
            for(int i = 0; i < n_remaining_messages; i++) {
                memcpy(&message_[i], &message_[n_sent_messages + i], sizeof(mavlink_message_t));
            }
            n_queue_ = n_remaining_messages;
        }
    }
}


int mavesp::UART::send(mavlink_message_t* message, int n_messages) {
    for(int i = 0; i < n_messages; i++) {
        send(&message[i]);
    }
    return n_messages;
}


int mavesp::UART::send(mavlink_message_t* message) {
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
    Serial.write((uint8_t*)(void*)buf, len);
    return 1;
}

bool mavesp::UART::_is_mavlink_message(){
    bool msg_received = false;
    while(Serial.available()){
        int result = Serial.read();
        if (result >= 0){
            msg_received = mavlink_frame_char_buffer(&rx_msg_,
                                                     &rx_status_,
                                                     result,
                                                     &message_[n_queue_],
                                                     &mav_status_);
            if(msg_received) {
                break;
            }
        }
    }
    return msg_received;
}
