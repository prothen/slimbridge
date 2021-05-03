#include "mavesp.h"
#include "udp.h"
#include "uart.h"

mavesp::UART            uart_link;
mavesp::UDP             udp_link;

void setup() {
    delay(1500);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);

    /*          ------------------------        */
    /*        USER SPECIFIC CONFIGURATION       */
    /*          ------------------------        */

    IPAddress IP(11,0,0,220);
    IPAddress GATEWAY(11, 0, 0, 1);
    IPAddress SUBNET(255, 255, 255, 0);
    IPAddress DNS(1, 1, 1, 1);

    WiFi.config(IP, GATEWAY, SUBNET, DNS, 0U);
    WiFi.begin("SML", "SML4admin.");

    /*          ------------------------        */

    while(WiFi.status() != WL_CONNECTED){delay(500);}

    WiFi.setAutoReconnect(true);
    WiFi.setOutputPower(20.5);

    udp_link.link(&uart_link);
    uart_link.link(&udp_link);
}


void loop() {
    udp_link.forward_messages();
    uart_link.forward_messages();
}
