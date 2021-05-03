# slimbridge
Slim mavlink bridge exposing mavlink based serial interface to UDP world. This repository aims to provide a minimal code base for defining a bridge between serial and UDP interfaces. It tries to provide an alternative for the feature rich prominent prevalent solutions. Extensions and improvements are welcome in any form without adding features exceeding the forwarding application.

**Features:**
- Establish connection to compile time defined hotspot
- Forward UDP to UART interface
- Forward UART to UDP interface


## Usage

### Flash and build

Use `mavros` with the connection url
```bash
roslaunch mavros px4.launch fcu_url:=udp://:UDP_HOST_PORT@192.168.3.14:UDP_CLIENT_PORT
```
_Note: You can find the parameters under `firmware/src/config.h`._

### Configuration
Adjust the following code segment excerpt in `main.cpp` under `firmware/src` to your liking.

```cpp
// Main.cpp line 14f
IPAddress IP(192, 168, 3, 14);
IPAddress GATEWAY(192, 168, 3, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS(1, 1, 1, 1);

WiFi.config(IP, GATEWAY, SUBNET, DNS, 0U);
WiFi.begin("yourssid", "yourpassword");
```

_Note: Additionally you can adjust the bridge `UART` and `UDP` configuration constants under `config.h`._

## Setup
- initialise the repository
    - `git submodule update --init --recursive`
- install platformio with `pip install platformio`
- (optional) install visual studio code extension [here](https://platformio.org/platformio-ide) (in VSC under extensions `PlatformIO IDE`)
- (optional) install libraries for flashing via `esptool`
    - install esptool to flash binaries
        - `cd resources/esptool && python setup.py install`

### Build & Flash using PlatformIO
The following commands are executed in `/firmware`:
- flash `platformio run -e esp01_1m -t upload`
    - specific port `platformio run -e esp01_1m -t upload --upload-port /dev/ttyUSB0`
- clean `platformio run -e esp01_1m -t clean`

### Flash Procedure
- flash procedure wiring [instructions](http://www.whatimade.today/esp8266-easiest-way-to-program-so-far/)
- official px4 [documentation](https://docs.px4.io/v1.9.0/en/telemetry/esp8266_wifi_module.html)


**ESP01 Pinout:**

![esp_pinout](./resources/figures/esp01_pinout.png)


**FTDI Wire Guide:**
- connect FTDI [cable](https://www.ftdichip.com/Products/Cables/USBTTLSerial.htm<Paste>)
    - black:GND, brown:CTS, red:VCC, orange:TXD, yellow:RXD, green:RTS

![ftdi](./resources/figures/ftdi.png)


**Breadboard Wiring Guide:**

Note:
- 3.3v (red/purple)
- gnd (brown - rightmost)
- reset btn (brown centered)
- flash btn (gray)
Modes:
- normal operation (btns released)
- reset (left btn pressed)
- flash (both pressed, left (brown) released first then right (gray))

Example breadboard setup with two buttons:
![breadboard](./resources/figures/breadboard.png)

And a more neat version produced by Robin Baran ([@RBinsonB](https://github.com/RBinsonB))
![breadboard_soldered](./resources/figures/board.jpg)
![breadboard_soldered_assembly](./resources/figures/assembly.jpg)

### Debug serial interface
- Connect ftdi cable with (vcc, gnd, ch_pd, tx(esp) -> rx(ftdi-yellow))
    - in terminal execute `sudo screen /dev/ttyUSB0 115200 8N1`


## Common Problems
- if device is busy use `lsof | grep /dev/ttyUSB0` (for `USB0`) and shutdown the occupying process
    - **Often QGroundControl is the culprit.**


## References
- Configured to connect to a fixed hotspot (see section _Usage_)
- esp8266 runs on `/dev/ttyS0` on pixracer configuration with 20000 B/s (baudrate of 921600)
    - see `px/boards/px4/fmuv4/init/rc.board_mavlink`
- inspired from [dogmaphobic](https://github.com/dogmaphobic/mavesp8266)'s work.
- depends on [esp8266wifi](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi) ([Documentation](https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html)).
    - e.g. [see IP class](https://github.com/esp8266/Arduino/blob/master/cores/esp8266/IPAddress.h)
    - e.g. [see WiFi header](https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/ETH.h)
    - e.g. [see UDP driver](https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiUdp.h)

**Preparation of PX4:**
- flashing `px4_fmu-v4`
	- install `arm-none-eabi-gcc` [here](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html#nuttx-based-hardware).
	- in this repository root execute `pip install -r requirements`
	- in px4 firmware folder `make px4_fmu-v4 upload`

**Connect to the Device:**
- connect via mavros ([installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation))
    - `roslaunch mavros px4.launch fcu_url:=udp://:14550@192.168.3.14:14555`
    - test it with `rostopic echo /mavros/rc/in` (shows rc commands send to the aerial vehicle)

