# slimbridge
Slim mavlink bridge exposing mavlink based serial interface to UDP world. This repository aims to provide a minimal code base for defining a bridge between serial and UDP interfaces. It tries to provide an alternative for the feature rich prominent prevalent solutions. Extensions and improvements are welcome in any form without adding features exceeding the forwarding application.

**Features:**
- Establish connection to compile time defined hotspot
- Forward UDP to UART interface
- Forward UART to UDP interface


## Usage
### Build & Flash using PlatformIO
Flash to the microcontroller by using the following commands under the directory `./firmware`

```bash
platformio run -e esp01_1m -t upload
```
_Note: Specific port `platformio run -e esp01_1m -t upload --upload-port /dev/ttyUSB0` and `platformio run -e esp01_1m -t clean`._

### Connect via mavros
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

**Multi-Vehicle Setup**:

In order to connect to multiple vehicles from the same workstation, each ESP needs to be configured with a separate `UDP_HOST_PORT`.

_Note: that the vehicle's slimbridge is written with the perspective of the `HOST` being a external workstation and the vehicle being the `CLIENT`.
This means that the `UDP_CLIENT_PORT` can be the same, as long as each vehicle has its unique IP in the network._

## Setup
For details about the hardware setup see the [wiring guide](./docs/WIRING_GUIDE.md).
1. Initialise the repository with
    - `git submodule update --init --recursive`
2. Install platformio with
    - `pip install platformio`
3. (optional) install visual studio code extension [here](https://platformio.org/platformio-ide) (in VSC under extensions `PlatformIO IDE`)
4. (optional) install `esptool` to flash binaries
    - `cd resources/esptool && python setup.py install`

## Contribution
Any contribution is welcome.
If you find missing instructions or something did not work as expected please create an issue and let me know.

## License
See the `LICENSE` file for details of the available open source licensing.
