
#include "linux_uinput.hpp"
#include "serial_helper.hpp"

#define ARDUINO_DEV "/dev/ttyUSB1"

#define ARDUINO_DEV_1 "/dev/ttyUSB1"

LinuxUinput *player1 = 0;
LinuxUinput *player2 = 0;


int main(int argc, char **argv) {
  SerialHelper serialHelper;
  serialHelper.ReadControllerInputThread();
  return 0;
}
