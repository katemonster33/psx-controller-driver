#include <thread>
#include "linux_uinput.hpp"

static const short psx_axes[] = {
	ABS_HAT0X, ABS_HAT0Y, ABS_RX, ABS_RY, ABS_X, ABS_Y
};
static const short psx_btn[] = {
	BTN_TL2, BTN_TR2, BTN_TL, BTN_TR, BTN_X, BTN_A, BTN_B, BTN_Y,
	BTN_SELECT, BTN_THUMBR, BTN_THUMBL, BTN_START, 0, 0, 0, 0,
};

enum class PsxType
{
  Mouse = 1,
  NegCon = 2,
  Normal = 4,
  Analog = 5,
  AnalogRumble = 7
};

enum class PsxCommand
{
  Button = 0x01,
  Axis,
};

class SerialHelper
{
  bool threadActive;
  bool stopThread;
  std::thread readControllerThread;
  LinuxUinput **players;
  LinuxUinput *getPlayerById(int player);
  
  void parseMessage(LinuxUinput *input, unsigned char *buffer, size_t bufferLen);
  public:
    SerialHelper()
    { 
      players = new LinuxUinput*[2];
      players[0] = nullptr;
      players[1] = nullptr;
      threadActive = false; 
      stopThread = false; 
    }
    int ReadControllerInputThread();
    void Start();
    void Stop();
};