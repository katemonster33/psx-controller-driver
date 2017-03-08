#include <iostream>
#include <linux/uinput.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions       
#include <time.h>
#include <string.h>

#include "serial_helper.hpp"
using namespace std;


int serialport_read_until(int fd, unsigned char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}

// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        //perror("serialport_init: Unable to open port ");
        return -1;
    }
    
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        //perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        //perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

int spinread(int USB, unsigned char *buff, int timeout)
{
  int index = 0, bytesRemaining = 255;
  while(timeout > 0 && bytesRemaining > 0)
  {
    if(read(USB, buff + index, 1) == 1)
    {
      if(index == 0)
      {
	if(buff[0] == 0xFF)
	{
	  return 1;
	}
	else
	{
	  bytesRemaining = (buff[0] & 0x0F) << 1;
	}
      }
      else
      {
	bytesRemaining--;
      }
      index++;
    }
    usleep(1);
    timeout--;
  }
  return index;
}

LinuxUinput *SerialHelper::getPlayerById(int player)
{
  if(players[player] == nullptr)
  {
    bool recreate_p2 = false;
    if(player == 0 && players[1] != nullptr)
    {
      delete players[1];
      recreate_p2 = true;
    }
    cout << "PSX controller detected on slot " << (player + 1) << endl;
    struct input_id id;
    id.bustype = BUS_PARPORT;
    id.product = player + 1;
    id.vendor = 0x0001;
    id.version = 0x0100;
    players[player] = new LinuxUinput(LinuxUinput::kJoystickDevice, "PSX controller", id);
    for (int i = 0; i < 2; i++) players[player]->add_abs(psx_axes[i], -1, 1);
    
    for (int i = 2; i < 6; i++) players[player]->add_abs(psx_axes[i], 0, 255, 0, 28);
    
    for (int i = 0; i < 16; i++) if(psx_btn[i]) players[player]->add_key(psx_btn[i]);
    
    players[player]->finish();
    
    if(recreate_p2)
    {
      players[1] = getPlayerById(1);
    }
  }
  return players[player];
}

void SerialHelper::parseMessage(LinuxUinput *input, unsigned char *buffer, size_t bufferLen)
{
  PsxType type = (PsxType)(buffer[0] >> 4);
  if(type == PsxType::Analog || type == PsxType::AnalogRumble || type == PsxType::NegCon)
  {
    for (int i = 0; i < 4; i++)
    {
      input->send(EV_ABS, psx_axes[i + 2], buffer[i + 3]);
    }
  }
  input->send(EV_ABS, ABS_HAT0X, !(buffer[1] & 0x20) - !(buffer[1] & 0x80));
  input->send(EV_ABS, ABS_HAT0Y, !(buffer[1] & 0x40) - !(buffer[1] & 0x10));
  
  for (int i = 0; i < 16; i++)
  {
    if(!psx_btn[i]) continue;
    input->send(EV_KEY, psx_btn[i], ~buffer[(i / 8) + 1] & (1 << (i % 8)));
  }
  input->sync();
}

bool isPacketValid(unsigned char *buf)
{
  int padCount = 0;
  for(int i = 0; i < 30;)
  {
    if(buf[i] == 0) return false;
    if(buf[i] == 0xAA)
    {
      break;
    }
    else if(buf[i] == 0xFF)
    {
      padCount++;
      i++;
    }
    else
    {
      int bufCount = (buf[i] & 0x0F) << 1;
      i += bufCount + 1;
      padCount++;
    }
  }
  return padCount == 2;
}

int SerialHelper::ReadControllerInputThread()
{
  stopThread = false;
  int USB = serialport_init("/dev/ttyUSB1", B115200);

  /* Error Handling */
  if ( USB < 0 )
  {
    USB = serialport_init("/dev/ttyUSB0", B115200);
    if(USB < 0)
    {
      USB = serialport_init("/dev/ttyUSB2", B115200);
      if(USB < 0)
      {
	cout << "Error " << errno << " opening /dev/ttyUSB1: " << strerror (errno) << endl;
	return 1;
      }
    }
  }
  sleep(2);
  unsigned char buf [30];
  struct timespec sleepTime;
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 10000000;
  
  while(!stopThread)
  {
    unsigned char outByte = 0x01;
    int n = write (USB, &outByte, 1);
    if(n != 1)
    {
      cout << "No write.";
      close(USB);
      return 0;
    }
    memset(buf, 0, 30);
    int rc = serialport_read_until(USB, buf, 0xAA, 30, 15);
    if(rc != 0 || buf[0] == 0 || !isPacketValid(buf))
    {
      cout << "read fail." << std::endl;
      continue;
    }
    unsigned char *ptr = buf;
    for(int player = 0; player < 2; player++)
    {
      if(*ptr == 0xFF)
      {
	if(players[player]) 
	{
	  delete players[player];
	  players[player] = nullptr;
	  cout << "Lost PSX controller on slot " << (player + 1) << endl;
	}
	ptr++;
      }
      else
      {
	int count = (*ptr & 0x0F) << 1;
	LinuxUinput *struc = getPlayerById(player);
	parseMessage(struc, buf, count);
	ptr += count + 1;
      }
    }
    nanosleep(&sleepTime, nullptr);
  }
  close(USB);
  threadActive = false;
}

void SerialHelper::Start()
{
  if(threadActive) Stop();
  readControllerThread = thread(&SerialHelper::ReadControllerInputThread, this);
  threadActive = true;
}

void SerialHelper::Stop()
{
  if(threadActive)
  {
    stopThread = true;
    readControllerThread.join();
  }
  threadActive = false;
}