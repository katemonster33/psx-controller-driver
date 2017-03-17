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

int tryReadPacket(unsigned char *ptr)
{
  switch(*ptr)
  {
    case 0x01:
      
      return 3;
    default:
      return 1;
    case 0x02:
      return 4;
  }
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
	cout << "Error " << errno << " opening tty: " << strerror (errno) << endl;
	return 1;
      }
    }
  }
  sleep(2);
  const int bufSize = 255;
  unsigned char buf [bufSize];
  struct timespec sleepTime;
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 15000000;
  
  while(!stopThread)
  {
    unsigned char outByte = 0x01;
    memset(buf, 0, bufSize);
    int rc = read(USB, buf, bufSize);
    if(rc == -1)
    {
      cout << "Got read error." << std::endl;
    }
    else
    {
      unsigned char *ptr = buf, *lastptr = buf + rc;
      while(ptr < lastptr)
      {
	unsigned char mode = *(ptr++);
	switch(mode)
	{
	  case 0x01: // found controller 
	  case 0x02: // lost controller 
	  {
	    int player = *(ptr++);
	    if(mode == 0x01)
	    {
	      getPlayerById(player);
	    }
	    else
	    {
	      delete players[player];
	      players[player] = nullptr;
	      cout << "Lost PSX controller on slot " << (player + 1) << endl;
	    }
	  }
	  break;
	  case 0x03: // button
	  {
	    int player = *(ptr++);
	    int button = *(ptr++);
	    int val = *(ptr++);
	    if(button == 15 || button == 13)
	    {
	      if(val != 0) val = (button == 15 ? -1 : 1);
	      getPlayerById(player)->send(EV_ABS, ABS_HAT0X, val);
	    }
	    else if(button == 14 || button == 12)
	    {
	      if(val != 0) val = (button == 14 ? -1 : 1);
	      getPlayerById(player)->send(EV_ABS, ABS_HAT0Y, val);
	    }
	    else
	    {
	      getPlayerById(player)->send(EV_KEY, psx_btn[button], val);
	    }
	    getPlayerById(player)->sync();
	  }
	  break;
	  case 0x04: // axis
	  {
	    int player = *(ptr++);
	    int axis = *(ptr++);
	    int val = *(ptr++);
	    getPlayerById(player)->send(EV_ABS, psx_axes[axis + 2], val);
	    getPlayerById(player)->sync();
	  }
	  break;
	  default:
	    break;
	}
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