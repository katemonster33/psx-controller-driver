
#define DATA 2
#define COMMAND 3
#define ACK 4
#define CLOCK 5
#define PLAYER_1 6
#define PLAYER_2 7

#define CLOCK_SPEED 4

#define SET_ASCII         0x31
#define SET_BINARY        0x32
#define START_RUMBLE_P1   0x33
#define STOP_RUMBLE_P1    0x34
#define START_RUMBLE_P2   0x35
#define STOP_RUMBLE_P2    0x36

byte controllerIds[2];
uint16_t buttons[2];
uint16_t axisBuffer[2][15];
boolean enableRumble[2];

byte tempBufferCount = 0;
boolean printAscii = true;

byte readMode[] = { 0x01, 0x42, 0x00, 0x00, 0x00 };

byte enterConfigMode[] = { 0x01, 0x43, 0x00, 0x01, 0x00 };

byte setAnalogMode[] = { 0x01, 0x44, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

byte enableRumbleMode[] = { 0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF };

byte exitConfigMode[] = { 0x01, 0x43, 0x00, 0x00, 0x00 };

void setup() 
{
  // put your setup code here, to run once:
  pinMode(COMMAND, OUTPUT);
  digitalWrite(COMMAND, HIGH);
  pinMode(DATA, INPUT_PULLUP);
  pinMode(PLAYER_2, OUTPUT);
  digitalWrite(PLAYER_2, HIGH);
  pinMode(PLAYER_1, OUTPUT);
  digitalWrite(PLAYER_1, HIGH);
  pinMode(CLOCK, OUTPUT);
  digitalWrite(CLOCK, HIGH);
  pinMode(ACK, INPUT_PULLUP);
  //Serial.begin(1000000);
  controllerIds[0] = 0xFF;
  controllerIds[1] = 0xFF;
  enableRumble[0] = false;
  enableRumble[1] = false;
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available() > 0)
  {
    char command = Serial.read();
    if(command == SET_ASCII)
    {
      printAscii = true;
    }
    else if(command == SET_BINARY)
    {
      printAscii = false;
    }
    else if(command == START_RUMBLE_P1)
    {
      enableRumble[0] = true;
    }
    else if(command == STOP_RUMBLE_P1)
    {
      enableRumble[0] = false;
    }
    else if(command == START_RUMBLE_P2)
    {
      enableRumble[1] = true;
    }
    else if(command == STOP_RUMBLE_P2)
    {
      enableRumble[1] = false;
    }
    Serial.flush();
  }
  for(int i = 0; i < 1; i++)
  {
    byte oldId = controllerIds[i];
    if(enableRumble[i])
    {
      Serial.println("RUMBLING");
      readMode[3] = 0xFF;
      readMode[4] = 0xFF;
    }
    else
    {
      readMode[3] = 0x00;      
      readMode[4] = 0x00;
    }
    boolean success = send_string(i, readMode, sizeof(readMode));
    if(success && oldId == 0xFF)
    {
      notify_controllerFound(i);
      setup_controller(i);
    }
    else if(!success && oldId != 0xFF)
    {
      controllerIds[i] = 0xFF;
      notify_controllerLost(i);
    }
  }
  delayMicroseconds(500);
}

void setup_controller(int port)
{
  delayMicroseconds(150);
  send_string(port, enterConfigMode, sizeof(enterConfigMode));
  delayMicroseconds(150);
  send_string(port, setAnalogMode, sizeof(setAnalogMode));
  delayMicroseconds(150);
  send_string(port, enableRumbleMode, sizeof(enableRumbleMode));
  delayMicroseconds(150);
  send_string(port, exitConfigMode, sizeof(exitConfigMode));
}

uint16_t bufferNew[20];

#define ControllerFound 1
#define ControllerLost 2
#define ButtonUpdate 3
#define AxisUpdate 4

void notify_controllerLost(int idx)
{
  if(printAscii)
  {
    Serial.print("Controller ");
    Serial.print(idx, DEC);
    Serial.println(" lost.");
  } 
  else
  {
    Serial.write(ControllerLost);
    Serial.write(idx);
  }
}

void notify_controllerFound(int idx)
{
  if(printAscii)
  {
    Serial.print("Controller ");
    Serial.print(idx, DEC);
    Serial.println(" found.");
  } 
  else
  {
    Serial.write(ControllerFound);
    Serial.write(idx);
  }
}

String buttonsDesc[] = { "L2", "R2", "L1", "R1", "Triangle", "Circle", "X", "Square", "Select", "R3", "L3", "Start", "Up", "Right", "Down", "Left", };

String axes[] = { "Right Joy(X)", "Right Joy(Y)", "Left Joy(X)", "Left Joy(X)", "5", "6", "7", "8", "9", "10", "11", "12" };

uint16_t memOld, memNew;
void notify_buttons(int idx, uint16_t dataNew)
{
  for(int i = 0; i < 16; i++)
  {
    memOld = buttons[idx] & (1 << i);
    memNew = dataNew & (1 << i);
    if(memOld == memNew) continue;
    if(printAscii)
    {
      Serial.print("Port ");
      Serial.print(idx, DEC); 
      if(memOld < memNew) Serial.print(" released button ");
      else Serial.print(" pressed button ");
      Serial.println(i, DEC);
    }
    else
    {
      Serial.write(ButtonUpdate);
      Serial.write(idx);
      Serial.write(i);
      Serial.write(memOld > memNew ? 1 : 0);
    }
  }
}

void notify_axes(int idx, int bufIndex, uint16_t dataOld, uint16_t dataNew)
{
  int axisIndex = (bufIndex - 1) * 2;
  if(dataOld == dataNew) return;
  notify_axis(idx, axisIndex, (byte)(dataOld >> 8), (byte)(dataNew >> 8));
  notify_axis(idx, axisIndex + 1, (byte)(dataOld), (byte)(dataNew));
}

void notify_axis(int idx, int axisIndex, byte dataOld, byte dataNew)
{
  if(dataOld != dataNew)
  {
    if(printAscii)
    {
      Serial.print("Port ");
      Serial.print(idx, DEC); 
      Serial.print(" set axis ");
      Serial.print(axisIndex);
      Serial.print(":");
      Serial.println(dataNew);
    }
    else
    {
      Serial.write(AxisUpdate);
      Serial.write(idx);
      Serial.write(axisIndex);
      Serial.write(dataNew);
    }
  }
}

void cleanup_psx()
{
  digitalWrite(PLAYER_1, HIGH);
  digitalWrite(PLAYER_2, HIGH);
  digitalWrite(COMMAND, HIGH);
  digitalWrite(CLOCK, HIGH);
}

boolean send_string(int port, byte string[], int len)
{
  digitalWrite(port == 0 ? PLAYER_1 : PLAYER_2, LOW);
  delayMicroseconds(CLOCK_SPEED * 2);
  byte tmp = 0;
  int id = 0;
  boolean isDataMode = (string[1] == 0x42 || string[1] == 0x43);
  uint16_t buttonsTmp = 0;
  for(int i = 0; i < len; i++)
  {
    bool isLastByte = (i == (len - 1));
    if(isDataMode) isLastByte = isLastByte && id == 0x41;
    if(!put_byte(string[i], tmp, isLastByte))
    {
      return false;
    }
    if(isDataMode)
    {
      if(i == 1) id = tmp;
      else if(i == 3) buttonsTmp |= (tmp << 8);
      else if(i == 4) buttonsTmp |= tmp;
    }
  }
  if(isDataMode && id != 0 && id != 0xFF)
  {
    int buflen = 0;
    int numWords = (id & 0x0F) - 1;
    for(int i = 0; i < numWords; i++)
    {
      if(!read_word(bufferNew[buflen++], i == (numWords - 1))) 
      {
        return false;
      }
    }
    controllerIds[port] = id;
    notify_buttons(port, buttonsTmp);
    buttons[port] = buttonsTmp;
    for(int i = 0; i < numWords; i++)
    {
      notify_axes(port, i, axisBuffer[port][i], bufferNew[i]);
      axisBuffer[port][i] = bufferNew[i];
    }
  }
  cleanup_psx();
  return true;
}

bool read_word(uint16_t &output, bool lastWord)
{
  byte by1, by2;
  if(!put_byte(0x5A, by1, false)) return false;
  if(!put_byte(0x5A, by2, lastWord)) return false;
  output = by1 << 8 | by2;
  return true;
}

bool put_byte(byte input, byte& output, bool lastByte)
{
  output = 0;
  uint8_t old_sreg = SREG;        // *** KJE *** save away the current state of interrupts
  for(int i = 0; i < 8; i++)
  {
    cli();
    digitalWrite(CLOCK, LOW); 
    digitalWrite(COMMAND, (input & (1 << i)) ? HIGH : LOW);
    SREG = old_sreg;
    delayMicroseconds(CLOCK_SPEED);
    cli();
    digitalWrite(CLOCK, HIGH);
    int by = digitalRead(DATA);
    if(by == HIGH)
    {
      output |= (1 << i);
    }
    SREG = old_sreg;
    delayMicroseconds(CLOCK_SPEED);
  }
  if(!lastByte)
  {
    if(!wait_ack()) { cleanup_psx(); return false; }
  }
  return true;
}

bool wait_ack()
{
  for(int i = 0; i < 50; i++)
  {
    if(digitalRead(ACK) == LOW)
    {
      delayMicroseconds(CLOCK_SPEED * 2);
      return true;
    }
    delayMicroseconds(1);
  }
  return false;
}

