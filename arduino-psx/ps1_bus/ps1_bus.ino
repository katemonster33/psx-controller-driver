
#define DATA 2
#define COMMAND 3
#define ACK 4
#define CLOCK 5
#define PLAYER_1 6
#define PLAYER_2 7

#define CLOCK_SPEED 4

#define RUMBLEON   0x30
#define RUMBLEOFF  0x31
#define SET_ASCII  0x32
#define SET_BINARY 0x33

byte controllerIds[2];
uint16_t wordBuffer[2][15];
byte tempBufferCount = 0;
boolean printAscii = true;

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
    Serial.flush();
  }
  for(int i = 0; i < 2; i++)
  {
    read_psx_data(i);
  }
  delayMicroseconds(1500);
}

uint16_t bufferNew[20];
bool read_psx_data(int port)
{
  int playerPin = (port == 0 ? PLAYER_1 : PLAYER_2);
  digitalWrite(playerPin, LOW);
  delayMicroseconds(CLOCK_SPEED * 2);
  byte output = 0;
  tempBufferCount = 0;
  if(!put_byte(0x01, output, false)) 
  {
    if(controllerIds[port] != 0xFF)
    {
        notify_controllerLost(port);
    }
    controllerIds[port] = 0xFF;
    return false;
  }
  if(!put_byte(0x42, output, false)) return false;
  if(output != controllerIds[port])
  {
    notify_controllerFound(port);
    controllerIds[port] = output;
  }
  if(!put_byte(0x5A, output, false)) return false;
  uint16_t outputWord;
  int numWords = controllerIds[port] & 0x0F;
  for(int i = 0; i < numWords; i++)
  {
    bool lastWord = i == (numWords - 1);
    if(!read_word(outputWord, lastWord)) 
    {
      notify_controllerLost(port);
      return false;
    }
    bufferNew[tempBufferCount++] = outputWord;
  }
  for(int i = 0; i < numWords; i++)
  {
    if(i == 0) notify_buttons(port, wordBuffer[port][0], bufferNew[0]); 
    else notify_axes(port, i, wordBuffer[port][i], bufferNew[i]);
    wordBuffer[port][i] = bufferNew[i];
  }
  cleanup_psx();
  //printMessage(port);
  return true;
}

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

String buttons[] = 
{
  "L2",
  "R2",
  "L1",
  "R1",
  "Triangle",
  "Circle",
  "X",
  "Square",
  "Select",
  "R3",
  "L3",
  "Start",
  "Up",
  "Right",
  "Down",
  "Left",
};

String axes[] = 
{
  "Right Joy(X)",
  "Right Joy(Y)",
  "Left Joy(X)",
  "Left Joy(X)"
};

uint16_t memOld, memNew;
void notify_buttons(int idx, uint16_t dataOld, uint16_t dataNew)
{
  for(int i = 0; i < 16; i++)
  {
    memOld = dataOld & (1 << i);
    memNew = dataNew & (1 << i);
    if(memOld == memNew) continue;
    if(printAscii)
    {
      Serial.print("Port ");
      Serial.print(idx, DEC); 
      if(memOld > memNew) Serial.print(" released button ");
      else Serial.print(" pressed button ");
      Serial.println(buttons[i]);
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
      Serial.print(axes[axisIndex]);
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
    if(input & (1 << i))
    {
      digitalWrite(COMMAND, HIGH);
    }
    else
    {
      digitalWrite(COMMAND, LOW);
    }
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

