
#define DATA 2
#define COMMAND 3
#define ACK 4
#define CLOCK 5
#define PLAYER_1 6
#define PLAYER_2 7

#define CLOCK_SPEED 2

#define RUMBLEON   0x30
#define RUMBLEOFF  0x31
#define SET_ASCII       0x32
#define SET_BINARY      0x33

uint8_t tempBuffer[16][2];
uint8_t tempBufferCount = 0;
boolean printAscii = false;

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
  tempBuffer[0][0] = 0xFF;
  tempBuffer[0][1] = 0xFF;
  Serial.begin(115200);
}

void printMessage(int idx)
{
  if(tempBuffer[0] == 0xFF)
  {
    if(printAscii)
    {
      Serial.print(0xFF, HEX);
    }
    else
    {
      Serial.write(0xFF);
    }
  }
  else
  {
    if(printAscii)
    {
      for(int i = 0; i < tempBufferCount; i++)
      {
        Serial.print(tempBuffer[i], HEX); 
      }
    }
    else
    {
      Serial.write(tempBuffer, tempBufferCount);
    }
  }
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
  delayMicroseconds(200);
}

bool read_psx_data(int port)
{
  int playerPin = (port == 0 ? PLAYER_1 : PLAYER_2);
  digitalWrite(playerPin, LOW);
  delayMicroseconds(CLOCK_SPEED * 2);
  byte output = 0;
  tempBuffer[0] = 0xFF;
  tempBufferCount = 0;
  if(!put_byte(0x01, output, false, false)) return false;
  if(!put_byte(0x42, output, true, false)) return false;
  uint8_t numBytes = (output & 0x0F) << 1;
  if(!put_byte(0xFF, output, false, false)) return false;
  for(int i = 0; i < numBytes; i++)
  {
    bool lastByte = i == (numBytes - 1);
    if(!put_byte(0xFF, output, true, lastByte)) return false;
  }
  cleanup_psx();
  //printMessage(port);
  return true;
}

void cleanup_psx()
{
  digitalWrite(PLAYER_1, HIGH);
  digitalWrite(PLAYER_2, HIGH);
  digitalWrite(COMMAND, HIGH);
  digitalWrite(CLOCK, HIGH);
}

inline bool put_byte(byte input, byte& output)
{
  return put_byte(input, output, true, false);
}

bool put_byte(byte input, byte& output, bool writeOutput, bool lastByte)
{
  output = 0;
  for(int i = 0; i < 8; i++)
  {
    digitalWrite(CLOCK, LOW); 
    if(input & (1 << i))
    {
      digitalWrite(COMMAND, HIGH);
    }
    else
    {
      digitalWrite(COMMAND, LOW);
    }
    delayMicroseconds(CLOCK_SPEED);
    digitalWrite(CLOCK, HIGH);
    int by = digitalRead(DATA);
    if(by == HIGH)
    {
      output |= (1 << i);
    }
    delayMicroseconds(CLOCK_SPEED);
  }
  if(writeOutput)
  {
    tempBuffer[tempBufferCount++] = output;
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
      delayMicroseconds(CLOCK_SPEED);
      return true;
    }
    delayMicroseconds(1);
  }
  return false;
}

