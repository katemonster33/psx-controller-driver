#define DATA 2
#define COMMAND 3
#define ACK 4
#define CLOCK 5
#define PLAYER_1 6
#define PLAYER_2 7

#define CLOCK_SPEED 4 // = 250 khz

#define ControllerFound 1
#define ControllerLost 2
#define ButtonUpdate 3
#define AxisUpdate 4

#define TOGGLE_LOGGING    0x31
#define START_RUMBLE_P1   0x32
#define STOP_RUMBLE_P1    0x33
#define START_RUMBLE_P2   0x34
#define STOP_RUMBLE_P2    0x35

#define NUM_CONTROLLERS 2

typedef struct Controller
{
  byte ID;
  byte Index;
  byte Pin;
  uint16_t Buttons;
  byte RumbleActive;
};
Controller controllers[NUM_CONTROLLERS];

boolean loggingActive = true, msgLoggingActive = false;

byte readMode[] = { 0x01, 0x42, 0x00, 0x00, 0x00 };
byte setAnalogMode[] = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
byte enterConfigMode[] = { 0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte enableRumbleMode[] = { 0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF };
byte exitConfigMode[] = { 0x01, 0x43, 0x00, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A };

void setup()
{
  // put your setup code here, to run once:
  pinMode(COMMAND, OUTPUT);
  pinMode(DATA, INPUT_PULLUP);
  pinMode(CLOCK, OUTPUT);
  pinMode(ACK, INPUT_PULLUP);
  //Serial.begin(1000000);
  for(int i = 0; i < NUM_CONTROLLERS; i++)
  {
    controllers[i].ID = 0x0F;
    controllers[i].Buttons = 0xFFFF; // all released by default
    controllers[i].Index = i;
    controllers[i].RumbleActive = false;
    if(i == 0) controllers[i].Pin = PLAYER_1;
    else if(i == 1) controllers[i].Pin = PLAYER_2;
    pinMode(controllers[i].Pin, OUTPUT);
  }
  cleanup_psx();
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available() > 0)
  {
    char command = Serial.read();
    if(command == TOGGLE_LOGGING)
    {
      loggingActive = !loggingActive;
    }
    else if(command == START_RUMBLE_P1)
    {
      controllers[0].RumbleActive = true;
    }
    else if(command == STOP_RUMBLE_P1)
    {
      controllers[0].RumbleActive = false;
    }
    else if(command == START_RUMBLE_P2)
    {
      controllers[1].RumbleActive = true;
    }
    else if(command == STOP_RUMBLE_P2)
    {
      controllers[1].RumbleActive = false;
    }
    Serial.flush();
  }
  for(int i = 0; i < NUM_CONTROLLERS; i++)
  {
    Controller *port = &controllers[i];
    if(port->RumbleActive)
    {
      readMode[3] = 0xFF;
      readMode[4] = 0xFF;
    }
    else
    {
      readMode[3] = 0x00;
      readMode[4] = 0x00;
    }
    boolean success = send_string_retry(port, readMode, sizeof(readMode));
    if(!success && port->ID != 0x0F)
    {
      port->ID = 0x0F;
      notify_controllerLost(port);
    }
  }
  delayMicroseconds(3000);
}

void setup_controller(struct Controller *port)
{
  msgLoggingActive = loggingActive;
  delayMicroseconds(50);
  if(!send_string_retry(port, enterConfigMode, sizeof(enterConfigMode)))
  {
    if(loggingActive)
    {
      Serial.print("Failed to enter config mode on controller ");
      Serial.println(port->Index, DEC);
    }
    msgLoggingActive = false;
    return;
  }
  delayMicroseconds(50);
  if(!send_string_retry(port, setAnalogMode, sizeof(setAnalogMode)) && loggingActive)
  {
    Serial.print("Failed to set analog mode on controller ");
    Serial.println(port->Index, DEC);
  }
  delayMicroseconds(50);
  if(!send_string_retry(port, enableRumbleMode, sizeof(enableRumbleMode)) && loggingActive)
  {
    Serial.print("Failed to enable rumble on controller ");
    Serial.println(port->Index, DEC);
  }
  delayMicroseconds(50);
  if(!send_string_retry(port, exitConfigMode, sizeof(exitConfigMode)) && loggingActive)
  {
    Serial.print("Failed to exit config mode on controller ");
    Serial.println(port->Index, DEC);
  }
  msgLoggingActive = false;
}

bool send_string_retry(Controller *port, byte string[], int stringLen)
{
  for(int i = 0; i < 3; i++)
  {
    if(send_string(port, string, stringLen) == true)
    {
      return true;
    }
    delayMicroseconds(25);
  }
  return false;
}

void notify_controllerLost(struct Controller *port)
{
  if(loggingActive)
  {
    Serial.print("Controller ");
    Serial.print(port->Index, DEC);
    Serial.println(" lost.");
  }
  else
  {
    Serial.write(ControllerLost);
    Serial.write(port->Index);
  }
}

void notify_controllerFound(struct Controller *port)
{
  if(loggingActive)
  {
    Serial.print("Controller ");
    Serial.print(port->Index, DEC);
    Serial.println(" found.");
  }
  else
  {
    Serial.write(ControllerFound);
    Serial.write(port->Index);
  }
}

uint16_t memOld, memNew;
void notify_buttons(struct Controller *port, uint16_t dataNew)
{
  for(int i = 0; i < 16; i++)
  {
    memOld = port->Buttons & (1 << i);
    memNew = dataNew & (1 << i);
    if(memOld == memNew) continue;
    if(loggingActive)
    {
      Serial.print("Port ");
      Serial.print(port->Index, DEC); 
      if(memOld < memNew) Serial.print(" released button ");
      else Serial.print(" pressed button ");
      Serial.println(i, DEC);
    }
    else
    {
      Serial.write(ButtonUpdate);
      Serial.write(port->Index);
      Serial.write(i);
      Serial.write(memOld > memNew ? 1 : 0);
    }
  }
}

void notify_axis(struct Controller *port, int axisIndex, byte data)
{
  if(!loggingActive)
  {
    Serial.write(AxisUpdate);
    Serial.write(port->Index);
    Serial.write(axisIndex);
    Serial.write(data);
  }
}

void cleanup_psx()
{
  digitalWrite(COMMAND, HIGH);
  digitalWrite(CLOCK, HIGH);
  for(int i = 0; i < NUM_CONTROLLERS; i++)
  {
    digitalWrite(controllers[i].Pin, HIGH);
  }
}

boolean send_string(struct Controller *port, byte string[], int stringLen)
{
  int newLen = stringLen;
  byte dataBuffer[30];
  int numWords = 0, tmpLen = 0;
  byte mode = string[1];
  digitalWrite(port->Pin, LOW);
  delayMicroseconds(CLOCK_SPEED * 3);
  for(; tmpLen < newLen; tmpLen++)
  {
    byte cmd = (tmpLen < stringLen ? string[tmpLen] : 0x5A);
    dataBuffer[tmpLen] = put_byte(cmd);
    if(tmpLen == 1) newLen = ((dataBuffer[tmpLen] & 0x0F) << 1) + 3;
    
    if(tmpLen != (newLen - 1) && !wait_ack())
    {
      cleanup_psx();
      delayMicroseconds(25);
      return false;
    }
  }
  boolean foundController = false;
  if(mode == 0x42)
  {
    byte newId = dataBuffer[1] >> 4;
    foundController = (newId != port->ID && port->ID == 0x0F);
    port->ID = newId;
    if(foundController)
    {
      notify_controllerFound(port);
    }
    uint16_t buttonsTmp = (dataBuffer[3] << 8) | dataBuffer[4];
    notify_buttons(port, buttonsTmp);
    port->Buttons = buttonsTmp;
    
    for(int axisIndex = 5; axisIndex < newLen; axisIndex++)
    {
      notify_axis(port, axisIndex - 5, dataBuffer[axisIndex]);
    }
  }
  cleanup_psx();
  if(msgLoggingActive)
  {
    Serial.print("send_string(OUT/IN): ");
    for(int i = 0; i < newLen; i++)
    {
      Serial.print(string[i], HEX);
      Serial.print("/");
      Serial.print(dataBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  if(foundController)
  {
    setup_controller(port);
  }
  return true;
}

byte put_byte(byte input)
{
  byte output = 0;
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
  return output;
}

bool wait_ack()
{
  for(int i = 0; i < 75; i++)
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


