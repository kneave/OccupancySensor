#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "controller.h"
#include "sensor.h"
#include <FastLED.h>

#define NUM_LEDS 10
CRGBArray<NUM_LEDS> leds;

//  Pin Definitions
#define LEDPIN 7
#define CE 19
#define CSN 18

// 0 = controller, 1 = peripheral, 2/3 = sensor nodes
int radioNumber = 0;

//  Radio Setup

// Role management: Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.
typedef enum
{
  role_controller = 0,
  role_peripheral,
  role_sensor0,
  role_sensor1
} role_e;
// The various roles supported by this sketch
const char *role_friendly_name[] = {"invalid", "controller", "peripheral", "sensor0", "sensor1"}; // The debug-friendly names of those roles

RF24 radio(CE, CSN);

byte addresses[][6] = {"1Node", "2Node", "3Node", "4Node"};

role_e role = role_e(radioNumber); // The role of the current running sketch

byte counter = 1;

//  peripheral, sensor0, sensor1
unsigned long radiosLastSeen[] = {0, 0, 0, 0};

//  Check the sensor states, generate room states
void CheckSensorStates(bool debug = false)
{
  String debugmsg = "Sensors: ";

  long time = millis();
  for (uint8_t i = 0; i < 6; i++)
  {
    long timeSinceTriggered = time - lastTriggered[i];
    debugmsg += String(timeSinceTriggered) + ",";

    if (timeSinceTriggered < amberRoomDelay)
    {
      //  Set state to red
      sensorStates[i] = state_e(red);
    }
    else if (timeSinceTriggered < greenRoomTrigger)
    {
      //  Set state to amber
      sensorStates[i] = state_e(amber);
    }
    else
    {
      //  Set state to green
      sensorStates[i] = state_e(green);
    }
  }

  if (debug == true)
  {
    Serial.println(debugmsg);
  }
}

void UpdatePeripheral()
{
  radio.stopListening(); // First, stop listening so we can talk.

  //  Update the peripheral, send room status and get footswitch status
  radio.openWritingPipe(addresses[1]);

  if (!radio.write(&roomStateData, sizeof(roomStateData)))
  {
    radiosLastSeen[1] = 0;
  }

  radio.startListening(); // Now, continue listening

  unsigned long started_waiting_at = micros(); // Set up a timeout period, get the current microseconds
  boolean timeout = false;                     // Set up a variable to indicate if a response was received or not

  while (!radio.available())
  { // While nothing is received
    if (micros() - started_waiting_at > 200000)
    { // If waited longer than 200ms, indicate timeout and exit while loop
      timeout = true;
      break;
    }
  }

  if (timeout)
  { // Describe the results
    Serial.println(F("Failed, response timed out."));
  }
  else
  {
    radio.read(&peripheralFootSwitch, sizeof(peripheralFootSwitch));
    // Grab the response, compare, and send to debugging spew
    Serial.print("Remote footswitch: ");
    Serial.println(peripheralFootSwitch);

    radiosLastSeen[1] = millis();
  }
}

void UpdateRoomData(int roomId)
{
  radio.stopListening(); // First, stop listening so we can talk.
  int radioId = 2 + roomId;

  //  send a byte to initiate a response
  radio.openWritingPipe(addresses[radioId]);

  if (!radio.write(&roomStateData, sizeof(roomStateData)))
  {
    radiosLastSeen[radioId] = 0;
  }

  radio.startListening(); // Now, continue listening

  unsigned long started_waiting_at = micros(); // Set up a timeout period, get the current microseconds
  boolean timeout = false;                     // Set up a variable to indicate if a response was received or not

  while (!radio.available())
  { // While nothing is received
    if (micros() - started_waiting_at > 200000)
    { // If waited longer than 200ms, indicate timeout and exit while loop
      timeout = true;
      break;
    }
  }

  if (timeout)
  { // Describe the results
    Serial.println(F("Failed, response timed out."));
  }
  else
  {
    radio.read(&peripheralFootSwitch, sizeof(peripheralFootSwitch));
    // Grab the response, compare, and send to debugging spew
    Serial.print("Remote footswitch: ");
    Serial.println(peripheralFootSwitch);

    radiosLastSeen[radioId] = millis();
  }
}

void PrintRoomStates()
{
  Serial.print(millis());
  Serial.print(" - Room States: ");
  Serial.print(roomStateData.room1);
  Serial.print(",");
  Serial.print(roomStateData.room2);
  Serial.print(",");
  Serial.print(roomStateData.alley_front);
  Serial.print(",");
  Serial.println(roomStateData.alley_rear);
}

void InitLights()
{
  FastLED.addLeds<WS2812, LEDPIN, RGB>(leds, NUM_LEDS);

  leds[0] = CRGB::Blue;
  for (int i = 1; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void SetRoomLEDs(state_e state, int start)
{
  switch (state)
  {
  case state_e(green):
    leds[start] = CRGB::Green;
    leds[start + 1] = CRGB::Black;
    leds[start + 2] = CRGB::Black;
    break;
  case state_e(amber):
    leds[start] = CRGB::Black;
    leds[start + 1] = CRGB::Yellow;
    leds[start + 2] = CRGB::Black;
    break;
  case state_e(red):
  case state_e(waiting):
    leds[start] = CRGB::Black;
    leds[start + 1] = CRGB::Black;
    leds[start + 2] = CRGB::Red;
    break;
  case state_e(off):
    leds[start] = CRGB::Black;
    leds[start + 1] = CRGB::Black;
    leds[start + 2] = CRGB::Black;
    break;
  }
}

void UpdateLEDS(int radioNumber, unsigned long radioLastSeenDiff)
{
  //  Radio status LED
  if (radioLastSeenDiff < 5000)
  {
    leds[0] = CRGB::Green;
  }
  else if (radioLastSeenDiff < 10000)
  {
    leds[0] = CRGB::Yellow;
  }
  else
  {
    leds[0] = CRGB::Red;
  }

  //  Alley
  if (radioNumber == 0)
  {
    SetRoomLEDs(roomStateData.alley_front, 1);
  }
  else
  {
    SetRoomLEDs(roomStateData.alley_rear, 1);
  }

  //  Room 1
  SetRoomLEDs(roomStateData.room1, 4);

  //  Room 2
  SetRoomLEDs(roomStateData.room2, 7);

  FastLED.show();
}

void UpdateRadio()
{
  //  Controller functions
  if (radioNumber == 0)
  {
    UpdatePeripheral();
    //  UpdateRoomOne();
    //  UpdateRoomTwo();

    // Try again 1s later
    delay(50);
  }

  //  I could use an "else" but this makes it easier to read the code
  if (radioNumber == 1)
  {
    if (radio.available())
    {
      // Variable for the received timestamp
      while (radio.available())
      {                                                    // While there is data ready
        radio.read(&roomStateData, sizeof(roomStateData)); // Get the payload
      }
      radio.stopListening();                                            // First, stop listening so we can talk
      radio.write(&peripheralFootSwitch, sizeof(peripheralFootSwitch)); // Send the final one back.
      radio.startListening();                                           // Now, resume listening so we catch the next packets.
      radioLastSeen = millis();
    }
  }

  if (radioNumber == 2 | radioNumber == 3)
  {
    if (radio.available())
    {
      byte incoming;
      while (radio.available())
      {                                          // While there is data ready
        radio.read(&incoming, sizeof(incoming)); // Get the payload
      }
      radio.stopListening();                      // First, stop listening so we can talk
      radio.write(&roomState, sizeof(roomState)); // Send the sensor states back.
      radio.startListening();                     // Now, resume listening so we catch the next packets.
      radioLastSeen = millis();
    }
  }
}

void InitRadio()
{
  //  radio init
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);

  switch (radioNumber)
  {
  case 0:
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    break;
  case 1:
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    break;
  case 2:
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[2]);
    break;
  case 3:
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[3]);
    break;
  }

  radio.startListening(); // Start listening
  radio.printDetails();   // Dump the configuration of the rf unit for debugging
}

void setup()
{
  Serial.begin(115200);

  if (radioNumber <= 1)
  {
    InitController();

    //  Init the footswitch
    Serial.println("Set footswitch pullup...");
    pinMode(FOOTSWITCH, INPUT_PULLUP);
  }
  else
  {
    InitSensors();
  }

  //  LED init
  Serial.println("Init lights...");
  InitLights();

  Serial.println("Init Radio...");
  InitRadio();

  Serial.println("Setup complete.");
}

void loop()
{
  if (radioNumber == 0)
  {
    ReadSensors();
    CheckSensorStates(true);
    //  CheckRoomStates();
    PrintRoomStates();
    controllerFootSwitch = digitalRead(FOOTSWITCH) == LOW;

    //  If button pressed on either side, state_e(4)
    if (roomStateData.alley_front == state_e(red))
    {
      if (controllerFootSwitch == HIGH)
      {
        roomStateData.alley_front = state_e(waiting);
      }
    }

    if (roomStateData.alley_rear == state_e(red))
    {
      if (peripheralFootSwitch == HIGH)
      {
        roomStateData.alley_rear = state_e(waiting);
      }
    }
  }
  else
  {
    peripheralFootSwitch = digitalRead(FOOTSWITCH) == LOW;
  }

  unsigned long radioLastSeenDiff = millis() - radiosLastSeen[radioNumber];

  UpdateLEDS(radioNumber, radioLastSeenDiff);
  UpdateRadio();
}