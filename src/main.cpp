#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <NewPing.h>
#include <FastLED.h>

//  Pin Definitions
#define SONAR1 2
#define SONAR2 3
#define SONAR3 4
#define SONAR4 5
#define LEDPIN 7
#define FOOTSWITCH 8
#define CE 19
#define CSN 18
#define PIR0 20
#define PIR1 21

// 0 = controller, 1 = peripheral
bool radioNumber = 0;

//  Last time in millis since last used the radio
long radioLastSeen = 1;

#define NUM_LEDS 10
CRGBArray<NUM_LEDS> leds;

//  Sensor Setup
#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
    NewPing(10, SONAR1, MAX_DISTANCE),
    NewPing(10, SONAR2, MAX_DISTANCE),
    NewPing(10, SONAR3, MAX_DISTANCE),
    NewPing(10, SONAR4, MAX_DISTANCE)};

//  Distance below which sonar considered triggered
int sonarTriggerDistance[] = {10, 10, 10, 10};

//  Time in milliseconds from last activation
//  If time is greater than the value, that state is active
//  red is < amber
int amberRoomDelay = 7500;
int greenRoomTrigger = 15000;

int amberAlleyDelay = 10000;
int redAlleyDelay = 14000;

//  Interval is how quickly to flash while waiting on the alley
int flashInterval = 500;
//  Flash last is the time since we last toggled the LED
int flashLast = 0;

int pir0value = LOW;
int pir1value = LOW;
int controllerFootSwitch = LOW;
int peripheralFootSwitch = LOW;

//  Radio Setup

// Role management: Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.
typedef enum
{
  role_controller = 0,
  role_peripheral
} role_e;
// The various roles supported by this sketch
const char *role_friendly_name[] = {"invalid", "controller", "peripheral"}; // The debug-friendly names of those roles

RF24 radio(CE, CSN);

byte addresses[][6] = {"1Node", "2Node"};

role_e role = role_e(radioNumber); // The role of the current running sketch

byte counter = 1;

//  Setup variables for sharing room states
//  0 = LED off, 1 = engaged, 2 = possibly vacant, 3 = vacant, 4 = waiting (alleyway)
typedef enum
{
  off,
  red,
  amber,
  green,
  waiting
} state_e;

struct RoomStates
{
  state_e room1;
  state_e room2;
  state_e alley_front;
  state_e alley_rear;
} roomStateData;

//  sonar 0 to 3, pir0, pir1, controller footswitch, peripheral footswitch
state_e sensorStates[] = {
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1)};

typedef enum
{
  room1_1,
  room1_2,
  room2_1,
  room2_2,
  pir0,
  pir1,
  contFootswitch,
  periFootswitch
} sensor_e;

//  sensor_e references the locations below
long lastTriggered[] = {0, 0, 0, 0, 0, 0, 0, 0};

void PrintRoomStates()
{
  Serial.print(millis());
  Serial.print("-");
  Serial.print(radioNumber);
  Serial.print("- Room States: ");
  Serial.print(roomStateData.room1);
  Serial.print(",");
  Serial.print(roomStateData.room2);
  Serial.print(",");
  Serial.print(roomStateData.alley_front);
  Serial.print(",");
  Serial.println(roomStateData.alley_rear);
}

void CheckRoomStates()
{
  if ((sensorStates[sensor_e(room1_1)] == state_e(red)) | (sensorStates[sensor_e(room1_2)] == state_e(red)) | (sensorStates[sensor_e(pir0)] == state_e(red)))
  {
    roomStateData.room1 = state_e(red);
  }
  else if ((sensorStates[sensor_e(room1_1)] == state_e(amber)) | (sensorStates[sensor_e(room1_2)] == state_e(amber)) | (sensorStates[sensor_e(pir0)] == state_e(amber)))
  {
    roomStateData.room1 = state_e(amber);
  }
  else
  {
    roomStateData.room1 = state_e(green);
  }

  if ((sensorStates[sensor_e(room2_1)] == state_e(red)) | (sensorStates[sensor_e(room2_2)] == state_e(red)) | (sensorStates[sensor_e(pir1)] == state_e(red)))
  {
    roomStateData.room2 = state_e(red);
  }
  else if ((sensorStates[sensor_e(room2_1)] == state_e(amber)) | (sensorStates[sensor_e(room2_2)] == state_e(amber)) | (sensorStates[sensor_e(pir0)] == state_e(amber)))
  {
    roomStateData.room2 = state_e(amber);
  }
  else
  {
    roomStateData.room2 = state_e(green);
  }
}

state_e CheckAlley(state_e here, state_e there, sensor_e sensor)
{
  //  If waiting and the other side is green, flash red
  switch (here)
  {
  case state_e(off):
  case state_e(waiting):
    if (there == state_e(red))
    {
      here = state_e(green);
      lastTriggered[sensor] = millis();
    }

    if (millis() - flashLast > flashInterval)
    {
      flashLast = millis();
      here = here == state_e(waiting) ? state_e(off) : state_e(waiting);
    }
    break;
  case state_e(green):
    if (millis() - lastTriggered[sensor] > amberAlleyDelay)
    {
      here = state_e(amber);
    }
    break;
  case state_e(amber):
    if (millis() - lastTriggered[sensor] > redAlleyDelay)
    {
      here = state_e(red);
    }
    break;
  }

  return here;
}

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

  roomStateData.alley_front = CheckAlley(roomStateData.alley_front, roomStateData.alley_rear, contFootswitch);
  // roomStateData.alley_rear = CheckAlley(roomStateData.alley_rear, roomStateData.alley_front, periFootswitch);

  if (debug == true)
  {
    Serial.println(debugmsg);
  }
}

void ReadSensors(bool debug = false)
{
  if (debug == true)
  {
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {            // Loop through each sensor and display results.
      delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      Serial.print(i);
      Serial.print("=");
      Serial.print(sonar[i].ping_cm());
      Serial.print("cm ");
    }
    Serial.print("PIR0 ");
    Serial.print(digitalRead(PIR0) == HIGH);

    Serial.print(" PIR1 ");
    Serial.print(digitalRead(PIR1) == HIGH);

    Serial.println();
  }
  else
  {
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
      delay(40);
      int distance = sonar[i].ping_cm();
      if (distance > 0 && distance < sonarTriggerDistance[i])
      {
        lastTriggered[i] = millis();
      }
    }

    if (digitalRead(PIR0) == HIGH)
    {
      lastTriggered[sensor_e(pir0)] = millis();
    }

    if (digitalRead(PIR1) == HIGH)
    {
      lastTriggered[sensor_e(pir1)] = millis();
    }
  }
}

void UpdateRadio()
{
  if (radioNumber == 0)
  {
    radio.stopListening(); // First, stop listening so we can talk.

    if (!radio.write(&roomStateData, sizeof(roomStateData)))
    {
      radioLastSeen = 0;
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
      // Grab the response, compare, and send to debugging spew
      radio.read(&peripheralFootSwitch, sizeof(peripheralFootSwitch));
      Serial.print("Remote footswitch: ");
      Serial.println(peripheralFootSwitch);
      radioLastSeen = millis();
    }

    // Try again 1s later
    delay(50);
  }

  //  I could use an "else", this makes it easier to read the code
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

void UpdateLEDS()
{
  //  Radio status LED
  long radioLastSeenDiff = millis() - radioLastSeen;
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
  SetRoomLEDs(roomStateData.alley_front, 1);

  //  Room 1
  SetRoomLEDs(roomStateData.room1, 4);

  //  Room 2
  SetRoomLEDs(roomStateData.room2, 7);

  FastLED.show();
}

void InitRadio()
{
  //  radio init
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);

  if (radioNumber)
  {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  }
  else
  {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  radio.startListening(); // Start listening
  radio.printDetails();   // Dump the configuration of the rf unit for debugging
}

void setup()
{
  Serial.begin(115200);

  //  LED init
  Serial.println("Init lights...");
  InitLights();

  //  Init the footswitch
  Serial.println("Set footswitch pullup...");
  pinMode(FOOTSWITCH, INPUT_PULLUP);

  //  Sensor init

  Serial.println("Set PIR inputs...");
  pinMode(PIR0, INPUT);
  pinMode(PIR1, INPUT);

  Serial.println("Init Radio...");
  InitRadio();

  //  Assume all rooms busy until proven otherwise
  roomStateData.alley_front = state_e(red);
  roomStateData.alley_rear = state_e(red);
  roomStateData.room1 = state_e(red);
  roomStateData.room2 = state_e(red);

  Serial.println("Setup complete.");
}

void loop()
{
  if (radioNumber == 0)
  {
    ReadSensors();
    CheckSensorStates(true);
    CheckRoomStates();
    PrintRoomStates();
    controllerFootSwitch = digitalRead(FOOTSWITCH) == LOW;
  }
  else
  {
    peripheralFootSwitch = digitalRead(FOOTSWITCH) == LOW;
  }

  //  Implement footswitch logic here
  //  If button pressed on either side, state_e(4)

  if ((roomStateData.alley_front != state_e(waiting)) & (roomStateData.alley_front != state_e(off)))
  {
    if (controllerFootSwitch == HIGH)
    {
      roomStateData.alley_front = state_e(waiting);
    }
  }

  // if (peripheralFootSwitch == LOW)
  // {
  //   if ((roomStateData.alley_rear != state_e(waiting)) | (roomStateData.alley_rear != state_e(off)))
  //   {
  //     roomStateData.alley_rear = state_e(waiting);
  //   }
  // }

  UpdateLEDS();
  UpdateRadio();
}