#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <NewPing.h>
#include <FastLED.h>
#define NUM_LEDS 10

CRGBArray<NUM_LEDS> leds;

//  Sensor Setup
#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PIR0 20          // A2
#define PIR1 21          // A3
#define FOOTSWITCH 7

NewPing sonar[SONAR_NUM] = { // Sensor object array.
    NewPing(10, 2, MAX_DISTANCE),
    NewPing(10, 3, MAX_DISTANCE),
    NewPing(10, 4, MAX_DISTANCE),
    NewPing(10, 5, MAX_DISTANCE)};

//  Distance below which sonar considered triggered
int sonarTriggerDistance[] = {10, 10, 10, 10};

//  Time in milliseconds from last activation
//  If time is greater than the value, that state is active
//  red is < amber
int amberTrigger = 15000;
int greenTrigger = 30000;

int pir0value = LOW;
int pir1value = LOW;
int footswitchvalue = LOW;

//  Radio Setup

// 0 = controller, 1 = peripheral
bool radioNumber = 1;
RF24 radio(19, 18);

byte addresses[][6] = {"1Node", "2Node"}; // Radio pipe addresses for the 2 nodes to communicate.

// Role management: Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.
typedef enum
{
  role_controller = 0,
  role_peripheral
} role_e;                                                                   // The various roles supported by this sketch
const char *role_friendly_name[] = {"invalid", "controller", "peripheral"}; // The debug-friendly names of those roles
role_e role = role_e(radioNumber);                                          // The role of the current running sketch

byte counter = 1;

typedef enum
{
  red = 1,
  amber,
  green
} state_e;

//  Setup variables for sharing room states
//  1 = engaged, 2 = possibly vacant, 3 = vacant
struct RoomStates
{
  state_e room1;
  state_e room2;
  state_e alley_front;
  state_e alley_rear;
} roomStateData;

//  sonar 0 to 3, pir0, pir1, footswitch
state_e sensorStates[] = {
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1),
    state_e(1)
};

typedef enum
{
  room1_1,
  room1_2,
  room2_1,
  room2_2,
  pir0,
  pir1,
  footswitch
} sensor_e;

//  sensor_e references the locations below
long lastTriggered[] = {0, 0, 0, 0, 0, 0, 0};

void PrintRoomStates()
{
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
  if(sensorStates[sensor_e(room1_1)] == state_e(1) 
  | sensorStates[sensor_e(room1_2)] == state_e(1) 
  | sensorStates[sensor_e(pir0)] == state_e(1))
  {
      roomStateData.room1 = state_e(1);
  }
  else if(sensorStates[sensor_e(room1_1)] == state_e(2) 
    | sensorStates[sensor_e(room1_2)] == state_e(2)
    | sensorStates[sensor_e(pir0)] == state_e(2))
  {
      roomStateData.room1 = state_e(2);
  }
  else
  {
    roomStateData.room1 = state_e(3);
  }
    
  if(sensorStates[sensor_e(room2_1)] == state_e(1) 
  | sensorStates[sensor_e(room2_2)] == state_e(1)
  | sensorStates[sensor_e(pir1)] == state_e(1))
  {
      roomStateData.room2 = state_e(1);
  }
  else if(sensorStates[sensor_e(room2_1)] == state_e(2) 
  | sensorStates[sensor_e(room2_2)] == state_e(2)
  | sensorStates[sensor_e(pir1)] == state_e(2))
  {
      roomStateData.room2 = state_e(2);
  }
  else
  {
    roomStateData.room2 = state_e(3);
  }
}

//  Check the sensor states, generate room states
void CheckSensorStates()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  {
    long timeSinceTriggered = millis() - lastTriggered[i];

    if (timeSinceTriggered < amberTrigger)
    {
      //  Set state to red
      sensorStates[i] = state_e(1);
    } 
    else if(timeSinceTriggered < greenTrigger)
    {
      //  Set state to amber
      sensorStates[i] = state_e(2);  
    }
    else
    {
      //  Set state to green
      sensorStates[i] = state_e(3);
    }    
  }

  

}

void ReadSensors()
{
  bool debug = false;
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
    Serial.print(digitalRead(PIR0));

    Serial.print(" PIR1 ");
    Serial.print(digitalRead(PIR1));

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

    footswitchvalue = analogRead(FOOTSWITCH);
  }
}

void setup()
{
  Serial.begin(115200);

  //  LED init
  FastLED.addLeds<NEOPIXEL,7>(leds, NUM_LEDS);

  //  Sensor init
  pinMode(PIR0, INPUT);
  pinMode(PIR1, INPUT);
  pinMode(FOOTSWITCH, INPUT);

  //  radio init
  radio.begin();

  radio.enableAckPayload();      // Allow optional ack payloads
  radio.enableDynamicPayloads(); // Ack payloads are dynamic payloads

  if (radioNumber == 1)
  {
    radio.openWritingPipe(addresses[1]);    // Both radios listen on the same pipes by default, but opposite addresses
    radio.openReadingPipe(1, addresses[0]); // Open a reading pipe on address 0, pipe 1
  }
  else
  {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }
  radio.startListening(); // Start listening

  radio.writeAckPayload(1, &counter, 1); // Pre-load an ack-paylod into the FIFO buffer for pipe 1
  //radio.printDetails();

  //  Assume all rooms busy until proven otherwise
  roomStateData.alley_front = state_e(1);
  roomStateData.alley_rear = state_e(1);
  roomStateData.room1 = state_e(1);
  roomStateData.room2 = state_e(1);
}

void loop()
{
  // static uint8_t hue;
  // for(int i = 0; i < NUM_LEDS/2; i++) {   
  //   // fade everything out
  //   leds.fadeToBlackBy(40);

  //   // let's set an led value
  //   leds[i] = CHSV(hue++,255,255);

  //   // now, let's first 20 leds to the top 20 leds, 
  //   leds(NUM_LEDS/2,NUM_LEDS-1) = leds(NUM_LEDS/2 - 1 ,0);
  //   FastLED.delay(33);
  // }

  //  If 1, access requested
  footswitchvalue = digitalRead(FOOTSWITCH);

  if (radioNumber == 0)
  {
    byte gotByte; // Initialize a variable for the incoming response

    radio.stopListening(); // First, stop listening so we can talk.
    // Serial.print(F("Now sending ")); // Use a simple byte counter as payload
    // Serial.println(counter);

    unsigned long time = micros(); // Record the current microsecond count

    if (radio.write(&roomStateData, sizeof(roomStateData)))
    { // Send the counter variable to the other radio
      if (!radio.available())
      { 
        //  Set status LED to red
      }
      else
      {
        while (radio.available())
        {                          // If an ack with payload was received
          radio.read(&gotByte, 1); // Read it, and display the response time
          unsigned long timer = micros();

          // Serial.print(F("Got response "));
          // Serial.print(gotByte);
          // Serial.print(F(" round-trip delay: "));
          // Serial.print(timer - time);
          // Serial.println(F(" microseconds"));
          counter++; // Increment the counter variable
        }
      }

      ReadSensors();
      CheckSensorStates();
      CheckRoomStates();
      PrintRoomStates();
    }
    else
    {
      Serial.print(millis());
      Serial.println(F(": sending failed.")); // If no ack response, sending failed
      delay(1000);
    }
  }
  else
  {
    //  Radio Number 2
    byte pipeNo, gotByte; // Declare variables for the pipe and the byte received
    while (radio.available(&pipeNo))
    { // Read all available payloads
      radio.read(&roomStateData, sizeof(roomStateData));
      // Since this is a call-response. Respond directly with an ack payload.
      gotByte += 1;                               // Ack payloads are much more efficient than switching to transmit mode to respond to a call
      // radio.writeAckPayload(pipeNo, &gotByte, 1); // This can be commented out to send empty payloads.
      Serial.print(F("Loaded response "));
      Serial.print(roomStateData.room1);
      Serial.print(",");
      Serial.println(roomStateData.room2);
    }
  }
}