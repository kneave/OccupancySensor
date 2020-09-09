//  All code specific to the sensor nodes
#include <NewPing.h>

//  Pin Definitions
#define SONAR0 2
#define SONAR1 3
#define PIR 20

//  Sensor Setup
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

//  Setup variables for sharing room states
//  1 = engaged, 2 = possibly vacant, 3 = vacant

//  sonar 0, sonar 1, pir
int sensorStates[] = {
    1,
    1,
    1};

int roomState;

typedef enum
{
    distance0,
    distance1,
    pir,
} sensor_e;

//  sensor_e references the locations below
unsigned long lastTriggered[] = {0, 0, 0};

NewPing sonar[SONAR_NUM] = { // Sensor object array.
    NewPing(10, SONAR0, MAX_DISTANCE),
    NewPing(10, SONAR1, MAX_DISTANCE)};

//  Distance below which sonar considered triggered
//  Note: This will be different between rooms so needs splitting or setting in setup
int sonarTriggerDistance[] = {10, 10};

int pirValue = LOW;

//  Time in milliseconds from last activation
//  If time is greater than the value, that state is active
//  red is < amber
int amberRoomDelay = 7500;
int greenRoomTrigger = 15000;

void ReadSensors(bool debug = false)
{
    if (debug == true)
    {
        for (uint8_t i = 0; i < SONAR_NUM; i++)
        {              // Loop through each sensor and display results.
            delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
            Serial.print(i);
            Serial.print("=");
            Serial.print(sonar[i].ping_cm());
            Serial.print("cm ");
        }
        Serial.print("PIR0 ");
        Serial.print(digitalRead(PIR) == HIGH);

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

        if (digitalRead(PIR) == HIGH)
        {
            lastTriggered[sensor_e(pir)] = millis();
        }
    }
}

//  Update the state of the room this node is in
//  1: green, 2: amber, 3: green
void UpdateRoomState()
{
    if ((sensorStates[sensor_e(distance0)] == 1) | (sensorStates[sensor_e(distance1)] == 1) | (sensorStates[sensor_e(pir)] == 1))
    {
        roomState = 1;
    }
    else if ((sensorStates[sensor_e(distance0)] == 2) | (sensorStates[sensor_e(distance1)] == 2) | (sensorStates[sensor_e(pir)] == 2))
    {
        roomState = 2;
    }
    else
    {
        roomState = 3;
    }
}

void InitSensors()
{
    Serial.println("Set PIR inputs...");
    pinMode(PIR, INPUT);

    roomState = 1;
}

//  This will only be called when data requested
int ReturnState()
{
    return roomState;
}