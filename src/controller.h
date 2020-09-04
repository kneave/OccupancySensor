//  All code specific to the controller/peripheral
//  Pin Definitions
#define FOOTSWITCH 8

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

int amberAlleyDelay = 10000;
int redAlleyDelay = 14000;

//  Interval is how quickly to flash while waiting on the alley
int flashInterval = 500;
//  Flash last is the time since we last toggled the LED
int flashLast = 0;

int controllerFootSwitch = 0;
int peripheralFootSwitch = 0;
unsigned long footswitchLastSeen[] = {0, 0};

state_e CheckAlley(state_e here, state_e there, int footswitch)
{
    //  If waiting and the other side is green, flash red
    switch (here)
    {
    case state_e(off):
    case state_e(waiting):
        if (there == state_e(red))
        {
            here = state_e(green);
            footswitchLastSeen[footswitch] = millis();
        }
        else if (millis() - flashLast > flashInterval)
        {
            flashLast = millis();
            here = here == state_e(waiting) ? state_e(off) : state_e(waiting);
        }
        break;
    case state_e(green):
        if (millis() - footswitchLastSeen[footswitch] > amberAlleyDelay)
        {
            here = state_e(amber);
        }
        break;
    case state_e(amber):
        if (millis() - footswitchLastSeen[footswitch] > redAlleyDelay)
        {
            here = state_e(red);
        }
        break;
    }

    return here;
}

void InitController()
{
    //  Assume all rooms busy until proven otherwise
    roomStateData.alley_front = state_e(red);
    roomStateData.alley_rear = state_e(red);
    roomStateData.room1 = state_e(red);
    roomStateData.room2 = state_e(red);
}