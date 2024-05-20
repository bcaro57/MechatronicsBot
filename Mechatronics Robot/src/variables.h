#define SYSTEM_START 0
#define DETECT_BOTTOM_EDGE 1
#define FIRST_BACK_UP 2
#define FIRST_TURN_LEFT 3
#define DETECT_LEFT_EDGE 4
#define SECOND_BACK_UP 5
#define SECOND_TURN_LEFT 6
#define ORIENT_FINAL 7

// Ultra sonic Stuff
long duration;
double distance;

// These variables were determined based on some testing of the motors
int left_LowerStopLimit = 1434;
int left_UpperStopLimit = 1496;
int right_LowerStopLimit = 1430;
int right_UpperStopLimit = 1489;

// These give us our "stopped speed" for the left and right servo, which allows us to fine tune the straight driving capabilities
int left_StoppedSpeed = (left_UpperStopLimit + left_LowerStopLimit)/2 - 5;
int right_StoppedSpeed = (right_UpperStopLimit + right_LowerStopLimit)/2;

/*
'runSpeed' is how fast we are running our servos. The units are arbitrary, but they add/subtract to the StoppedSpeed 
variables to create forward and backward motion (which is written in microseconds of pwm signal). 'rightSpeed' and 
'leftSpeed' are fed to the motors for how fast they should go
*/
int runSpeed = 70;
int rightSpeed = left_StoppedSpeed;
int leftSpeed = right_StoppedSpeed;

// 'currentTime' keeps the time of our system
long currentTime = 0;

long lineTimer = 0;
long ledBlinkingTimer = 0;
bool currentlyCounting = false;
bool hasSeenLine = false;
bool outOfBounds = false;
bool printBoundaryError = false;
int lineCount = 0;

long fireTimerForward = 0;
long fireTimerLeft = 0;
long fireTimerRight = 0;
int upPosition = 1000;
int downPosition = 1400;

int MAP[8][8]={{0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0},
              {1,0,0,0,0,0,0,0}};
int Current_X=0;
int Current_Y=0;
char Orientation= 'F';

int Lines = 0;

bool buttonPressed = false;

bool calibrated = false;
int slowSpeed = 45;
int calibrationState = SYSTEM_START;
long leftTurnTime = 0;
long reverseTime = 0;
long turnLength = 2325;
long backupLength = 1250;
long forwardLength = 1600;

long back_up_time;