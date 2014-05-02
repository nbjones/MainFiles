//message types for sensorTask
#define GATHER_MSG 0
#define GATHER_CHECK 1
#define SENSORVALUE_MSG 2
#define ROVERMOVE_MSG 3
#define GATHER_ERROR_MSG 4
#define MACROSTATE_OVERRIDE 5

//message types for motorTask
#define SENSORTASK_MSG 10
#define ROVERACK_ERROR 11
#define ROVERACK_CHECK 12
#define ROVERMOVE_CHECK 13
#define ROVERMOVE_CHECKCHECK 14

//message types for motorTask/sensorTask communication
#define ROVERMOVE_FORWARD_ABSOLUTE 20
#define ROVERMOVE_FORWARD_CORRECTED 21
#define ROVERMOVE_REVERSE 22
#define ROVERMOVE_TURN_LEFT 23
#define ROVERMOVE_TURN_RIGHT 24
#define ROVERMOVE_STOP 25

//movement algorithm states
#define ALG_FORWARD 30
#define ALG_STOPPED	31
#define ALG_AGAINST_OBSTACLE 32
#define ALG_ON_CORNER 33
#define ALG_CLEARING 34

//macroState states
#define MACROSTATE_IDLE 40
#define MACROSTATE_FINDING_LINE 41
#define MACROSTATE_RUN_ONE 42
#define MACROSTATE_FINISHED 43
#define MACROSTATE_RUN_TWO 44