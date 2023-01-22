//The header file template for a ROSS model
//This file includes:
// - the state and message structs
// - extern'ed command line arguments
// - custom mapping function prototypes (if needed)
// - any other needed structs, enums, unions, or #defines

#ifndef _model_h
#define _model_h
#include "ross.h"

#define CHARGE_STATION 4
#define GENERATOR_STATION 2
#define CARGO_PLACE_MAX_NUM 25
#define CARGO_PLACE_MIN_NUM 5
#define MAX_ROW 9
#define MAX_COL 10

static int g_depositary[10][11] = 
{
//  0  1  2  3  4  5  6  7  8  9 10
  { 0,17,19,21,23,25,24,22,20,18, 0 }, // 0
  {15, 1, 1, 1, 1, 1, 1, 1, 1, 1,16 }, // 1
  {13, 1, 1, 1, 1, 1, 1, 1, 1, 1,14 }, // 2
  {11, 1, 1, 1, 1, 1, 1, 1, 1, 1,12 }, // 3
  { 9, 1, 1, 1, 1, 1, 1, 1, 1, 1,10 }, // 4
  { 7, 1, 1, 1, 1, 1, 1, 1, 1, 1, 8 }, // 5
  { 5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 6 }, // 6
  { 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4 }, // 7
  { 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4 }, // 8
  { 4, 1, 1, 2, 1, 1, 1, 2, 1, 1, 4 }, // 9
};

#define NO_ROBOT 0
#define EMPTY_ROBOT 1
#define CARGO_ROBOT 2
#define NO_FUEL_ROBOT 3
static int g_robots[10][11] = 
{
//  0  1  2  3  4  5  6  7  8  9 10
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 0
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 3
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 8
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 9
};

// All times in second
const static double g_robot_calc_time = 0.001;
const static double g_robot_wait_time = 0.5;
const static double g_robot_move_time = 0.5;
const static double g_robot_round_time = 4;
const static double g_robot_load_time = 2;
const static double g_robot_unload_time = 1;
const static double g_robot_charge_time = 12 * 60;

// All distances in meters
const static double g_robot_max_mileage = 12 * 60 * 10;
const static double g_robot_min_mileage = 12 * 60 * 5;

#define MAX_CARGO_NUMBER 100005
#define NO_CARGO 1337

typedef enum
{
  MSGT_ROBOT_TO_ROBOT,
  MSGT_ROBOT_TO_GENERATOR,
  MSGT_GENERATOR_TO_ROBOT
} msgType;

typedef enum
{
  EVENTR_STAY,
  EVENTR_MOVE,
  EVENTR_ROUND,
  EVENTR_CREATE_ROUTE_TO_GENERATOR,
  EVENTR_CREATE_ROUTE_TO_UNLOAD,
  EVENTR_CREATE_ROUTE_TO_CHARGE,
} robotEventType;

typedef enum
{
  D_UP    = 0,
  D_LEFT  = 1,
  D_DOWN  = 2,
  D_RIGHT = 3,
  D_STAY  = 4
} Direction;

typedef enum
{
  A_0,
  A_90,
  A_180,
  A_M90,
} Angle;

typedef struct
{
  msgType type;
  union
  {
    // Robot to robot message
    struct 
    {
      robotEventType event_type;
      Direction direction;
      Angle angle;
    } r_r;

    // Robot to generator message
    struct 
    {
      tw_lpid robot_id;
    } r_g;

    // Generator to robot message
    struct 
    {
      int cargo_num;
    } g_r;

  } body;
} message;

typedef struct
{
  int row;
  int col;
  Direction look_direction;
  int mileage_remain;
  int cargo_num;
} robot_state;

typedef struct
{
  // cargo_timetable[i][0] - arrival of cargo in offset from simulation start
  // cargo_timetable[i][1] - number of place to put cargo
  int cargo_timetable[MAX_CARGO_NUMBER][2];
  int current_cargo;
} generator_state;

//MAPPING -----------------------------
enum lpTypeVals
{
  TYPE_GENERATOR = 0,
  TYPE_ROBOT = 1
};

extern tw_lpid lpTypeMapper(tw_lpid gid);
extern tw_peid model_map(tw_lpid gid);

extern void robot_init(robot_state *s, tw_lp *lp);
extern void robot_prerun(robot_state *s, tw_lp *lp);
extern void robot_event_handler(robot_state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void robot_RC_event_handler(robot_state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void robot_final(robot_state *s, tw_lp *lp);
extern void robot_commit(robot_state *s, tw_bf *bf, message *m, tw_lp *lp);


extern void generator_init(generator_state *s, tw_lp *lp);
extern void generator_prerun(generator_state *s, tw_lp *lp);
extern void generator_event_handler(generator_state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void generator_RC_event_handler(generator_state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void generator_final(generator_state *s, tw_lp *lp);
extern void generator_commit(generator_state *s, tw_bf *bf, message *m, tw_lp *lp);

//MAIN STUFF-----------------------------

extern tw_lptype model_lps[];
unsigned int custom_LPs_per_pe;

int total_robots;
int total_generators;

#endif
