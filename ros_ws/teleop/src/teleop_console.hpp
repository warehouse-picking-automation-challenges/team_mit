#include <ncurses.h>
#include <unistd.h>
#include <string>
#include <cstdio>

using namespace std;

#define BKG_COLOR 1
#define KEYS_COLOR 2
#define ENABLE_COLOR 3
#define DISABLE_COLOR 4
#define DATA_COLOR 5

typedef enum KEY_CMDS
{
  DEC_X = 0,
  DEC_Y,
  DEC_Z,
  INC_X,
  INC_Y,
  INC_Z,
  DEC_RX,
  DEC_RY,
  DEC_RZ,
  INC_RX,
  INC_RY,
  INC_RZ,
  DEC_J1,
  DEC_J2,
  DEC_J3,
  DEC_J4,
  DEC_J5,
  DEC_J6,
  INC_J1,
  INC_J2,
  INC_J3,
  INC_J4,
  INC_J5,
  INC_J6,
  ALIGN,
  DISABLE,
  ENABLE,
  DEC_TCP,
  INC_TCP,
  DEC_ORI,
  INC_ORI,
  QUIT,
  COARSE,
  FINE,
  NO_KEY,
  UNKNOWN_KEY,
  NUM_KEY_CMDS
} KEY_CMDS;

typedef struct cmd_info
{
  char key;
  int row, col;
  const char* text;
} cmd_info;

const cmd_info key_lib[NUM_KEY_CMDS] = 
{
  {'X', 17, 6, " X "},  // DEC_X
  {'Y', 18, 6, " Y "},  // DEC_Y
  {'Z', 19, 6, " Z "},  // DEC_Z
  {'x', 20, 6, " x "},  // INC_X
  {'y', 21, 6, " y "},  // INC_Y
  {'z', 22, 6, " z "},  // INC_Z
  {'A', 17, 15, " A "}, // DEC_RX
  {'S', 18, 15, " S "}, // DEC_RY
  {'D', 19, 15, " D "}, // DEC_RZ
  {'a', 20, 15, " a "}, // INC_RX
  {'s', 21, 15, " s "}, // INC_RY
  {'d', 22, 15, " d "}, // INC_RZ
  {'!', 17, 24, " ! "}, // DEC_J1
  {'@', 18, 24, " @ "}, // DEC_J2
  {'#', 19, 24, " # "}, // DEC_J3
  {'$', 20, 24, " $ "}, // DEC_J4
  {'%', 21, 24, " %% "},// DEC_J5
  {'^', 22, 24, " ^ "}, // DEC_J6
  {'1', 17, 33, " 1 "}, // INC_J1
  {'2', 18, 33, " 2 "}, // INC_J2
  {'3', 19, 33, " 3 "}, // INC_J3
  {'4', 20, 33, " 4 "}, // INC_J4
  {'5', 21, 33, " 5 "}, // INC_J5
  {'6', 22, 33, " 6 "}, // INC_J6
  {'0', 22, 49, " 0 "}, // ALIGN
  {'e', 21, 65, " e "}, // DISABLE
  {'E', 21, 49, " E "}, // ENABLE
  {'P', 18, 49, " P "}, // DEC_TCP
  {'p', 18, 65, " p "}, // INC_TCP
  {'R', 19, 49, " R "}, // DEC_ORI
  {'r', 19, 65, " r "}, // INC_ORI
  {'q', 22, 65, " q "}, // QUIT
  {'<', 17, 49, " < "}, // FINE
  {'>', 17, 65, " > "}, // COARSE
  {ERR, 0, 0, "ERR"},   // NO_KEY
  {ERR, 0, 0, "UNK"}    // UNKNOWN_KEY
};

void console_init();
void console_update_cartesian(double cart[7]);
void console_update_joints(double joints[6]);
void console_update_enabled(bool enabled);
void console_update_motion_mode(double curLinStep);
void console_update_workObj(double wObj[7]);
void console_update_tool(double tool[7]);
void console_update_inertia(double inertia[7]);
void console_update_speed(double spd[2]);
void console_flash();

KEY_CMDS console_get_key();
void console_clear_key(KEY_CMDS last_key);

void console_close();
