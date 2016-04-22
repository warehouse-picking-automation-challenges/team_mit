#include "teleop_console.hpp"

WINDOW *w;

void console_init()
{
  // Open up the ncurses screen
  w = initscr();
  nodelay(w, TRUE);
  noecho();
  curs_set(0);
  
  start_color();
  use_default_colors();

  // Setup the coloring scheme
  init_pair(BKG_COLOR, COLOR_WHITE, COLOR_BLACK);
  init_pair(KEYS_COLOR, COLOR_BLACK, COLOR_YELLOW);
  init_pair(ENABLE_COLOR, COLOR_WHITE, COLOR_RED);
  init_pair(DISABLE_COLOR, COLOR_WHITE, COLOR_BLUE);
  init_pair(DATA_COLOR, COLOR_WHITE, COLOR_CYAN);

  // Clear Screen
  attron(COLOR_PAIR(BKG_COLOR));
  for (int i=0; i<24;i++)
    for (int j=0; j<80;j++)
	mvprintw(i,j," ");

  // Make the title
  attron(A_STANDOUT);
  mvprintw(0,17,   "MCube - ABB Teleoperation Controller");
  
  // Make all of the data titles
  mvprintw(2,2," Cur WObj ");
  mvprintw(2,13," Cur Tool ");
  mvprintw(2,35,"Cur Speed");
  mvprintw(2,46,"Joint Angs");
  mvprintw(2,57," Cart Pos ");
  mvprintw(2,24," Cur Iner ");
  
  mvprintw(4,2," X:");mvprintw(4,13," X:");mvprintw(4,35,"TCP:");
  mvprintw(5,2," Y:");mvprintw(5,13," Y:");mvprintw(5,35,"ORI:");
  mvprintw(6,2," Z:");mvprintw(6,13," Z:");
  mvprintw(7,2,"Q0:");mvprintw(7,13,"Q0:");
  mvprintw(8,2,"QX:");mvprintw(8,13,"QX:");
  mvprintw(9,2,"QY:");mvprintw(9,13,"QY:");
  mvprintw(10,2,"QZ:");mvprintw(10,13,"QZ:");
  mvprintw(4,46,"J1:");mvprintw(4,57," X:");
  mvprintw(5,46,"J2:");mvprintw(5,57," Y:");
  mvprintw(6,46,"J3:");mvprintw(6,57," Z:");
  mvprintw(7,46,"J4:");mvprintw(7,57,"Q0:");
  mvprintw(8,46,"J5:");mvprintw(8,57,"QX:");
  mvprintw(9,46,"J6:");mvprintw(9,57,"QY:");
  mvprintw(10,57,"QZ:");

  mvprintw(4,24," M:");
  mvprintw(5,24,"CX:");
  mvprintw(6,24,"CY:");
  mvprintw(7,24,"CZ:");
  mvprintw(8,24,"IX:");
  mvprintw(9,24,"IY:");
  mvprintw(10,24,"IZ:");
  
  // Make all of the command titles
  mvprintw(13,2,"Cartesian Moves ");
  mvprintw(13,20,"   Joint Moves  ");
  mvprintw(13,38,"        Other Commands        ");

  mvprintw(15,2,"Cmd");
  mvprintw(15,6,"Key");
  mvprintw(15,11,"Cmd");
  mvprintw(15,15,"Key");
  mvprintw(15,20,"Cmd");
  mvprintw(15,24,"Key");
  mvprintw(15,29,"Cmd");
  mvprintw(15,33,"Key");
  mvprintw(15,38," Command  ");
  mvprintw(15,49,"Key");
  mvprintw(15,54," Command  ");
  mvprintw(15,65,"Key");
  
  mvprintw(17,2,"-X ");mvprintw(17,11,"-rX");mvprintw(17,20,"-J1");mvprintw(17,29,"+J1");
  mvprintw(18,2,"-Y ");mvprintw(18,11,"-rY");mvprintw(18,20,"-J2");mvprintw(18,29,"+J2");
  mvprintw(19,2,"-Z ");mvprintw(19,11,"-rZ");mvprintw(19,20,"-J3");mvprintw(19,29,"+J3");
  mvprintw(20,2,"+X ");mvprintw(20,11,"+rX");mvprintw(20,20,"-J4");mvprintw(20,29,"+J4");
  mvprintw(21,2,"+Y ");mvprintw(21,11,"+rY");mvprintw(21,20,"-J5");mvprintw(21,29,"+J5");
  mvprintw(22,2,"+Z ");mvprintw(22,11,"+rZ");mvprintw(22,20,"-J6");mvprintw(22,29,"+J6");

  mvprintw(17,54,"Coarse    ");
  mvprintw(17,38,"Fine      ");
  mvprintw(18,38,"-TCP      ");
  mvprintw(18,54,"+TCP      ");
  mvprintw(19,38,"-ORI      ");
  mvprintw(19,54,"+ORI      ");
  mvprintw(21,38,"Enable    ");
  mvprintw(21,54,"Disable   ");
  mvprintw(22,38,"Align     ");
  mvprintw(22,54,"Quit      ");
  
  // Draw all of the keys
  attroff(A_STANDOUT);
  attron(COLOR_PAIR(KEYS_COLOR));
  for (int i=0; i < NUM_KEY_CMDS; i++)
  {
    if (i != NO_KEY && i != UNKNOWN_KEY)
      mvprintw(key_lib[i].row, key_lib[i].col, key_lib[i].text);
  }

  // Draw disable
  console_update_enabled(false);
  console_update_motion_mode(10);

  // Done, so actually show all of our work!
  refresh();
}

void console_update_cartesian(double cart[7])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<7; i++)
  {
    mvprintw(4+i,60, "       ");
    mvprintw(4+i,60, "%+7.2f", cart[i]);
  }
  refresh();
}

void console_update_joints(double joints[6])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<6; i++)
  {
    mvprintw(4+i,49, "       ");
    mvprintw(4+i,49, "%+7.2f", joints[i]);
  }
  refresh();
}

void console_update_enabled(bool enabled)
{
  if (enabled)
  {
    attrset(A_NORMAL | COLOR_PAIR(ENABLE_COLOR));
    mvprintw(0,3, " ENABLED  ");
  }
  else
  {
    attrset(A_NORMAL | COLOR_PAIR(DISABLE_COLOR));
    mvprintw(0,3, " DISABLED ");
  }
  refresh();
}

void console_update_motion_mode(double curLinStep)
{
  attrset(A_NORMAL | COLOR_PAIR(ENABLE_COLOR));
  mvprintw(0,59,"CURRENT STEP(%d mm)",(int)curLinStep);
  refresh();
}

void console_update_workObj(double wObj[7])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<7; i++)
  {
    mvprintw(4+i,5, "       ");
    mvprintw(4+i,5, "%+7.2f", wObj[i]);
  }
  refresh();
}

void console_update_tool(double tool[7])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<7; i++)
  {
    mvprintw(4+i,16, "       ");
    mvprintw(4+i,16, "%+7.2f", tool[i]);
  }
  refresh();
}

void console_update_inertia(double inertia[7])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<7; i++)
  {
    mvprintw(4+i,27, "       ");
    mvprintw(4+i,27, "%+7.2f", inertia[i]);
  }
  refresh();
}

void console_update_speed(double spd[2])
{
  attrset(COLOR_PAIR(DATA_COLOR));
  for (int i=0; i<2; i++)
  {
    mvprintw(4+i,38, "       ");
    mvprintw(4+i,38, "%+7.2f", spd[i]);
  }
  refresh();
}

void console_flash()
{
  attrset(A_NORMAL | COLOR_PAIR(BKG_COLOR));
  mvprintw(0,21,   "MCube - ABB Teleoperation Controller");
  refresh();
  usleep(250000);
  attrset(A_STANDOUT | COLOR_PAIR(BKG_COLOR));
  mvprintw(0,21,   "MCube - ABB Teleoperation Controller");
  refresh();
}

KEY_CMDS console_get_key()
{
  // First, find out what key (if any) has been pressed
  KEY_CMDS key = UNKNOWN_KEY;
  char c;
  cbreak();
  c = getch();
  for (int i=0; i<NUM_KEY_CMDS; i++)
  {
    if (c == key_lib[i].key)
    {
      key = (KEY_CMDS)i;
      break;
    }
  }

  // Now, if this is a valid key, highlight it
  if (key != NO_KEY && key != UNKNOWN_KEY)
  {
    attrset(A_STANDOUT | COLOR_PAIR(KEYS_COLOR));
    mvprintw(key_lib[key].row, key_lib[key].col, key_lib[key].text);
    refresh();
  }

  // Flush all new key input so our program doesn't get confused
  flushinp();

  return key;
}

void console_clear_key(KEY_CMDS last_key)
{
  if (last_key != NO_KEY && last_key != UNKNOWN_KEY)
  {
    attrset(A_NORMAL | COLOR_PAIR(KEYS_COLOR));
    mvprintw(key_lib[last_key].row, 
             key_lib[last_key].col, 
             key_lib[last_key].text);
    refresh();
  }
}

void console_close()
{
  endwin();
}
