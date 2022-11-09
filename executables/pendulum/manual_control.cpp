#include <atomic>
#include <array>
#include <chrono>
#include <curses.h>
#include <mutex>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <utility>

std::atomic<uint16_t> command;
std::atomic<int> new_value;
std::atomic<int> cmd_value;
std::array<std::string, 3> send_status = { "NA", "New Value", "Sent" };
std::array<std::string, 3> cmds_str = { "Right", "Left", "ZERO" };
std::atomic<int> command_str_ix;
std::atomic<int> key_code;
bool finish = false;

std::mutex command_mutex;
void get_key()
{
  int y = 0;
  int x = 0;
  key_code = 0;
  while (key_code != 'q')
  {
    // timeout(-1);
    // endwin();

    switch (key_code)
    {
      case KEY_RIGHT:

        command = 0xff00;
        command_str_ix = 0;
        new_value = 1;
        break;
      case KEY_LEFT:
        command = 0x00ff;
        command_str_ix = 1;
        // command_str = "Left";
        new_value = 1;
        break;
      default:
        command = 0x0000;
        command_str_ix = 2;
        // command_str = "ZERO";
        new_value = 0;
        // }
    }

    // printw(command_str.c_str());

    // printw("key_code = %i - ( %i, %i)", key_code, KEY_RIGHT, KEY_LEFT);
    // printw(command_str.c_str());
  }
  finish = true;
}

void send()
{
  while (!finish)
  {
    if (new_value == 1)
    {
      // printw("\tDONE");
      // printw("key_code = %d\n", command);
      // command = 0;
      new_value = 2;
    }
  }
}

void ctrl_window()
{
  int y = 0;
  int x = 0;

  auto window = initscr();
  keypad(window, true);
  printw("Use Right/Left arrow keys\n");
  refresh();

  while (!finish)
  {
    key_code = getch();

    // if (new_value > 0)
    // {
    getyx(stdscr, y, x);
    move(y, 0);  // move to begining of line
    clrtoeol();
    // }
    printw("%s %s", cmds_str[command_str_ix].c_str(), send_status[new_value].c_str());
  }
  endwin();
}

int main(int argc, char* argv[])
{
  new_value = 0;
  command_str_ix = 2;

  std::thread th_poll(get_key);
  std::thread th_send(send);
  std::thread th_window(ctrl_window);

  th_poll.join();
  th_send.join();
  th_window.join();

  return 0;
}
