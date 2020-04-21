#include <webots/robot.h>

int main() {
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  if (time_step == 0)
    time_step = 1;
  for (;;)
    wb_robot_step(time_step);
  return 0;
}