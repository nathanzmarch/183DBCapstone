/*
 * File:          force.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
/*
 * You may want to add macros here.
 */
#define SPEED 4
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  WbDeviceTag force;

  wb_robot_init();

/* get a handle to the sensor and activate it */
  force = wb_robot_get_device("force");
  wb_touch_sensor_enable(force, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  // left_motor = wb_robot_get_device("left wheel motor");
  // right_motor = wb_robot_get_device("right wheel motor");
  // wb_motor_set_position(left_motor, INFINITY);
  // wb_motor_set_position(right_motor, INFINITY);
  // wb_motor_set_velocity(left_motor, 0.0);
  // wb_motor_set_velocity(right_motor, 0.0);
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     const double force_value = wb_touch_sensor_get_value(force);

    if (force_value > -0.01) {
      // printf("Detecting a collision of %g N\n", force_value);
      char s[50] = {0};
      sprintf(s, "%f", force_value);
      wb_robot_set_custom_data(s);
      // movement_counter = 15;
    }
    /*
     * We use the movement_counter to manage the movements of the robot. When
     * the value is 0 we move straight, then when there is another value this
     * means that we are avoiding an obstacle. For avoiding we first move
     * backward for some cycles and then we turn on ourself.
     */
    // if (movement_counter == 0) {
      // left_speed = SPEED;
      // right_speed = SPEED;

    // } else if (movement_counter >= 7) {
      // left_speed = -SPEED;
      // right_speed = -SPEED;
      // movement_counter--;
    // } else {
      // left_speed = -SPEED / 2;
      // right_speed = SPEED;
      // movement_counter--;
    // }

    // /* set the motor speeds. */
    // wb_motor_set_velocity(left_motor, left_speed);
    // wb_motor_set_velocity(right_motor, right_speed);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}