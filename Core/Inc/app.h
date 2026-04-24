#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

void setup_ros_and_app(void);
void loop_ros_task(void);
void loop_can_task(void);
void on_led_subscription_callback(const void *msgin);
void pid_control_loop(void const *argument);

#ifdef __cplusplus
}
#endif

#endif // APP_H
