// https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// https://github.com/micro-ROS/micro_ros_platformio

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <two_wheels_interfaces/msg/motor.h>
#include <two_wheels_interfaces/msg/motors_odom.h>
#include "Motor.h"
#include "FourPinStepperMotor.h"
// #include "DCMotor.h"
// #include "DCMotorConfig.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t velocityCommandSubscriber;
two_wheels_interfaces__msg__MotorsOdom velocityCommandCallbackMessage;
rclc_executor_t subscriberExecutor;

rcl_publisher_t odomStatePublisher;
rclc_executor_t publisherExecutor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publisherTimer;
const unsigned int PUBLISHER_TIMER_TIMEOUT_MILL = 100;
// const unsigned int ENC_COUNT_REV = 105; // Motor encoder output pulses per 360 degree revolution (measured manually)
// DCMotorConfig motorConfig(ENC_COUNT_REV, 1.2, 1.0, 0.1);
// DCMotor dcMotor1(motorConfig,
//          16,
//          4,
//          0,
//          23,
//          22,
//          true);
// DCMotor dcMotor2(motorConfig,
//          27,
//          14,
//          12,
//          36,
//          39,
//          false);

const int front_left_1 = 15;
const int front_left_2 = 2;
const int front_left_3 = 0;
const int front_left_4 = 4;

const int rear_left_1 = 19;
const int rear_left_2 = 18;
const int rear_left_3 = 5;
const int rear_left_4 = 17;


const int front_right_1 = 13;
const int front_right_2 = 12;
const int front_right_3 = 14;
const int front_right_4 = 27;

const int rear_right_1 = 26;
const int rear_right_2 = 25;
const int rear_right_3 = 33;
const int rear_right_4 = 32;

FourPinStepperMotor front_left(front_left_1, front_left_2, front_left_3, front_left_4, true);
FourPinStepperMotor rear_left(rear_left_1, rear_left_2, rear_left_3, rear_left_4, true);

FourPinStepperMotor front_right(front_right_1, front_right_2, front_right_3, front_right_4, false);
FourPinStepperMotor rear_right(rear_right_1, rear_right_2, rear_right_3, rear_right_4, false);

// https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
TaskHandle_t moveMotorsTask;


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
// void error_loop() {
//     while (1) {
//         delay(100);
//     }
// }

void velocityCommandCallback(const void *msgin) {
    const two_wheels_interfaces__msg__MotorsOdom *command = (const two_wheels_interfaces__msg__MotorsOdom *) msgin;
    front_left.setVelocity(command->front_left.velocity);
    front_right.setVelocity(command->front_right.velocity);

    rear_left.setVelocity(command->rear_left.velocity);
    rear_right.setVelocity(command->rear_right.velocity);
}

void odomStateTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        two_wheels_interfaces__msg__MotorsOdom msg;

        msg.front_left.position = front_left.getPosition();
        msg.front_left.velocity = front_left.getAngularVelocity();

        msg.rear_left.position = rear_left.getPosition();
        msg.rear_left.velocity = rear_left.getAngularVelocity();

        msg.front_right.position = front_right.getPosition();
        msg.front_right.velocity = front_right.getAngularVelocity();

        msg.rear_right.position = rear_right.getPosition();
        msg.rear_right.velocity = rear_right.getAngularVelocity();

        RCSOFTCHECK(rcl_publish(&odomStatePublisher, &msg, NULL));
    }
}

void createStatePublisher() {

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(two_wheels_interfaces, msg,
                                                                                    MotorsOdom);


    RCSOFTCHECK(rclc_publisher_init_default(
            &odomStatePublisher,
            &node,
            type_support,
            "two_wheels/motors_state"));

    // create timer
    RCSOFTCHECK(rclc_timer_init_default(
            &publisherTimer,
            &support,
            RCL_MS_TO_NS(PUBLISHER_TIMER_TIMEOUT_MILL),
            odomStateTimerCallback));


    // create executor
    RCSOFTCHECK(rclc_executor_init(&publisherExecutor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_timer(&publisherExecutor, &publisherTimer));
}

void createCommandSubscriber() {
    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(two_wheels_interfaces, msg,
                                                                                    MotorsOdom);

    RCSOFTCHECK(rclc_subscription_init_default(
            &velocityCommandSubscriber,
            &node,
            type_support,
            "two_wheels/motors_cmd"));


    RCSOFTCHECK(rclc_executor_init(&subscriberExecutor, &support.context, 1, &allocator));

    RCSOFTCHECK(rclc_executor_add_subscription(&subscriberExecutor,
                                               &velocityCommandSubscriber,
                                               &velocityCommandCallbackMessage,
                                               &velocityCommandCallback,
                                               ON_NEW_DATA));
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_two_wheels_motors", "", &support));

    createStatePublisher();

    createCommandSubscriber();

    xTaskCreatePinnedToCore(
            moveMotors,   /* Task function. */
            "moveMotorsTask",     /* name of task. */
            10000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            0,           /* priority of the task */
            &moveMotorsTask,      /* Task handle to keep track of created task */
            0);          /* pin task to core 1 */

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&publisherExecutor, RCL_MS_TO_NS(50)));
    RCSOFTCHECK(rclc_executor_spin_some(&subscriberExecutor, RCL_MS_TO_NS(50)));
}

void moveMotors(void *pvParameters) {
    for (;;) {
        front_left.move();
        front_right.move();
        rear_left.move();
        rear_right.move();
    }
}