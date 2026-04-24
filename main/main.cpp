/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// static const char* TAG = "foc";

//******************************** SimpleFOC Configuration //********************************

#define BLDC_MOTOR_PP (7)

#define VOLTAGE_POWER (9.0f)
#define VOLTAGE_LIMIT (6.0f)
#define VOLTAGE_SENSOR_ALIGN (3.0f)
#define DAMPING_MAX_VELOCITY (6.0f * PI)

#define MOTOR_A (GPIO_NUM_25)
#define MOTOR_B (GPIO_NUM_26)
#define MOTOR_C (GPIO_NUM_27)

#define WIRE_SDA (GPIO_NUM_32)
#define WIRE_SCL (GPIO_NUM_33)

#define FOC_MONITOR_BAUD CONFIG_MONITOR_BAUD

#define TASK_STACK_SIZE (4096)

//******************************** UART Configure //********************************

#define UART_TXD (GPIO_NUM_23)
#define UART_RXD (GPIO_NUM_22)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM (UART_NUM_2)
#define UART_BAUD_RATE (921600)
#define UART_TASK_STACK_SIZE (4096)

#define BUF_SIZE (1024)

HardwareSerial Serial2;

//******************************** SimpleFOC Input //********************************

static float g_constant_force;
static float g_damper;

Commander command1 = Commander(Serial);
void doForce1(char* cmd) { command1.scalar(&g_constant_force, cmd); }
void doDamper1(char* cmd) { command1.scalar(&g_damper, cmd); }

Commander command2 = Commander(Serial2);
void doForce2(char* cmd) { command2.scalar(&g_constant_force, cmd); }
void doDamper2(char* cmd) { command2.scalar(&g_damper, cmd); }

//******************************** SimpleFOC Output //********************************

//******************************** SimpleFOC Function //********************************

#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

// magnetic sensor instance - I2C
AS5600 as5600 = AS5600(I2C_NUM_0, WIRE_SCL, WIRE_SDA);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(BLDC_MOTOR_PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C);

static void get_angle_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        float wheel_rad = as5600.getAngle();
        static float wheel_rad_last;
        if (fabsf(wheel_rad_last - wheel_rad) < 0.002f) {
            continue;
        }
        wheel_rad_last = wheel_rad;
        char buffer[32];
        sprintf(buffer, "A%f\n", wheel_rad);
        Serial2.write(buffer, strlen(buffer));
        // ESP_LOGI(TAG, "A%f", wheel_rad);
    }
}

static void foc_loop_task(void* arg) {
    // initialise magnetic sensor hardware
    as5600.init();
    // link the motor to the sensor
    motor.linkSensor(&as5600);

    // power supply voltage
    driver.voltage_power_supply = VOLTAGE_POWER;
    driver.voltage_limit = VOLTAGE_LIMIT;
#ifdef USING_MCPWM
    driver.init(0);
#else
    driver.init({1, 2, 3});
#endif
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set motion control loop to be used
    motor.controller = MotionControlType::torque;
    // set torque control loop to be used
    motor.torque_controller = TorqueControlType::voltage;

    // use monitoring with serial
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.begin(FOC_MONITOR_BAUD);
    Serial2.begin(UART_BAUD_RATE, UART_PORT_NUM, UART_TXD, UART_RXD);

    // comment out if not needed
    vTaskDelay(pdMS_TO_TICKS(100));
    SimpleFOCDebug::enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    command1.add('F', doForce1, const_cast<char*>("Constant Force"));
    command1.add('D', doDamper1, const_cast<char*>("Damper"));

    command2.echo = false;
    command2.verbose = VerboseMode::nothing;
    command2.add('F', doForce2);
    command2.add('D', doDamper2);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));

        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        motor.loopFOC();
    }
}

static void foc_move_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));

        float damper = g_damper;
        float constant_force = g_constant_force;
        float damping = damper * motor.shaft_velocity / DAMPING_MAX_VELOCITY;
        float torque_ratio = constant_force - damping;
        torque_ratio = torque_ratio > 1.0f ? 1.0f : (torque_ratio < -1.0f ? -1.0f : torque_ratio);

        // voltage set point variable
        float target_voltage = VOLTAGE_LIMIT * torque_ratio;

        // Motion control function
        // current_velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_voltage);

        // user communication
        command2.run();
    }
}

void foc_init(void) {
    xTaskCreate(foc_loop_task, "foc_loop_task", TASK_STACK_SIZE, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreate(foc_move_task, "foc_loop_task", TASK_STACK_SIZE, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreate(get_angle_task, "get_angle_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

extern "C" void app_main(void) {
    foc_init();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}