#include "interface.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//******************************** Motor Backend //********************************
TaskHandle_t* motor_task_handle;
void (*motor_output)(float* wheel_rad);
void (*motor_init)(void);
extern TaskHandle_t foc_task_handle;
extern void foc_backend_output(float* wheel_rad);
extern void foc_backend_init(void);
//******************************** FFB Backend //********************************
TaskHandle_t* ffb_task_handle;
void (*ffb_output)(float* constant_force, float* damper);
void (*ffb_init)(void);
extern TaskHandle_t uart_write_task_handle;
extern void uart_backend_output(float* constant_force, float* damper);
extern void uart_backend_init(void);
extern TaskHandle_t espnow_write_task_handle;
extern "C" {
extern void espnow_backend_output(float* constant_force, float* damper);
extern void espnow_backend_init(void);
}
//******************************** Interface Function //********************************
void interface_init(void) {
    motor_task_handle = &foc_task_handle;
    motor_output = foc_backend_output;
    motor_init = foc_backend_init;
#if ESPNOW_BACKEND
    ffb_task_handle = &espnow_write_task_handle;
    ffb_output = espnow_backend_output;
    ffb_init = espnow_backend_init;
#else
    ffb_task_handle = &uart_write_task_handle;
    ffb_output = uart_backend_output;
    ffb_init = uart_backend_init;
#endif
}
