// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "sra_board.h"
// #include "tuning_http_server.h"

// #define MODE NORMAL_MODE
// #define BLACK_MARGIN 4095
// #define WHITE_MARGIN 0
// #define bound_LSA_LOW 0
// #define bound_LSA_HIGH 1000
// #define BLACK_BOUNDARY  950    // Boundary value to distinguish between black and white readings

// /*
//  * weights given to respective line sensor
//  */
// const int weights[5] = {-5, -3, 1, 3, 5};

// /*
//  * Motor value bounds
//  */
// int optimum_duty_cycle = 60;
// int lower_duty_cycle = 45;
// int higher_duty_cycle = 65;
// float left_duty_cycle = 0, right_duty_cycle = 0;

// /*
//  * Line Following PID Variables
//  */
// float error=0, prev_error=0, difference, cumulative_error, correction;

// /*
//  * Union containing line sensor readings
//  */
// line_sensor_array line_sensor_readings;

// void calculate_correction()
// {
//     error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
//     difference = error - prev_error;
//     cumulative_error += error;

//     cumulative_error = bound(cumulative_error, -30, 30);

//     correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
//     prev_error = error;
// }

// void calculate_error()
// {
//     int all_black_flag = 1; // assuming initially all black condition
//     float weighted_sum = 0, sum = 0; 
//     float pos = 0; int k = 0;

//     for(int i = 0; i < 5; i++)
//     {
//         if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
//         {
//             all_black_flag = 0;
//         }
//         if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
//         {
//             k = 1;
//         }
//         if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
//         {
//             k = 0;
//         }
//         weighted_sum += (float)(weights[i]) * k;
//         sum = sum + k;
//     }

//     if(sum != 0) // sum can never be 0 but just for safety purposes
//     {
//         pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
//     }

//     if(all_black_flag == 1)  // If all black then we check for previous error to assign current error.
//     {
//         if(prev_error > 0)
//         {
//             error = 2.5;
//         }
//         else
//         {
//             error = -2.5;
//         }
//     }
//     else
//     {
//         error = pos;
//     }
// }

// void line_follow_task(void* arg)
// {
//     motor_handle_t motor_a_0, motor_a_1;
//     ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
//     ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
//     adc_handle_t line_sensor;
//     ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
//     ESP_ERROR_CHECK(enable_bar_graph());
// #ifdef CONFIG_ENABLE_OLED
//     // Initialising the OLED
//     ESP_ERROR_CHECK(init_oled());
//     vTaskDelay(100);

//     // Clearing the screen
//     lv_obj_clean(lv_scr_act());

// #endif

//     while(true)
//     {
//         line_sensor_readings = read_line_sensor(line_sensor);
//         for(int i = 0; i < 5; i++)
//         {
//             line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
//             line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
//             line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
//         }

//         calculate_error();
//         calculate_correction();

//         left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
//         right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        

        
//         if(left_duty_cycle > right_duty_cycle){
//         set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
//         set_motor_speed(motor_a_1, MOTOR_STOP, 0);
//         }
//         else if(right_duty_cycle > left_duty_cycle){
//         set_motor_speed(motor_a_0, MOTOR_STOP, 0);
//         set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
//         }


//         //ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
//         ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
// #ifdef CONFIG_ENABLE_OLED
//         // Diplaying kp, ki, kd values on OLED 
//         if (read_pid_const().val_changed)
//         {
//             display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
//             reset_val_changed_pid_const();
//         }
// #endif

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }

//     vTaskDelete(NULL);
// }

// void app_main()
// {
//     xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
//     start_tuning_http_server();
// }


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY 950

const int weights[5] = {-5, -3, 1, 3, 5};

// Motor value bounds
int optimum_duty_cycle = 60;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;

// Line Following PID Variables
float error = 0, prev_error = 0, difference, cumulative_error, correction;

// Binary history arrays for 0th and 4th sensors
int sensor_0_history[500] = {0};
int sensor_4_history[500] = {0};
int history_index = 0;
int threshold = 0.6;
int all_black_flag = 0;
// Line Sensor Readings
line_sensor_array line_sensor_readings;

void store_sensor_history()
{
    // Store binary values indicating black (1) or white (0) for sensors 0 and 4
    sensor_0_history[history_index] = (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY) ? 1 : 0;
    sensor_4_history[history_index] = (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY) ? 1 : 0;
    history_index = (history_index + 1) % 500; // Circular buffer
}

float calculate_average(int sensor_history[])
{
    int sum = 0;
    for (int i = 0; i < 500; i++)
    {
        sum += sensor_history[i];
    }
    return (float)sum / 500; // Result represents the proportion of black readings
}

void calculate_correction()
{
    error = error * 10; // Scale error for duty cycle range
    difference = error - prev_error;
    cumulative_error += error;
    
    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    all_black_flag = 1;
    float weighted_sum = 0, sum = 0;
    float pos = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            all_black_flag = 0;
        }
        int k = (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY) ? 1 : 0;
        weighted_sum += (float)(weights[i]) * k;
        sum += k;
    }

    if (sum != 0)
    {
        pos = (weighted_sum - 1) / sum;
    }

    if (all_black_flag == 1) // Handle all black scenario
    {
        // Check previous error direction
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

void line_follow_task(void* arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());

#ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);
    lv_obj_clean(lv_scr_act());
#endif

    while (true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for (int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

        calculate_error();
        calculate_correction();
        store_sensor_history();

        float leftavg = calculate_average(sensor_0_history);
        float rightavg = calculate_average(sensor_4_history);

        int duty_cycle = left_duty_cycle > right_duty_cycle? left_duty_cycle: right_duty_cycle;
        if(all_black_flag == 1){
            if(leftavg > threshold && rightavg < threshold){ // left turn
                set_motor_speed(motor_a_0, MOTOR_FORWARD, duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, duty_cycle);
            }
        }
        else{
            if(left_duty_cycle > right_duty_cycle){
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, right_duty_cycle);
            }
            else if(right_duty_cycle > left_duty_cycle){
                set_motor_speed(motor_a_0, MOTOR_BACKWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            }
        }
        // if (error > 2 || error < -2) // Large error indicates a turn
        // {

        // }
        // else
        // {
        //     // Regular line following with PID correction
        //     left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        //     right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        // }



        ESP_LOGI("debug", "leftavg: %f, rightavg: %f",leftavg, rightavg);
#ifdef CONFIG_ENABLE_OLED
        if (read_pid_const().val_changed)
        {
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
            reset_val_changed_pid_const();
        }
#endif

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
