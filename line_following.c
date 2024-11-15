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
    #define BLACK_BOUNDARY  950    // Boundary value to distinguish between black and white readings

    /*
    * weights given to respective line sensor
    */
    const int weights[5] = {-5, -3, 1, 3, 5};

    /*
    * Motor value boundts
    */
    int optimum_duty_cycle = 57;
    int lower_duty_cycle = 45;
    int higher_duty_cycle = 65;
    float left_duty_cycle = 0, right_duty_cycle = 0;
    int left_turn_flag =0;
    int right_turn_flag=0;
    int right_straight_turn_flag=0;
    // int t_flag=0;   
    int u_turn_flag=0;   
    char[2] prev_turns  = {'',''}   ; 
    bool inverted_line = (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY);
     /*
    * Line Following PID Variables
    */
    float error=0, prev_error=0, difference, cumulative_error, correction;

    /*
    * Union containing line sensor readings
    */
    line_sensor_array line_sensor_readings;

    void calculate_correction()
    {
       // error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
        difference = error - prev_error;
        cumulative_error += error;

        cumulative_error = bound(cumulative_error, -30, 30);

        correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
        prev_error = error;
    }

    int calculate_error()
    {
        int all_black_flag = 1; // assuming initially all black condition
        float weighted_sum = 0, sum = 0; 
        float pos = 0; 
        int k = 0;
        bool right_turn = (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[4] < BLACK_BOUNDARY);
        bool left_turn = (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY) && (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY);
        bool left_turn_flag =0;
        bool right_turn_flag=0;
        bool reverse_turn_flag=0; 
        for(int i = 0; i < 5; i++)
        {
            if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
            {
                all_black_flag = 0;
            }
            if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
            {
                k = 1;
            }
            if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
            {
                k = 0;
            }
        
            weighted_sum += (float)(weights[i]) * k;
            sum = sum + k;
        }
        if( (left_turn == 1 && right_rturn == 0)  ) //left and straight   
        {
            left_turn_flag = 1;
        }
        if(left_turn == 0 && right_turn == 1 ) //right and straight
        {
            right_turn_flag = 1;
        }
        if(left_turn == 0 && right_turn == 0) //straight
        {
            right_turn_flag = 0;
            left_turn_flag = 0;
        }
        if(left_turn == 1 && right_turn == 1){
            reverse_turn_flag = 1;
        }
        if(all_black_flag == 1){
            reverse_turn_flag = 1;
        } 
        else{
            reverse_turn_flag = 0; 
        }

        if(sum != 0) // sum can never be 0 but just for safety purposes
        {
            pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
        }
        if(all_black_flag == 1)  // If all black then we check for previous error to assign current error.
        {
            if(prev_error > 0)
            {
                error = 10;
            }
            else
            {
                error = -10;
            }
        }
        {
            error = pos;
        }
        return left_turn_flag, right_turn_flag, reverse_turn_flag
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
        // Initialising the OLED
        ESP_ERROR_CHECK(init_oled());
        vTaskDelay(100);

        // Clearing the screen
        lv_obj_clean(lv_scr_act());

    #endif

        while(true)
        {
            line_sensor_readings = read_line_sensor(line_sensor);
            for(int i = 0; i < 5; i++)
            {
                line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
                line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
            }

            ltf, rtf, rvtf = calculate_error();
            calculate_correction();

            left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
      
            if(left_turn_flag == 1 && right_turn_flag  == 0){
                set_motor_speed(motor_a_0, MOTOR_BACKWARD, higher_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, higher_duty_cycle);
                
            //    vTaskDelay(20);
            }
            else if(right_turn_flag  == 1 && left_turn_flag == 0 ){
                set_motor_speed(motor_a_0, MOTOR_FORWARD, higher_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, higher_duty_cycle);
                // vTaskDelay(20);
            }
            else if(reverse ) {
            // Execute a U-turn by rotating both motors in opposite directions
                set_motor_speed(motor_a_0, MOTOR_FORWARD, higher_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, higher_duty_cycle);
                // vTaskDelay(20);
            } 
            else if(inverted_line){
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
               
            else{
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
                // vTaskDelay(20);
            }

            //ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
           // ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
    #ifdef CONFIG_ENABLE_OLED
            // Diplaying kp, ki, kd values on OLED 
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