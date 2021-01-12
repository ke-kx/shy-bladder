/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "driver/gpio.h"

//TODO: Update with real values
#define SERVO_0_PIN 33 
#define SERVO_1_PIN 27
#define SERVO_2_PIN 35
#define SCAN_STEPS 18

#define TOLLERANCE_IN_MM 200

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 400 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond


#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define ECHO_TRIGGER_PIN 32
#define ECHO_RECEIVER_PIN 25
#define ESP_INTR_FLAG_DEFAULT 0

// #define ECHO_RECEIVER_MIN 0
// #define ECHO_RECEIVER_MAX 72000
#define MAX_DISTANCE 4000

static xQueueHandle gpio_evt_queue = NULL;
uint64_t timer = 0;
uint32_t last_distance = 0;

uint32_t current_angle_0 = 0;
uint32_t current_angle_1 = 0;
uint32_t current_angle_2 = 0;
bool is_second = false;

static uint32_t get_high_pulse_duty_width_for_degree(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

// get distance in millimetrs
static uint64_t calc_distance(uint64_t raw_value)
{
    // input in microseconds timing difference * 340 (speed of sound in meter per seconds) / 2 (back and forth) / 1000 (unitswitch meters -> mm, seconds -> us)
    float calculation = raw_value * 340.0 / 2 / 1000 ;
    printf("estimation:%f\n", calculation);
    return round(calculation);
}

static uint64_t get_degree_from_distance(uint64_t distance)
{
    uint64_t ret = round((1.0 * SERVO_MAX_DEGREE / MAX_DISTANCE) * distance);
    printf("get_degree output: %lli\n",ret);
    return ret;
}

static uint32_t scale_degree(uint32_t min, uint32_t max, uint32_t raw_degree)
{
    if(raw_degree<min) return 0;
    if(raw_degree>max) return SERVO_MAX_DEGREE;
    uint64_t ret =  round((raw_degree - min)*1.0 * SERVO_MAX_DEGREE/(max - min));
    printf("scale_degree output: %lli\n",ret);
    return ret;
}
/*
static void update_distance_from_queue(void* arg)
{
    uint64_t difference;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &difference, portMAX_DELAY)) {

            if (difference < ((2 * 8) / 340.0) * 1000 * 1000 ) {
                printf("difference between interrupts: %lli\n", difference);
                //get_distance(difference);

                uint64_t calculated_angle = scale_degree(0,45, get_degree(difference));

                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(calculated_angle));
            }else{
                printf("ignored value with diff in us %lli\n", difference);
            }
        }
    }
}
*/

static void IRAM_ATTR gpio_event_handler(void* arg)
{
    // uint32_t gpio_num = (uint32_t) arg;

    if (is_second) {
        uint64_t diff = esp_timer_get_time() - timer;
        // 4 meter * 2 / 340 m/s / factor für sekune auf us
        xQueueSendFromISR(gpio_evt_queue, &diff, NULL);
        if (diff > ((2 * 8) / 340.0) * 1000 * 1000 ) {
            is_second = !is_second;
        }
    }

    timer = esp_timer_get_time();
    is_second = !is_second;
}


static void mcpwm_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_0_PIN);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, SERVO_1_PIN);    //Set GPIO 18 as PWM0A, to which servo is connected

        //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(0));
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(0));

}

static void move_servo_0(uint32_t degree){
    while(current_angle_0 != degree)
    {
        current_angle_0 = current_angle_0 + ( -1 * current_angle_0 > degree);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(current_angle_0));
    }
}
static void slow_move_servo_1(uint32_t degree){
    // printf("current angle: %i\n", current_angle_0);
    // printf("target angle: %i\n", degree);
    int move_direction = 1;
    uint32_t degree_to_move = 0;
    if(current_angle_0 > degree){
        move_direction = -1;
        degree_to_move = current_angle_0 - degree;
    }else{
        move_direction = 1;
        degree_to_move = degree - current_angle_0;
    }

    // printf("move_direction: %i\n", move_direction);
    // printf("degree to move: %i\n", degree_to_move);

    for(uint32_t i=0 ;  i <= degree_to_move; i++)
    {
        uint32_t angle = current_angle_0 + (i * 1 *  move_direction);
        // printf("i: %i current angle: %i\n", i, angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(angle));
        vTaskDelay(1);
    }
    current_angle_0 = degree;
}
static void move_servo_1(uint32_t degree){
    uint32_t degree_to_move = 0;
    if(current_angle_0 > degree){
        degree_to_move = current_angle_0 - degree;
    }else{
        degree_to_move = degree - current_angle_0;
    }
    current_angle_0 = degree;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, get_high_pulse_duty_width_for_degree(current_angle_0));
    vTaskDelay(degree_to_move * 1);
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t get_high_pulse_duty_width2_for_degree(uint32_t degree_of_rotation, uint32_t min_offset, uint32_t max_offset)
{
    uint32_t cal_pulsewidth = 0;
    uint32_t min = SERVO_MIN_PULSEWIDTH - min_offset;
    uint32_t max = SERVO_MAX_PULSEWIDTH + max_offset;
    cal_pulsewidth = (min + (((max - min) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

static void init_echo(uint32_t trigger_pin, uint32_t receiver_pin)
{
    uint64_t trigger_pin_mask = (1ULL<<trigger_pin);
    uint32_t receiver_pin_mask = (1ULL<<receiver_pin);
    
    printf("config output\n");
    // config output / trigger pin
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    printf("trigger_pin_mask %lli\n",trigger_pin_mask);
    io_conf.pin_bit_mask = trigger_pin_mask;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    printf("config input\n");
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    printf("receiver pin mask %i\n",receiver_pin_mask);
    io_conf.pin_bit_mask = receiver_pin_mask;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    printf("init Queue\n");
    gpio_evt_queue = xQueueCreate(10, sizeof(uint64_t));
    // xTaskCreate(gpio_task_exampleddjk, "gpio_task_example\n", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(receiver_pin, gpio_event_handler, (void*) receiver_pin);

}

static uint64_t get_distance_reading()
{
    gpio_set_level(ECHO_TRIGGER_PIN, 1);
    vTaskDelay(1 / portTICK_RATE_MS);
    //printf("set_low\n");        
    gpio_set_level(ECHO_TRIGGER_PIN, 0);
    // vTaskDelay(100 / portTICK_RATE_MS);
    uint64_t difference;
    while(1){
        if(xQueueReceive(gpio_evt_queue, &difference, portMAX_DELAY)){
            // printf("measured distance: %lli\n", calc_distance(difference));
            difference =  calc_distance(difference);
        }
        if(difference > MAX_DISTANCE){
            gpio_set_level(ECHO_TRIGGER_PIN, 1);
            vTaskDelay(1 / portTICK_RATE_MS);
            //printf("set_low\n");        
            gpio_set_level(ECHO_TRIGGER_PIN, 0);
        }else{
            return difference;
        }
        // TODO check
    }
}
uint64_t scan_values[SCAN_STEPS] = {};
static void start_scan(){
    printf("Starting Scan\n");
    for(int i = 0 ; i < SCAN_STEPS; i++){
        uint64_t scan_angle = round(1.0 * i * SERVO_MAX_DEGREE / SCAN_STEPS);
        move_servo_1(scan_angle);
        vTaskDelay(99);

        printf("Scanning in Angle %lli\n", scan_angle);
        uint64_t distance = get_distance_reading();
        scan_values[i] = distance;
        // for (int j = 0 ; j < 2 ; j++) {
        //     get_distance_reading();
        //     vTaskDelay(10);
        // }

        vTaskDelay(1);
    }
    printf("Scann completed\n");
}
static bool out_of_bound(uint64_t sector_id, uint64_t distance){
    uint64_t remebered_sector_value = scan_values[sector_id];
    scan_values[sector_id] = round((remebered_sector_value + distance) / 2);
    printf("update %lli\n old: %lli\n new: %lli\n",sector_id, remebered_sector_value, scan_values[sector_id]);
    if(distance < remebered_sector_value + TOLLERANCE_IN_MM || distance > remebered_sector_value - TOLLERANCE_IN_MM){
        return false;
    }
    uint64_t angle = round(1.0 * sector_id * ( SERVO_MAX_DEGREE / SCAN_STEPS ));
    printf("Out of bound\nAngle: %lli\nOlddistance: %lli\nMeasured: %lli\n", angle ,remebered_sector_value, distance);
    return true;
}
static void surveilance(){
    printf("Starting Surveilance\n");
    uint64_t scan_stepsize = round(SERVO_MAX_DEGREE/SCAN_STEPS);
    while(1)
    {
        for(int i = 0 ; i < SERVO_MAX_DEGREE; i++){
            move_servo_1(i);
            vTaskDelay(3);
            if(i % scan_stepsize == 0)
            {
                uint64_t distance = get_distance_reading();
                uint64_t sector_id = round(i/scan_stepsize);
                if(out_of_bound(sector_id,distance))
                {
                    // printf("Out of bound\nAngle: %lli\nOlddistance: %lli\nMeasured: %lli\n");
                    vTaskDelay(200);
                    // return;
                }
            }
        }
        // for(int i = SERVO_MAX_DEGREE-1 ; i == 0; i--){
        //     move_servo_1(i);
        //     vTaskDelay(5);
        //     if(i % scan_stepsize == 0)
        //     {
        //         uint64_t distance = get_distance_reading();
        //         if(out_of_bound(round(i/scan_stepsize),distance))
        //         {
        //             // printf("Out of bound\nAngle: %lli\nOlddistance: %lli\nMeasured: %lli\n");
        //             vTaskDelay(200);
        //             // return;
        //         }
        //     }
        // }
        printf("Surveilance completed\n");
        move_servo_1(0);
        vTaskDelay(10);
    }
    
}

void peeRoutine(void *args)
{
    mcpwm_gpio_initialize();
    init_echo(ECHO_TRIGGER_PIN,ECHO_RECEIVER_PIN);

    vTaskDelay(10);
    for(;;){

        start_scan();
        vTaskDelay(10);
        surveilance();

        vTaskDelay(200);

    }
}
void app_main(void)
{
    printf("Starting PeeRoutine");
    xTaskCreate(peeRoutine,"PeeRoutine",4096,NULL,5,NULL);
    // xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
}









// /**
//  * @brief Configure MCPWM module
//  */
// void mcpwm_example_servo_control(void *arg)
// {
//     uint32_t angle, count;
//     uint32_t global_count = 0;
//     //1. mcpwm gpio initialization
//     mcpwm_example_gpio_initialize();

//     while (1) {
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
// //            printf("Angle of rotation: %d\n", count);
//             angle = get_high_pulse_duty_width2_for_degree(count, 0, global_count * 50);
// //            angle = count * 20;
//             //printf("angle: %dus\n", angle);

//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle );

//             //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, SERVO_MAX_PULSEWIDTH + global_count * 50 - angle );


//             //printf("total: %dus\n", SERVO_MAX_PULSEWIDTH + angle);
//             vTaskDelay(2);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V

//         }
//         printf("max_angle: %dus\n", angle);

//         angle = get_high_pulse_duty_width2_for_degree(0, global_count * 50, 0);
//         printf("min_angle: %dus\n", angle);
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
// //            printf("Angle of rotation: %d\n", count);
//             angle = get_high_pulse_duty_width2_for_degree(count, global_count * 50, 0);
// //            angle = count * 20;
//             //printf("angle: %dus\n", angle);

//            // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle );

//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, angle );


//             //printf("total: %dus\n", SERVO_MAX_PULSEWIDTH + angle);
//             vTaskDelay(2);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V

//         }
//         global_count++;
//     }
// }
