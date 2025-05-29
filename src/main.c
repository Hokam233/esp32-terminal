/*
    Projekt: IMP
    Meno riesitela: Jakub Dyrčík xdyrci02
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_bt_defs.h"

//for leds
#define RED_LED_PIN GPIO_NUM_5
#define GREEN_LED_PIN GPIO_NUM_23
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  
#define PWM_FERQUENCY 5000  
#define RED_LED_CHANNEL LEDC_CHANNEL_0
#define GREEN_LED_CHANNEL LEDC_CHANNEL_1
#define MAX_DUTY_CYCLE 255

//for terminal
#define TERMINAL_COL_1 GPIO_NUM_27
#define TERMINAL_COL_2 GPIO_NUM_12
#define TERMINAL_COL_3 GPIO_NUM_17
#define TERMINAL_ROW_1 GPIO_NUM_14
#define TERMINAL_ROW_2 GPIO_NUM_26
#define TERMINAL_ROW_3 GPIO_NUM_25
#define TERMINAL_ROW_4 GPIO_NUM_16

//nvs
#define MAX_PWD_LENGTH 5 //pwd + \0  => pwd can be only 4 digits

//for string handling
#define BUFFER_SIZE 255
#define TO_SEC 1000000

/* Bluetooth */
#define TAG "ESP_BLE_SERVER"
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SVC_INST_ID 0

/* 
 *         global variables
*/

//key map
const char key_map[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};

//password
char pwd[MAX_PWD_LENGTH+1] = {0};
uint8_t pwd_char_value[4] = {0x2f, 0x2f, 0x2f, 0x2f};
//get time for the open
uint32_t open_time = 0;
uint8_t open_time_char_value[5] = {0x2f, 0x2f, 0x2f, 0x2f};
//storage for the string
char input_buffer[BUFFER_SIZE] = {0};
int buffer_position = 0;
//state of the system (if we are open or not)
bool locked = true;
//for leds
typedef struct {
    ledc_channel_t channel;
    uint32_t duty_cycle;
} led_command_t;
//time for led
bool timer_active = false;
esp_timer_handle_t timer_handle;
//queue for interupts
static QueueHandle_t row_queue = NULL;

/* 
 * mostly from tutorial:
    https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server_service_table/tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md
    https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md
 */
//mostly copied from 
enum{
    IDX_SVC,
    PWD_CHAR,
    PWD_VAL,
    TIME_CHAR,
    TIME_VAL,
    GAT_DB,
};

static uint8_t service_uuid[16] = {
    //000000ff-0000-1000-8000-00805f9b34fb
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, 
    .max_interval        = 0x0010, 
    .appearance          = 0x00,
    .manufacturer_len    = 0,    
    .p_manufacturer_data = NULL, 
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

//scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL, 
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
};

static const uint16_t SERVICE_UUID = 0x00ff;
static const uint16_t PASSWORD_CHAR_UUID = 0xff01;
static const uint16_t TIME_CHAR_UUID = 0xff02;
#define CHAR_VAL_LEN_MAX 255
uint16_t handle_table[GAT_DB];
static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const esp_gatts_attr_db_t gatt_db[GAT_DB] ={
    [IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(SERVICE_UUID), (uint8_t *)&SERVICE_UUID}},

    [PWD_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, (sizeof(uint8_t)), (sizeof(uint8_t)), (uint8_t *)&char_prop_read_write_notify}},

    [PWD_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&PASSWORD_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_VAL_LEN_MAX, sizeof(pwd_char_value), (uint8_t *)pwd_char_value}},

    [TIME_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, (sizeof(uint8_t)), (sizeof(uint8_t)), (uint8_t *)&char_prop_write}},

    [TIME_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&TIME_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_VAL_LEN_MAX, sizeof(open_time_char_value), (uint8_t *)open_time_char_value}},
};


/*  
 * @brief delays the task by n ticks
 * @param n number of tick for delay
*/
void delay(int n);

/*
 * @brief inits the pins for leds and channel for the led so they can be more brighter
*/
void init_led();

/*
 * @brief inits the pins for terminal, create a queue for the intterupt, and set up  the interupt service routines
*/
void init_terminal();

/*
 * @brief  inits the storage for password and open time, 
*/
void init_nvs();

/*
 * @brief inits the bluetooth for comunication between the web api and mikrocontroler
*/
void init_ble();

/* 
 * @brief stores the given string into the storage(nvs)
 * @param pwd actuall string to be stored
 * @return 0 on success, 1 on failure
*/
int store_pwd(char *pwd);

/* 
 * @brief store the given intiger into the storage(nvs)
 * @param time pointer to the actuall unsigned intiger
 * @return 0 on success or 1 on failure
*/
int store_time_open(uint32_t *time);

/*
 * @brief retrieves password and opening time from nvs storage. If either is missing or invalid, sets default values(pwd='1111', time='10')
 * @param pwd pointer to a buffer where the retrived password will be stored.
 * @param open_time pointer to a variable where the retrived time will be stored.
 * @param 0 on success or 1 on failure
*/
int get_info(char *pwd, uint32_t *open_time);

/*
 * @brief turns seleced led on and off in a short interval, signalization for user
 * @param led_channel identifaier of the selected led(use defined macros RED/GREEN LED CHANNEL)
 * @param num_blink number of blinks required
*/
void short_blink(int led_channel, int num_blink);

/*
 * @brief turns seleced led on
 * @param led_channel identifaier of the selected led(use defined macros RED/GREEN LED CHANNEL)
*/
void led_on(int led_channel);

/*
 * @brief turns seleced led off
 * @param led_channel identifaier of the selected led(use defined macros RED/GREEN LED CHANNEL)
*/
void led_off(int led_channel);

/*
 * @brief identifay what key was pressed when interupts is beaing processed
 * @param active_row row where the intterupt occured
 * @return returns charater representing the key pressed on the keybord
*/
char scan_matric_keyboard(uint32_t active_row);

/*
 * @brief resets timer
*/
void reset_timer();

/*
 * @brief callback function that is called when timer finished, it resets timer and turn the greeen led off, and set locked on ture
*/
void timer_callback(void *arg);

/*
 * @brief start timer for open state, set locked on false
*/
void start_timer();

/*
 * @brief clean the global buffer
*/
void reset_buffer();

/*
 * @brief helping funciton for check_string. extract new time from second part of the msg.
 * @param start from what position in msg we are starting
 * @return 0 on failuer and number on success
*/
uint32_t extract_new_time(int start);

/*
 * @brief this funciton is called when user press commit key, it check buffer if there is valid input and decied what to do next
*/
void check_string();

/*
 * @brief callback functioon, for when interupt ocure on the keyboard, call chack_string for proccessing the input
*/
void matrix_keybord_task(void *arg);

/*
 * @brief help funciton for process_ble, return character representing hex number, supported character are 0-9
 * @param character in hex representation
 * @return character on succes or null terminater for undefined character
*/
char hex_to_str(uint8_t hex);

/*
 * @brief checks the content of given string and store correct pwd or time
 * @param str actual string to check and store
 * @return 0 on success or 1 on failure
*/
int check_str_ble(char *str);

/*
 * @brief extracts the string from retrived data from ble communication
 * @param data retrived string 
 * @param length length of given string
*/
void process_ble(uint8_t *data, uint16_t length);




void delay(int n){
    vTaskDelay(n/portTICK_PERIOD_MS);
}

/* 
 * @brief callback function for interuption on terminal, it will add a interupt to a queue
 * @param arg
*/
static void IRAM_ATTR row_isr_handler(void *arg) {
    uint32_t row_pin = (uint32_t)arg;
    xQueueSendFromISR(row_queue, &row_pin, NULL);
}

void init_led(){
    //led pwm
    ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,  
    .timer_num = LEDC_TIMER_0,           
    .duty_resolution = PWM_RESOLUTION,  
    .freq_hz = PWM_FERQUENCY,           
    .clk_cfg = LEDC_AUTO_CLK            
    };
    ledc_timer_config(&ledc_timer);

    //configure Red LED PWM channel
    ledc_channel_config_t ledc_channel_red = {
        .gpio_num = RED_LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = RED_LED_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_red);

    //configure Green LED PWM channel
    ledc_channel_config_t ledc_channel_green = {
        .gpio_num = GREEN_LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = GREEN_LED_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_green);
}

void init_terminal(){
    //configurate terminal
    gpio_config_t col_conf  = {
        .pin_bit_mask = (1ULL << TERMINAL_COL_1) | (1ULL << TERMINAL_COL_2) |(1ULL << TERMINAL_COL_3), //select the pin
        .mode = GPIO_MODE_OUTPUT,             
        .pull_up_en = GPIO_PULLUP_DISABLE,    
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE        
    };
    gpio_config(&col_conf);

    gpio_config_t row_conf = {
        .pin_bit_mask = (1ULL << TERMINAL_ROW_1) | (1ULL << TERMINAL_ROW_2) | (1ULL << TERMINAL_ROW_3) | (1ULL << TERMINAL_ROW_4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  
    };
    gpio_config(&row_conf);

    //create a queue to handle row events
    row_queue = xQueueCreate(10, sizeof(uint32_t));

    //set up interrupt service routines for each row
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(TERMINAL_ROW_1, row_isr_handler, (void *)TERMINAL_ROW_1);
    gpio_isr_handler_add(TERMINAL_ROW_2, row_isr_handler, (void *)TERMINAL_ROW_2);
    gpio_isr_handler_add(TERMINAL_ROW_3, row_isr_handler, (void *)TERMINAL_ROW_3);
    gpio_isr_handler_add(TERMINAL_ROW_4, row_isr_handler, (void *)TERMINAL_ROW_4);

    ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_1, 0));
    ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_2, 0));
    ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_3, 0));
}

void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

int store_pwd(char *pwd){
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 1;
    }

    err = nvs_set_str(nvs_handle, "password", pwd);
    if (err != ESP_OK) {
        printf("Error (%s) setting password!\n", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return 1;
    }
    nvs_commit(nvs_handle);
    return 0;
}

int store_time_open(uint32_t *time){
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 1;
    }

    err = nvs_set_u32(nvs_handle, "time_open", *time);
    if (err != ESP_OK) {
        printf("Error (%s) setting opening time!\n", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return 1;
    }
    //create new time for ble
    nvs_commit(nvs_handle);
    return 0;
}

int get_info(char *pwd, uint32_t *open_time) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size;

    //open NVS storage with read-write access
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 1;
    }

    //check if the password exists
    err = nvs_get_str(nvs_handle, "password", NULL, &required_size);
    if (err == ESP_OK && required_size <= MAX_PWD_LENGTH) {
        nvs_get_str(nvs_handle, "password", pwd, &required_size);
        printf("Retrieved password: %s\n", pwd);
    } else if (err == ESP_ERR_NVS_NOT_FOUND || err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("Password not found or invalid, setting default...\n");

        //set default password
        err = nvs_set_str(nvs_handle, "password", "1111");
        if (err != ESP_OK) {
            printf("Error (%s) setting default password!\n", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return 1;
        }
        strcpy(pwd, "1111");  
    } else {
        printf("Error retrieving password (%s)!\n", esp_err_to_name(err));
        printf("Requred_size: %d free size: %d\n", required_size, MAX_PWD_LENGTH);
        nvs_close(nvs_handle);
        return 1;
    }

    //commit changes after setting the password
    if (err != ESP_OK) {
        nvs_commit(nvs_handle);
    }

    //get the opening time
    err = nvs_get_u32(nvs_handle, "time_open", open_time);
    if (err == ESP_OK) {
        printf("Retrieved opening time: %ld seconds\n", *open_time);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("Opening time not found, setting default...\n");

        //set default open time
        *open_time = 10;
        err = nvs_set_u32(nvs_handle, "time_open", *open_time);
        if (err != ESP_OK) {
            printf("Error (%s) setting default opening time!\n", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return 1;
        }
    } else {
        printf("Error retrieving opening time (%s)!\n", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return 1;
    }

    //commit changes after setting defaults
    if (err != ESP_OK) {
        nvs_commit(nvs_handle);
    }

    //close the NVS handle
    nvs_close(nvs_handle);
    return 0;
}

void short_blink(int led_channel, int num_blink) {
    bool led_on = ledc_get_duty(LEDC_HIGH_SPEED_MODE, led_channel) ? true : false;
    if(led_on){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
        delay(200); //delay for 200ms to ensure LED is off
    }
    
    //blink the LED on and off
    for (int i = 0; i < num_blink; i++) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, MAX_DUTY_CYCLE);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
        delay(200); 
        
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
        delay(200); 
    }

    if(led_on){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, MAX_DUTY_CYCLE);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
    }
}

void led_on(int led_channel){
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, MAX_DUTY_CYCLE);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
}

void led_off(int led_channel){
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, led_channel, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, led_channel);
}

char scan_matrix_keyboard(uint32_t active_row) {
    
    //activate each column and check the rows
    for (int col = 0; col < 3; col++) {
        //set the current column low, others high
        ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_1, col == 0 ? 0 : 1));
        ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_2, col == 1 ? 0 : 1));
        ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_3, col == 2 ? 0 : 1));

        //check if the active row is triggered
        gpio_num_t row_pin = active_row;
        if (gpio_get_level(row_pin) == 0) { // Active low: key press detected
            //find the row 
            int row = 0;
            if(row_pin == TERMINAL_ROW_1){
                row = 0;
            }
            if(row_pin == TERMINAL_ROW_2){
                row = 1;
            }
            if(row_pin == TERMINAL_ROW_3){
                row = 2;
            }   
            if(row_pin == TERMINAL_ROW_4){
                row = 3;
            }
            
            return key_map[row][col];
        }
    }

    return '\0'; //nothing detected
}

void reset_timer(){
    esp_timer_stop(timer_handle);
    esp_timer_delete(timer_handle);
    timer_active = false;
}

void timer_callback(void *arg){
    printf("Timer expired. Resetting buffer.\n");
    reset_timer();
    led_off(GREEN_LED_CHANNEL);
    locked = true;
    led_on(RED_LED_CHANNEL);
}

void start_timer(){
    const esp_timer_create_args_t timer_args ={
        .callback = &timer_callback,
        .name = "key_time"
    };

    if(!timer_active){
        esp_timer_create(&timer_args, &timer_handle);
        esp_timer_start_once(timer_handle, open_time*TO_SEC);  // 10 seconds
        led_on(GREEN_LED_CHANNEL);
        led_off(RED_LED_CHANNEL);
        timer_active = true;
        printf("Timer started.\n");
    }
}

void reset_buffer(){
    buffer_position = 0;
    memset(input_buffer, '\0', sizeof(input_buffer));
    printf("Buffer reset.\n");
}

uint32_t extract_new_time(int start){
    char *start_ptr = &input_buffer[start]; //start from the 7th index
    char *end_ptr;

    //search for a termination character '*'
    end_ptr = strchr(start_ptr, '*');
    if (end_ptr != NULL) {
        return 0; 
    }

    //null-terminate the numeric substring before '#'
    end_ptr = strchr(start_ptr, '#');
    *end_ptr = '\0';

    //convert the substring to an integer
    int num = atoi(start_ptr);

    if(num > 9999){                                            //time cant be more then 5 digits
        return 0;
    }
    return (num >= 0) ? (uint32_t)num : 0;
}

void check_string(){

    char first = input_buffer[0];
    char extracted_string[11] = {0};
    //what to store
    char wh_to_st = '\0';
    
    //we are in locked state
    if(locked){
        if(first != '*'){      //we dont want to change anything
            for(int i=0; i<4; i++){
                extracted_string[i] = input_buffer[i];
            }
            extracted_string[4] = '\0';
            if(!strcmp(pwd, extracted_string)){
                locked = false;
                start_timer();
                reset_buffer();
                return;
            }
            short_blink(RED_LED_CHANNEL, 3);
            reset_buffer();
            return;
        }
        if(strlen(input_buffer) <= 8 ){
            short_blink(RED_LED_CHANNEL, 3);
            reset_buffer();
            return;
        }
        //we want to change something
        for(int i=0; i<4; i++){
            extracted_string[i] = input_buffer[i+1];
        }
        extracted_string[4] = '\0';
        
        if(strcmp(extracted_string, pwd)){           //
            //didnt get the correct pwd 
            short_blink(RED_LED_CHANNEL, 3);
            reset_buffer();
            return;
        }
        
        wh_to_st = input_buffer[6];
        
        if(wh_to_st == '*'){                //second argument was * we want to store time for the open period
            uint32_t tmp = extract_new_time(7);
            if(tmp == 0){
                short_blink(RED_LED_CHANNEL, 3);
                reset_buffer();
                return;
            }
            store_time_open(&tmp);
            open_time = tmp;
            short_blink(GREEN_LED_CHANNEL, 2);
            
        }else{                              //second argument was num. we want to store new pwd
            int k = 6;
            for(; input_buffer[k] != '#'; k++){        //i is for the positon in input buffer, we need to extract 6 to be on right position in extracted_string
                extracted_string[k-6] = input_buffer[k];
            }
            extracted_string[5] = '\0';
            if(k != 10){
                short_blink(RED_LED_CHANNEL, 3);
                reset_buffer();
                return;
            }
            if(atoi(extracted_string) == 0){                    //check if new pwd is in numbers
                short_blink(RED_LED_CHANNEL, 3);
                reset_buffer();
                return;
            }
            store_pwd(extracted_string);
            strcpy(pwd, extracted_string);
            short_blink(GREEN_LED_CHANNEL, 2);
        }
        reset_buffer();
        return;
    }

    //we are unlocked we can change struff without needing to add a pwd
    if(first == '*'){   //we want to change something
        printf("unlocked\n");
        wh_to_st = input_buffer[1];
        if(wh_to_st == '*'){                           //we want to store new timew
            uint32_t tmp = extract_new_time(2);
            if(tmp == 0){
                short_blink(RED_LED_CHANNEL, 3);
                reset_buffer();
                return;
            }
            store_time_open(&tmp);
            open_time = tmp;
            short_blink(GREEN_LED_CHANNEL, 2);
            reset_buffer();
            return;
        }
      
        //want to change pwd
        int j = 1;
        for(; input_buffer[j] != '#'; j++){        //i is for the positon in input buffer, we need to extract 6 to be on right position in extracted_string
            extracted_string[j-1] = input_buffer[j];
        }
        if(j != 5 ){                                      //pwd cant be longer then 4 digits(5 bc j is starting at 1)
            short_blink(RED_LED_CHANNEL, 2);
            reset_buffer();
            return;
        }
        extracted_string[5] = '\0';
        
        if(atoi(extracted_string) == 0){                    //check if new pwd is in numbers
            short_blink(RED_LED_CHANNEL, 3);
            reset_buffer();
            return;
        }
        store_pwd(extracted_string);
        strcpy(pwd, extracted_string);
        
        short_blink(GREEN_LED_CHANNEL, 2);
    }

    //reset
    reset_buffer();
}

void matrix_keyboard_task(void *arg) {
    uint32_t active_row;
    
    while (1) {
        //wait for a row interrupt event
        if (xQueueReceive(row_queue, &active_row, portMAX_DELAY)) {
            char key = scan_matrix_keyboard(active_row);
            if (key != '\0') {
                printf("Key pressed: %c\n", key);
                if(buffer_position >= BUFFER_SIZE){
                    reset_buffer();
                }
                input_buffer[buffer_position++] = key;
                if( key == '#'){                            //commit
                    check_string();
                }
                delay(300); //for key to go to normal position
            }
            //reset the colums
            ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_1, 0));
            ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_2, 0));
            ESP_ERROR_CHECK(gpio_set_level(TERMINAL_COL_3, 0));
        }
        delay(10);
    }
    vTaskDelete(NULL);
}

char hex_to_str(uint8_t hex){
    switch (hex){
        case 0x30: return '0';
        case 0x31: return '1';
        case 0x32: return '2';
        case 0x33: return '3';
        case 0x34: return '4';
        case 0x35: return '5';
        case 0x36: return '6';
        case 0x37: return '7';
        case 0x38: return '8';
        case 0x39: return '9';
        case 0x2a: return '*';
        default: return '\0';
    }
}

int check_str_ble(char *str){
    if(locked){                     //we cant  change pwd or time if we are in locked state
        return 1;
    }
    char new_nmb[5] = {0};

    if(str[1] == '*'){              //we want to store new time bc 2-second arg is *
        for(int i=2; str[i] != '\0'; i++){
            new_nmb[i-2] = str[i];
        }
        int tmp = atoi(new_nmb);
        if(tmp == 0){
            printf("wrong time form ble\n");
            return 1;
        }
        uint32_t tmp_uint = (uint32_t)tmp;
        store_time_open(&tmp_uint);
        open_time = tmp;
        printf("new time: %s\n", new_nmb);
        return 0;
    }

    for(int i=1; str[i] != '\0';i++){
        new_nmb[i-1] = str[i];
    }
    printf("new pwd: %s\n", new_nmb);
    store_pwd(new_nmb);
    strcpy(pwd, new_nmb);
    return 0;
}

void process_ble(uint8_t* data, uint16_t length){
    
    char str[7] = "\0"; // Max 4 characters + null terminator
    char tmp = '\0';

    // Validate input length
    if (length > 6 || length <= 1) { // Assuming we expect 1-4 valid characters
        printf("ERR: given wrong string length.\n");
        short_blink(RED_LED_CHANNEL, 3);
        return;
    }

    // Process each byte
    for (int i = 0; i < length; i++) {

        //convert to character
        tmp = hex_to_str(data[i]);

        //append to string 
        if (i <= sizeof(str) - 1) { // Ensure there's room for the null terminator
            str[i] = tmp;
        } else {
            printf("String buffer overload.\n");
            short_blink(RED_LED_CHANNEL, 3);
            return;
        }
    }

    // Null-terminate the string
    str[length] = '\0';

    if(check_str_ble(str)){
        short_blink(RED_LED_CHANNEL, 3);
        return;
    }
    //nothing wroong let user know
    short_blink(GREEN_LED_CHANNEL, 2);
}  

/* code from mentiooned tutorial */
//callback function
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            printf("Starting advertising.\n");
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                printf("ERR: advertising failed.\n");
            }else{
                printf("Advertising started successfully.\n");
            }
            break;

        default://unhandeled gap service
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    
    switch (event) {
        case ESP_GATTS_REG_EVT:{                            //register
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TAG);
            if(set_dev_name_ret != ESP_OK){
                printf("ERR: couldnt set name of the device.\n");
            }
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                printf("ERR: config adv data failed.\n ERRORCODE: %x\n", ret);
            }
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                printf("Config scan response data failed.\nERRORCODE: %x\n", ret);
            }
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GAT_DB, SVC_INST_ID);
            if (create_attr_ret){
                printf("Create attr table failed.\nERRORCODE: %x\n", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{                 //
            
            if (param->add_attr_tab.status != ESP_GATT_OK){
                printf("ERR: create attribute table failed.\n");
            }else {
                memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
                esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            }
            break;
        }

        case ESP_GATTS_WRITE_EVT:                               //!got data from web api 
         
            uint8_t* data = param->write.value; // The written data
            uint16_t length = param->write.len; // Length of the data

            //procces retrived data
            process_ble(data, length);
            break;
        case ESP_GATTS_CONNECT_EVT:
            printf("Device connected\n");
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            printf("Device disconnected.\n");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,     
    },

};

//callback funciton
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){

    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            printf("ERR registration of app failed.\n");
            return;
        }
    }

    
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_tab[idx].gatts_if) {
            if (profile_tab[idx].gatts_cb) {
                profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
   
}

void init_ble() {
    
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        printf("ERR: init controller failed: %s\n", esp_err_to_name(ret));
        return;
    }


    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        printf("ERR: enable controller failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        printf("ERR: init bluetooth failed: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        printf("ERR: enable bluetooth failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if(ret){
        printf("ERR: registering callback(gatts) failed: %s\n", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if(ret){
        printf("ERR: registering callback(gap) failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if(ret){
        printf("ERR: gatts app register failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(512);
    if(ret){
        printf("ERR: lcal mtu failed: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("BLE server initialized.\n");
}

void app_main() {
    
    //Initialize 
    init_led();
    init_terminal();
    init_nvs();
    if(get_info(pwd, &open_time)){
        printf("Couldnt get data.\n");
        exit(1);
    }
    init_ble();

    led_on(RED_LED_CHANNEL);
    xTaskCreate(matrix_keyboard_task, "matrix_keyboard_task", 2048, NULL, 10, NULL);

    while(1) {
        delay(500);
    }
}