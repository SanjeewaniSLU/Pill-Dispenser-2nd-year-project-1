#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"
#include "hardware/uart.h"

#define SW_0 9
#define SW_2 7
#define LED_1 20

#define STATE_ADDRESS 0x7FFE
#define DAY_ADDRESS 0x7FFC

#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define OPTO_FORK 28
#define STEP_DELAY_MS 2

#define PIEZO_SENSOR 27

#define I2C_PORT i2c0
#define EEPROM_ADDR 0x50
#define SDA_PIN 16
#define SCL_PIN 17
#define BAUD_RATE 100000

#define UART_ID uart1
#define TX_PIN 4
#define RX_PIN 5
#define BAUD_RATE_LORA 9600
#define SIZE_OF_BUFFER 200

void IdleMode();
void Calibrating();
void DispensingPills();
void HandleError();

typedef enum {
    STATE_WAIT,
    STATE_CALIBRATION,
    STATE_DISPENSE_PILLS,
}dispenser_state;

typedef struct {
    uint8_t value;
    uint8_t int_value;
} values_check;

bool sensor_triggered = false;
bool msg_sent_server = false;
bool connected_to_server= false;


void setup_UART(){  //--------------------------------------------------------------------------
    uart_init(UART_ID, BAUD_RATE_LORA);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
}

bool send_command_LoRa(char *command){ //-------------------------------------------------------
    uart_puts(UART_ID, command);
    sleep_ms(5);

    if (uart_is_readable_within_us(UART_ID, 500000)) {
        return true;

    } else {
        return false;

    }
}

bool read_response_with_us(uart_inst_t *uart, uint32_t us, char *f_buffer){//-----------------------------------
    int i = 0;
    uint32_t t = time_us_32();
    memset(f_buffer, '\0', SIZE_OF_BUFFER);
    do {
        if (uart_is_readable(uart)){
            char char_received = uart_getc(uart);
            if (char_received == '\n' || char_received == '\r'){
                if(i > 0){
                    f_buffer[i] = '\0';
                    return true;
                }
            }
            else{
                if(i < (SIZE_OF_BUFFER - 1)){
                    f_buffer[i] = char_received;
                    i++;
                    t = time_us_32();
                }
                else{
                    printf("Buffer is full.\n");
                }
            }
        }
    } while ((time_us_32() - t) <= us);
    return false;
}

bool process_response_status(char *f_buffer){ //-----------------------------------------------------------------
    if(strstr(f_buffer, "OK") != NULL){
        printf("Connected to LoRa module.\n");
        return true;
    }
    else{
        printf("Incorrect response.\n");
        return false;
    }
}

void i2c_setup() { //-------------------------------------------------------------------------------------------
    i2c_init(I2C_PORT, BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void eeprom_write_system_state(uint16_t addr, uint8_t data) {//---------------------------------------------------
    uint8_t buffer[4];
    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = addr & 0xFF;
    buffer[2] = data;
    buffer[3] = ~data;

    i2c_write_blocking(I2C_PORT, EEPROM_ADDR, buffer, 4, false);
    sleep_ms(30); // time?
}


values_check eeprom_read_system_state(uint16_t addr) { //--------------------------------------------------------------
    uint8_t addr_buf[2] = {(addr >> 8) & 0xFF, addr & 0xFF};
    uint8_t data[2];

    i2c_write_blocking(I2C_PORT, EEPROM_ADDR, addr_buf, 2, true);
    i2c_read_blocking(I2C_PORT, EEPROM_ADDR, data, 2, false);

    values_check valuesCheck =  {data[0], data[1]};
    return valuesCheck;
}

bool value_is_valid(values_check *valuesCheck){
    if(valuesCheck->value == (uint8_t)~valuesCheck->int_value){
        return true;
    }
    else{
        return false;
    }
}

void piezo_interrupt(uint gpio, uint32_t events) {//----------------------------------------------------------------
    sensor_triggered = true;
}

void SetMotor(){
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);

    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_set_dir(IN4, GPIO_OUT);
}

void SetOptoFork() { //--------------------------------------------------------------------------------------------
    gpio_init(OPTO_FORK);
    gpio_set_dir(OPTO_FORK, GPIO_IN);
    gpio_pull_up(OPTO_FORK);
}

void SetPiezoSensor() { //-------------------------------------------------------------------------------------------
    gpio_init(PIEZO_SENSOR);
    gpio_set_dir(PIEZO_SENSOR, GPIO_IN);
    gpio_pull_up(PIEZO_SENSOR);
}

const int half_step_sequence[8][4] = { //--------------------------------------------------------------------------
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1},
};

void activate_motor_step() { //------------------------------------------------------------------------------------
    static int step_index = -1;
    step_index = (step_index + 1) % 8;

    gpio_put(IN1, half_step_sequence[step_index][0]);
    gpio_put(IN2, half_step_sequence[step_index][1]);
    gpio_put(IN3, half_step_sequence[step_index][2]);
    gpio_put(IN4, half_step_sequence[step_index][3]);
}

void activate_motor_step_reverse() { //-------------------------------------------------------------------------
    static int step_index = 0;
    step_index = (step_index - 1 + 8) % 8;

    gpio_put(IN1, half_step_sequence[step_index][0]);
    gpio_put(IN2, half_step_sequence[step_index][1]);
    gpio_put(IN3, half_step_sequence[step_index][2]);
    gpio_put(IN4, half_step_sequence[step_index][3]);
}

void calibrate_motor() { //-------------------------------------------------------------------------------------
    int count = 0;

    for (int i = 0; i < 2; i++) {

        while (gpio_get(OPTO_FORK)) {
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }
        count = 0;
        while (!gpio_get(OPTO_FORK)) {
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }
        while (gpio_get(OPTO_FORK)) {
            activate_motor_step();
            sleep_ms(STEP_DELAY_MS);
            count++;
        }
    }
    printf("Step per revolution: %d\n", count);
    for (int i = 0; i < 142; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
    printf("System is calibrated.\n");
    printf("To start dispensing precess PRESS SW_0.\n");
}

void run_motor(int N) { //-------------------------------------------------------------------------------------------

    for (int i = 0; i < 512*N; i++) {
        activate_motor_step();
        sleep_ms(STEP_DELAY_MS);
    }
}

uint8_t  days_passed = 0;

void recalib_motor(){ //-------------------------------------------------------------------------------------------------
    gpio_put(LED_1, 1);
    values_check  result = eeprom_read_system_state(DAY_ADDRESS);
    if(value_is_valid(&result)){
        days_passed = result.value;
    }
    int rem_day = days_passed;
    while(days_passed != 1){
        for (int i = 0; i < 512; i++) {
            activate_motor_step_reverse();
            sleep_ms(STEP_DELAY_MS);
        }
        days_passed--;
    }
    while (gpio_get(OPTO_FORK)) {
        activate_motor_step_reverse();
        sleep_ms(STEP_DELAY_MS);
    }
    for (int i = 0; i < 152; i++) {
        activate_motor_step_reverse();
        sleep_ms(STEP_DELAY_MS);
    }

    sleep_ms(1000);
    run_motor(rem_day);
    eeprom_write_system_state(DAY_ADDRESS, rem_day);
   // sleep_ms(1000);
}

void IdleMode(){ //------------------------------------------------------------------------------------------------------
    gpio_put(LED_1, 1);
    sleep_ms(200);
    gpio_put(LED_1, 0);
    sleep_ms(200);
}

void Calibrating(){ //---------------------------------------------------------------------------------------------------
    if(msg_sent_server) {
        send_command_LoRa("AT+MSG=\"System is under calibration process.\"\r\n");
    }
    sleep_ms(10);
    gpio_put(LED_1, 1);
    calibrate_motor();
}

void DispensingPills(){ //---------------------------------------------------------------------------------------------
    gpio_put(LED_1, 0);

    values_check  result = eeprom_read_system_state(DAY_ADDRESS);
    if(value_is_valid(&result)){
        days_passed = result.value;
    }

    int step_and_stop = days_passed;
    while(step_and_stop < 7){
        sensor_triggered=false;
        run_motor(1);
        absolute_time_t time = make_timeout_time_ms(10);

        while(!sensor_triggered && !time_reached(time)){
            tight_loop_contents();
        }
        printf("DAY: %d\n", step_and_stop+1);
        if(sensor_triggered){
            if(msg_sent_server) {
                char msg[40];
                snprintf(msg, 39,  "AT+MSG=\"DAY %d: Dispensed.\"\r\n",step_and_stop+1);
                send_command_LoRa(msg);
            }

            eeprom_write_system_state(DAY_ADDRESS, step_and_stop+1);
            sleep_ms(10);
            printf("Pill was dropped from compartment:  #%d.\n", step_and_stop+1);
            sensor_triggered = false;
            sleep_ms(3000);
        }
        else{
            if(msg_sent_server) {
                char msg[40];
                snprintf(msg, 39,  "AT+MSG=\"DAY %d: NOT dispensed.\"\r\n",step_and_stop+1);
                send_command_LoRa(msg);
                sleep_ms(10);
            }

            eeprom_write_system_state(DAY_ADDRESS, step_and_stop+1);
            //sleep_ms(10);
            printf("Pill was  NOT dropped from compartment:  #%d.\n", step_and_stop+1);
            sleep_ms(10);

            HandleError();
            sleep_ms(2000);
        }

        step_and_stop++;

        if(step_and_stop  == 7){
            eeprom_write_system_state(DAY_ADDRESS, 0);
            printf("Dispenser is empty.\n");
            printf("System is in IDLE MODE. Waiting for calibration (PRESS SW_2).\n");
        }
    }
}

void HandleError(){ //------------------------------------------------------------------------------------------------
    for(int blink = 0; blink < 5; blink++ ){
        gpio_put(LED_1, 1);
        sleep_ms(200);
        gpio_put(LED_1, 0);
        sleep_ms(200);
    }
}

void SetupButtons(){ //-----------------------------------------------------------------------------------------------
    gpio_init(SW_0);
    gpio_set_dir(SW_0, GPIO_IN);
    gpio_pull_up(SW_0);

    gpio_init(SW_2);
    gpio_set_dir(SW_2, GPIO_IN);
    gpio_pull_up(SW_2);
}

void SetupLed(){ //---------------------------------------------------------------------------------------------------
    gpio_init(LED_1);
    gpio_set_dir(LED_1, GPIO_OUT);
    gpio_put(LED_1, false);
}

void connect_LORA(){ //-----------------------------------------------------------------------------------------------
    char buffer[SIZE_OF_BUFFER];
    int attempt_no;
    attempt_no = 1;
    while(attempt_no <  7){
        if(attempt_no == 6){
            printf("Lora module is not responding\n\n");
            break;
        }
        printf("Attempt %d...\n", attempt_no);

        if(send_command_LoRa("AT\r\n")){
            sleep_ms(10);
            if(read_response_with_us(UART_ID, 2000, buffer)){
                sleep_ms(10);
                if(process_response_status(buffer)){
                    attempt_no = 7;
                }
                else{
                    attempt_no++;
                }
            }
            else{
                attempt_no++;
            }
        }
        else{
            attempt_no++;
        }
    }
}

bool handle_join_command() { //--------------------------------------------------------------------------------------
    char temp_buffer[1000];
    bool join_done = false;
    time_t start_time = time(NULL);

    send_command_LoRa("AT+JOIN\r\n");
    printf("Join process in progress...\n");
    while (1) {
        memset(temp_buffer, 0, sizeof(temp_buffer));
        read_response_with_us(UART_ID, 20000000, temp_buffer);

        if (strstr(temp_buffer, "+JOIN: NetID") != NULL) {
            printf("Connection with server is established\n");
            join_done = true;
        }
        else if (strstr(temp_buffer, "+JOIN: Join failed") !=NULL) {
            join_done = false;
            printf("Connection to server failed\n");
        }
        else if((strstr(temp_buffer, "+JOIN: Done") != NULL)){
            return join_done;
        }

        if (difftime(time(NULL), start_time) > 30) {
            printf("Timeout: Join process did not complete within 20 seconds.\n");
            return false;
        }
    }
}

bool handle_msg_command(char *message) { //-------------------------------------------------------------------------
    char temp_buffer[1000];

    time_t start_time = time(NULL);

    send_command_LoRa(message);

    while (1) {

        memset(temp_buffer, 0, sizeof(temp_buffer));
        read_response_with_us(UART_ID, 20000000, temp_buffer);

        if (strstr(temp_buffer, "+MSG: FPENDING") != NULL) {
            msg_sent_server = true;
            printf("Message sent to server successfully.\n");
        }
        else if (strstr(temp_buffer, "+MSG: Done") != NULL) {
            return msg_sent_server;
        }

        if (difftime(time(NULL), start_time) > 30) {
            printf("Timeout: Sending message process did not complete within 30 seconds.\n");
            msg_sent_server = false;
            return false;
        }
    }
}

void send_connection_command_server(){ //---------------------------------------------------------------------------
    char temp_buffer[1000];
    for(int i = 0; i < 3 && !connected_to_server; i++) {
        send_command_LoRa("AT+MODE=LWOTAA\r\n");
        memset(temp_buffer, 0, sizeof(temp_buffer));
        read_response_with_us(UART_ID, 20000000, temp_buffer );
        if (strstr(temp_buffer, "+MODE: LWOTAA") != NULL) {
            send_command_LoRa("AT+KEY=APPKEY,\"da23d74bb2ab4fe57c115eae676c044b\"\r\n");
            memset(temp_buffer, 0, sizeof(temp_buffer));
            read_response_with_us(UART_ID, 20000000, temp_buffer );
            if (strstr(temp_buffer, "+KEY: APPKEY") != NULL) {
                send_command_LoRa("AT+CLASS=A\r\n");
                memset(temp_buffer, 0, sizeof(temp_buffer));
                read_response_with_us(UART_ID, 20000000, temp_buffer );
                if (strstr(temp_buffer, "+CLASS: A") != NULL) {
                    send_command_LoRa("AT+PORT=8\r\n");
                    memset(temp_buffer, 0, sizeof(temp_buffer));
                    read_response_with_us(UART_ID, 20000000, temp_buffer );
                    if (strstr(temp_buffer, "+PORT") != NULL) {
                        for(int i = 0; i < 3 && !connected_to_server; i++){
                            connected_to_server = handle_join_command();
                        }
                        if(connected_to_server)
                        {
                            handle_msg_command("AT+MSG=\"BOOT\"\r\n");
                        }
                        else{
                            printf("Command 'AT+JOIN' was not sent successfully\n");
                        }
                    }
                    else{
                        printf("Command 'AT+PORT=8' was not sent successfully\n");
                    }
                }
                else{
                    printf("Command 'AT+CLASS=A' was not sent successfully\n");
                }
            }
            else{
                printf("Command 'AT+KEY=APPKEY, da23d74bb2ab4fe57c115eae676c044b' was not sent successfully\n");
            }
        }
        else{
            printf("Command 'AT+MODE=LWOTAA' was not sent successfully\n");
        }
    }
}

int main() { //---------------------------------------------------MAIN--------------------------------------------------
    msg_sent_server = false;
    connected_to_server= false;

    stdio_init_all();
    SetMotor();
    SetOptoFork();
    SetPiezoSensor();
    i2c_setup();
    SetupButtons();
    SetupLed();
    setup_UART();
    connect_LORA();
    send_connection_command_server();

    gpio_set_irq_enabled_with_callback(PIEZO_SENSOR, GPIO_IRQ_EDGE_FALL, true, piezo_interrupt);

    dispenser_state current_state;
    uint8_t state = 0;
    values_check  result = eeprom_read_system_state(STATE_ADDRESS);
    if(value_is_valid(&result)){
        state = result.value;
    }

    if(state == 2){
        printf("System was powered off/rebooted during the Calibration process and must be calibrated again. (Press SW_2).\n");
        current_state = STATE_WAIT;
    }
    else if(state == 3){
        uint8_t  day = 0;
        values_check result_1 = eeprom_read_system_state(DAY_ADDRESS);
        if(value_is_valid(&result_1)){
            day = result_1.value;
        }

        if(day != 0){
            recalib_motor();
            DispensingPills();
            current_state = STATE_WAIT;
        }
        else if(day == 0){
            printf("System was powered off/rebooted. System is calibrated and ready to dispense pills. (Press SW_0).\n");
            current_state = STATE_DISPENSE_PILLS;
        }
    }

    else if(state == 1){
        current_state = STATE_WAIT;
        printf("System is in IDLE MODE. Waiting for calibration (PRESS SW_2).\n");
    }
    else{
        printf("Invalid state of the system.\n");
    }

    while (1) {
        switch (current_state) {

            case STATE_WAIT:
                if(msg_sent_server) {
                    send_command_LoRa("AT+MSG=\"System is in IDLE MODE.\"\r\n");
                }

                sleep_ms(10);
                days_passed = 0;
                eeprom_write_system_state(DAY_ADDRESS, 0);
                eeprom_write_system_state(STATE_ADDRESS, 1);
                while (current_state == STATE_WAIT){
                    IdleMode();
                    if (gpio_get(SW_2) == 0) {

                        printf("Calibration process is in progress...\n");
                        current_state = STATE_CALIBRATION;
                    }
                }
                break;

            case STATE_CALIBRATION:
                eeprom_write_system_state(STATE_ADDRESS, 2);
                Calibrating();
                if(msg_sent_server) {
                    send_command_LoRa("AT+MSG=\"System is calibrated.\"\r\n");
                }
                current_state = STATE_DISPENSE_PILLS;
                break;


            case STATE_DISPENSE_PILLS:
                sleep_ms(10);
                eeprom_write_system_state(STATE_ADDRESS, 3);
                if (gpio_get(SW_0) == 0) {
                    printf("Pill dispensing is in progress...\n");
                    DispensingPills();
                    current_state = STATE_WAIT;
                }
                break;
        }
    }
}