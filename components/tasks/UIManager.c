#include "UIManager.h"

#include <stdbool.h>

#include "ext_serial.h"

#include "AssessMotorState.h"

#define MENU_INPUT_SIZE 10

static char input[MENU_INPUT_SIZE];

static uint8_t rx_uart_input(){
  uint8_t index = 0;

  while(index < MENU_INPUT_SIZE - 1){
    char c_byte;
    int len = ext_serial_read(&c_byte, 1);
    if(len <= 0) continue;

    if(c_byte == '\n' || c_byte == '\r'){
      serial_print_from_task("\r\n");

      if(index == 0) return 0;

      input[index] = '\0';
      break;
    }

    input[index++] = c_byte;

    char echo_c_byte[2] = {c_byte, '\0'};
    serial_print_from_task(echo_c_byte);
  }

  if(index >= MENU_INPUT_SIZE - 1){
    serial_print_from_task("\r\n");

    while(1){
      char c_byte;
      int len = ext_serial_read(&c_byte, 1);
      if (len > 0 && (c_byte == '\n' || c_byte == '\r')) break;
    }
  }

  return index;
}

static void print_menu(){
  serial_print_from_task("\r\n");
  serial_print_from_task("=================================\r\n");
  serial_print_from_task("      MOTOR UI MANAGER MENU\r\n");
  serial_print_from_task("=================================\r\n");
  serial_print_from_task("1. Show Last 10 Measurements\r\n");
  serial_print_from_task("2. View Statistics\r\n");
  serial_print_from_task("3. Reset Statistics\r\n");
  serial_print_from_task("4. View Threshold Settings\r\n");
  serial_print_from_task("5. Change Threshold Settings\r\n");
  serial_print_from_task("=================================\r\n");
  serial_print_from_task("Select an option: ");
}

static EventGroupHandle_t menu_option_event_group;

static SemaphoreHandle_t serial_print_from_task_mutex;

void vTask_UIManager(void *pvParameters){
  while(1){
    print_menu();
    uint8_t option_index = rx_uart_input();
    Menu_Option chosen_option = (option_index == 0 || option_index > 2) ? MENU_INVALID_OPTION : (Menu_Option) atoi(input);

    EventBits_t finished_rq;

    switch(chosen_option){
      case MENU_SHOW_MEASUREMENTS:
        xEventGroupSetBits(menu_option_event_group, BIT_MENU_SHOW_MEASUREMENTS);
        finished_rq = xEventGroupWaitBits(menu_option_event_group, BIT_FINISHED_MENU_RQ_OPTION, pdTRUE, pdTRUE, portMAX_DELAY);
        if((finished_rq & BIT_FINISHED_MENU_RQ_OPTION) == BIT_FINISHED_MENU_RQ_OPTION) serial_print_from_task("\r\n");

        break;

      case MENU_VIEW_STATS:
        xEventGroupSetBits(menu_option_event_group, BIT_MENU_VIEW_STATS);
        finished_rq = xEventGroupWaitBits(menu_option_event_group, BIT_FINISHED_MENU_RQ_OPTION, pdTRUE, pdTRUE, portMAX_DELAY);
        if((finished_rq & BIT_FINISHED_MENU_RQ_OPTION) == BIT_FINISHED_MENU_RQ_OPTION) serial_print_from_task("\r\n");

        break;

      case MENU_RESET_STATS:
        xEventGroupSetBits(menu_option_event_group, BIT_MENU_RESET_STATS);
        finished_rq = xEventGroupWaitBits(menu_option_event_group, BIT_FINISHED_MENU_RQ_OPTION, pdTRUE, pdTRUE, portMAX_DELAY);
        if((finished_rq & BIT_FINISHED_MENU_RQ_OPTION) == BIT_FINISHED_MENU_RQ_OPTION) serial_print_from_task("\r\n");

        break;

      case MENU_VIEW_THRESHOLDS:
        for(Metric_Type i = METRIC_ACCEL_RMS; i <= METRIC_TEMPERATURE; i++){
          if(i == METRIC_ACCEL_RMS) serial_print_from_task("Accel RMS (g)\r\n");
          else if(i == METRIC_CREST_FACTOR) serial_print_from_task("Crest Factor\r\n");
          else serial_print_from_task("Temperature (°C)\r\n");

          for(Motor_State j = STATE_WARNING; j < STATE_FAULT_PENDING; j++){
            if(j == STATE_WARNING) serial_print_from_task("Warning State\r\n");
            else if(j == STATE_ALARM) serial_print_from_task("Alarm State\r\n");
            else serial_print_from_task("Fault State\r\n");

            for(uint8_t k = 0; k < 2; k++){
              char t_hold[10];
              bool e_flag;

              if(k == 0){
                serial_print_from_task("Enter: ");
                e_flag = true;
              }

              else{
                serial_print_from_task("Exit: ");
                e_flag = false;
              }

              snprintf(t_hold, sizeof(t_hold), "%f", thresholds_get_value(i, j, e_flag));
              serial_print_from_task(t_hold);
              serial_print_from_task("\r\n");
            }
          }
        }

        break;

      case MENU_CHANGE_THRESHOLDS:
        for(Metric_Type i = METRIC_ACCEL_RMS; i <= METRIC_TEMPERATURE; i++){
          if(i == METRIC_ACCEL_RMS) serial_print_from_task("Accel RMS (g)\r\n");
          else if(i == METRIC_CREST_FACTOR) serial_print_from_task("Crest Factor\r\n");
          else serial_print_from_task("Temperature (°C)\r\n");

          for(Motor_State j = STATE_WARNING; j < STATE_FAULT_PENDING; j++){
            if(j == STATE_WARNING) serial_print_from_task("Warning State\r\n");
            else if(j == STATE_ALARM) serial_print_from_task("Alarm State\r\n");
            else serial_print_from_task("Fault State\r\n");

            for(uint8_t k = 0; k < 2; k++){
              bool e_flag;

              if(k == 0){
                serial_print_from_task("Change Enter? (y/n): ");
                e_flag = true;
              }

              else{
                serial_print_from_task("Change Exit? (y/n): ");
                e_flag = false;
              }

              uint8_t yn_index = rx_uart_input();
              while(yn_index == 0 || yn_index > 1 || (input[0] != 'y' && input[0] != 'n' && yn_index == 1)){
                serial_print_from_task("Invalid entry! Either type \'y\' to change value or \'n\' to skip: ");
                yn_index = rx_uart_input();
              }

              if(input[0] == 'y'){
                serial_print_from_task("Type new value: ");
                uint8_t value_index = rx_uart_input();
                char *endptr;
                float num = strtof(input, &endptr);
                while(value_index == 0 || input == endptr){
                  serial_print_from_task("Invalid entry! Type a valid number: ");
                  value_index = rx_uart_input();
                  num = strtof(input, &endptr);
                }
                thresholds_set_value(i, j, e_flag, num);
                serial_print_from_task("Value changed\r\n");
              } else serial_print_from_task("Value kept\r\n");
            }
          }
        }

        break;

      case MENU_INVALID_OPTION:
      default:
        serial_print_from_task("Invalid option! Please enter a number within 1 to 5\r\n");

        break;
    }
  }
}

esp_err_t menu_option_event_group_init(){
	menu_option_event_group = xEventGroupCreate();
	if(menu_option_event_group == NULL) return ESP_ERR_NOT_FOUND;
	return ESP_OK;
}

EventBits_t rx_user_menu_option(){
  return xEventGroupWaitBits(menu_option_event_group, BIT_MENU_SHOW_MEASUREMENTS | BIT_MENU_VIEW_STATS | BIT_MENU_RESET_STATS, pdTRUE, pdFALSE, 0);
}

void set_bit_finished_menu_rq_option(){
  xEventGroupSetBits(menu_option_event_group, BIT_FINISHED_MENU_RQ_OPTION);
}

esp_err_t serial_print_from_task_mutex_init(){
	serial_print_from_task_mutex = xSemaphoreCreateMutex();
	if(serial_print_from_task_mutex == NULL) return ESP_ERR_NOT_FOUND;
	return ESP_OK;
}

void serial_print_from_task(const char *msg){
  if(xSemaphoreTake(serial_print_from_task_mutex, portMAX_DELAY)){
    ext_serial_print(msg);
    xSemaphoreGive(serial_print_from_task_mutex);
  }
}
