#include "UIManager.h"

#include "ext_serial.h"

#define MENU_INPUT_SIZE 5

static void print_menu(){
  ext_serial_print("\r\n");
  ext_serial_print("=================================\r\n");
  ext_serial_print("      MOTOR UI MANAGER MENU\r\n");
  ext_serial_print("=================================\r\n");
  ext_serial_print("1. Show Last 10 Measurements\r\n");
  ext_serial_print("2. View Statistics\r\n");
  ext_serial_print("3. Reset Statistics\r\n");
  ext_serial_print("4. View Threshold Settings\r\n");
  ext_serial_print("5. Change Threshold Settings\r\n");
  ext_serial_print("=================================\r\n");
  ext_serial_print("Select an option: ");
}

static menu_option_t rx_menu_option(){
  char input[MENU_INPUT_SIZE];
  uint8_t index = 0;

  while(index < MENU_INPUT_SIZE - 1){
    char c_byte;
    int len = ext_serial_read(&c_byte, 1);
    if(len <= 0) continue;

    if(c_byte == '\n' || c_byte == '\r'){
      ext_serial_print("\r\n");

      if(index == 0) return MENU_INVALID_OPTION;

      input[index] = '\0';
      break;
    }

    input[index++] = c_byte;

    char echo_c_byte[2] = {c_byte, '\0'};
    ext_serial_print(echo_c_byte);
  }

  if(index >= MENU_INPUT_SIZE - 1){
    ext_serial_print("\r\n");

    while(1){
      char c_byte;
      int len = ext_serial_read(&c_byte, 1);
      if (len > 0 && (c_byte == '\n' || c_byte == '\r')) break;
    }

    return MENU_INVALID_OPTION;
  }

  if(index > 2) return MENU_INVALID_OPTION; // User input is 2 digit max

  return (menu_option_t) atoi(input);
}

void vTask_UIManager(void *pvParameters){
  while(1){
    print_menu();
    menu_option_t chosen_option = rx_menu_option();

    switch(chosen_option){
      case MENU_SHOW_MEASUREMENTS:
        ext_serial_print("a");
        ext_serial_print("\r\n");
        // TODO: Handle showing measurements
        break;

      case MENU_VIEW_STATS:
        ext_serial_print("b");
        ext_serial_print("\r\n");
        // TODO: Handle viewing statistics
        break;

      case MENU_RESET_STATS:
        ext_serial_print("c");
        ext_serial_print("\r\n");
        // TODO: Handle resetting statistics
        break;

      case MENU_VIEW_THRESHOLDS:
        ext_serial_print("d");
        ext_serial_print("\r\n");
        // TODO: Handle viewing thresholds
        break;

      case MENU_CHANGE_THRESHOLDS:
        ext_serial_print("e");
        ext_serial_print("\r\n");
        // TODO: Handle changing thresholds
        break;

      case MENU_INVALID_OPTION:
      default:
        ext_serial_print("Invalid option! Please enter a number within 1 to 5\r\n");
        break;
    }
  }
}
