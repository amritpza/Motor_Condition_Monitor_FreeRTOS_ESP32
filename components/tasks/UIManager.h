#ifndef UIMANAGER_H
#define UIMANAGER_H

#include "freertos/FreeRTOS.h"

#define BIT_MENU_SHOW_MEASUREMENTS  (1 << 0)
#define BIT_MENU_VIEW_STATS         (1 << 1)
#define BIT_MENU_RESET_STATS        (1 << 2)
#define BIT_FINISHED_MENU_RQ_OPTION (1 << 3)

typedef enum{
  MENU_INVALID_OPTION = 0,
  MENU_SHOW_MEASUREMENTS,
  MENU_VIEW_STATS,
  MENU_RESET_STATS,
  MENU_VIEW_THRESHOLDS,
  MENU_CHANGE_THRESHOLDS
} Menu_Option;

void vTask_UIManager(void *pvParameters);

esp_err_t menu_option_event_group_init();
esp_err_t serial_print_from_task_mutex_init();

EventBits_t rx_user_menu_option();
void set_bit_finished_menu_rq_option();

void serial_print_from_task(const char *msg);

#endif
