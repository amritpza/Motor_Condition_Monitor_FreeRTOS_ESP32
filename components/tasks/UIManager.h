#ifndef UIMANAGER_H
#define UIMANAGER_H

typedef enum{
  MENU_INVALID_OPTION = 0,
  MENU_SHOW_MEASUREMENTS,
  MENU_VIEW_STATS,
  MENU_RESET_STATS,
  MENU_VIEW_THRESHOLDS,
  MENU_CHANGE_THRESHOLDS
} menu_option_t;

void vTask_UIManager(void *pvParameters);

#endif
