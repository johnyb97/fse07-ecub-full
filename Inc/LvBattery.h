#ifndef LVBAT
#define LVBAT

void LV_init(void);
void LV_process(CAN_HandleTypeDef* hcan);
int LV_voltage_recive(void);
#endif
