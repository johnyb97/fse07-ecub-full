#ifndef LVBAT
#define LVBAT

#define BMS_CONVERSION_INTERVAL_MS 500
#define BMS_COMBINED_CONVERSION_MS 60

typedef enum {
	BMS_Fault,
	BMS_Ready,
	BMS_CombinedConversion,
} BMS_State_t;

void LV_init(void);
void LV_process(CAN_HandleTypeDef* hcan);
int LV_voltage_recive(void);
#endif
