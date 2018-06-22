#ifndef CAN_ECUB_H
#define CAN_ECUB_H

#include <tx2/can.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    bus_CAN1_powertrain = 0,
    bus_CAN2_aux = 1,
};

enum { ECUA_Status_id = 0x090 };
enum { ECUA_Status_timeout = 500 };
enum { ECUB_Status_id = 0x0A0 };
enum { ECUB_Status_timeout = 1000 };
enum { ECUB_Wheelspeed_id = 0x1A3 };
enum { ECUB_Wheelspeed_timeout = 50 };
enum { ECUB_GLV_AMS_id = 0x3A2 };
enum { ECUB_Cooling_id = 0x4A4 };
enum { ECUB_Power_dist_id = 0x4A6 };
enum { ECUB_TEMPSuspR_id = 0x4A8 };
enum { ECUB_TEMPAux_id = 0x4AA };
enum { ECUF_Status_id = 0x0C0 };
enum { ECUF_Status_timeout = 1000 };
enum { ECUF_Dashboard_id = 0x3C5 };
enum { ECUP_Status_id = 0x040 };
enum { ECUP_Status_timeout = 500 };
enum { MCF_GeneralReport_id = 0x100 };
enum { MCF_ThermalMeasuresA_id = 0x303 };
enum { MCF_ThermalMeasuresB_id = 0x304 };
enum { MCR_GeneralReport_id = 0x110 };
enum { MCR_ThermalMeasuresA_id = 0x313 };
enum { MCR_ThermalMeasuresB_id = 0x314 };
enum { VDCU_Status_id = 0x050 };
enum { VDCU_Status_timeout = 500 };

enum ECUA_StateAMS {
    /* without_fault */
    ECUA_StateAMS_All_OK = 0,
    /* All_fucked */
    ECUA_StateAMS_SHIT = 1,
};

enum ECUP_CAL_PedalIndex {
    /* None */
    ECUP_CAL_PedalIndex_None = 0,
    /* Minimum apps position */
    ECUP_CAL_PedalIndex_AppsMin = 1,
    /* Maximum apps position */
    ECUP_CAL_PedalIndex_AppsMax = 2,
    /* Minimum brake position */
    ECUP_CAL_PedalIndex_BrakeMin = 3,
    /* Maximum brake position */
    ECUP_CAL_PedalIndex_BrakeMax = 4,
};

enum ECUP_CalibrationIndex {
    /* FIX THIS SHIT */
    ECUP_CalibrationIndex_dummy = 1,
};

enum ECUB_Batt_code {
    /* No power drawn nor charged */
    ECUB_Batt_code_IDLE = 0,
    /* Charging with balancing */
    ECUB_Batt_code_CHARGING = 1,
    /* Charging without balancing */
    ECUB_Batt_code_FAST_CHARGING = 2,
    /* Only balancing */
    ECUB_Batt_code_BALANCING = 3,
    /* Is being discharged */
    ECUB_Batt_code_DISCHARGING = 4,
    /* Fully charged */
    ECUB_Batt_code_FULL = 5,
    /* Is in error state */
    ECUB_Batt_code_ERROR = 6,
};

enum ECUB_CarState {
    /* SDC interrupted -> not ready for start */
    ECUB_CarState_NOT_READY = 0,
    /* Fatal error \w SDC latching */
    ECUB_CarState_LATCHED = 1,
    /* Ready for TSON button */
    ECUB_CarState_TS_READY = 2,
    /* ACP is being precharged -> waiting for ECUA status */
    ECUB_CarState_PRECHARGE = 3,
    /* Ready for START */
    ECUB_CarState_TS_ON = 4,
    /* Waiting for completion of RTDS */
    ECUB_CarState_WAITING_FOR_RTDS = 5,
    /* Drive! */
    ECUB_CarState_STARTED = 6,
};

enum ECUB_GLV_PowerSource {
    /* ACP */
    ECUB_GLV_PowerSource_ACP = 0,
    /* GLV battery */
    ECUB_GLV_PowerSource_GLV_BATTERY = 1,
    /* Service box input */
    ECUB_GLV_PowerSource_SERVICE_INPUT = 2,
};

enum ECUB_Notready_reason {
    /* No error */
    ECUB_Notready_reason_NONE = 0,
    /* Vehicle was latched at the startup */
    ECUB_Notready_reason_LATCH_START = 1,
    /* Vehicle was latched due to BSPD error */
    ECUB_Notready_reason_LATCH_BSPD = 2,
    /* Vehicle was latched due to AMS error */
    ECUB_Notready_reason_LATCH_AMS = 3,
    /* Error in SDC chain */
    ECUB_Notready_reason_SDC_FAILURE = 4,
    /* Motor controller CAN timeout */
    ECUB_Notready_reason_TIMEOUT_MC = 5,
    /* AMS CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUA = 6,
    /* ECU Front CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUF = 7,
    /* Pedal unit CAN timeout */
    ECUB_Notready_reason_TIMEOUT_ECUP = 8,
    /* VDCU CAN timeout */
    ECUB_Notready_reason_TIMEOUT_VDCU = 9,
    /* Fault on PWM_fault pins */
    ECUB_Notready_reason_PWM_FAULT = 10,
};

enum VDCU_CAL_DisIndex {
    /* None */
    VDCU_CAL_DisIndex_None = 0,
    /* Front right displacement sensor calibration */
    VDCU_CAL_DisIndex_DisFR = 1,
    /* Front left displacement sensor calibration */
    VDCU_CAL_DisIndex_DisFL = 2,
    /* Rear right displacement sensor calibration */
    VDCU_CAL_DisIndex_DisRR = 3,
    /* Rear left displacement sensor calibration */
    VDCU_CAL_DisIndex_DisRL = 4,
};

enum VDCU_Parameters {
    /* Torque gain (0-128) */
    VDCU_Parameters_TorqueGain = 0,
    /* Maximum power in kW (0-120) */
    VDCU_Parameters_PowerMax = 1,
    /* Maximum regenerative power in kW (0-40) */
    VDCU_Parameters_PowerMax_Gen = 2,
    /* Request for reverse [0,1] */
    VDCU_Parameters_Reverse_REQ = 3,
    /* Racing mode (accel, skid, autox, endu ...) (0-9) */
    VDCU_Parameters_Mode = 4,
    /* Request for not limited performance [0,1] */
    VDCU_Parameters_Turbo_REQ = 5,
    /* Torque distribution between front/rear (0-100) */
    VDCU_Parameters_Torque_Dist = 6,
    /* Torque vectoring gain on front axle (0-200) */
    VDCU_Parameters_TV_GainF = 7,
    /* Torque vectoring gain on rear axle (0-200) */
    VDCU_Parameters_TV_GainR = 8,
    /* Maximum torque for rear axle (0-32) */
    VDCU_Parameters_TorqueMaxF = 9,
    /* Maximum torque for front axle (0-96) */
    VDCU_Parameters_TorqueMaxR = 10,
    /* Slip ratio controller slip setpoint (4-20) */
    VDCU_Parameters_TC_SpSlip = 11,
    /* Slip ratio controller Kp gain (0-) */
    VDCU_Parameters_TC_Kp = 12,
    /* Slip ratio controller Ki gain (0-) */
    VDCU_Parameters_TC_Ki = 13,
    /* Yaw controller Kp gain (0-) */
    VDCU_Parameters_YC_Kp = 14,
    /* Yaw controller Ki gain (0-) */
    VDCU_Parameters_YC_Ki = 15,
    /* ENABLE torque vectoring [0,1] */
    VDCU_Parameters_TV_EN = 16,
    /* ENABLE Slip ratio controller [0,1] */
    VDCU_Parameters_TC_EN = 17,
    /* ENABLE Yaw controller [0,1] */
    VDCU_Parameters_YC_EN = 18,
    /* ENABLE regenerative braking [0,1] */
    VDCU_Parameters_GEN_EN = 19,
    /* Pedal map selector (0-4) */
    VDCU_Parameters_Ped_MAP = 20,
    /* Field weakening (0-100) */
    VDCU_Parameters_FW = 21,
};

enum ECUF_CAL_STWIndex {
    /* None */
    ECUF_CAL_STWIndex_None = 0,
    /* Left position (output is positive) */
    ECUF_CAL_STWIndex_STWLeft = 1,
    /* Center position */
    ECUF_CAL_STWIndex_STWCenter = 2,
    /* Right position (output is negative) */
    ECUF_CAL_STWIndex_STWRight = 3,
};

/*
 * Base status of ECUA and its subsystems (AMS)
 */
typedef struct ECUA_Status_t {
	/* Lead from ECUB (1 = open) */
	uint8_t	SDC_IN_Open;

	/* True if HV interlock is closed */
	uint8_t	SDC_HV_ILOCK;

	/* True if SDC is not broken by IMD */
	uint8_t	SDC_IMD;

	/* AMS = ECUA+BMS */
	uint8_t	SDC_AMS;

	/* SDC out to final stretch (BSPD etc...) */
	uint8_t	SDC_OUT;

	/* End of SDC (input to AIRS) */
	uint8_t	SDC_END;

	/* HW latch engaged (caused by IMD or AMS) */
	uint8_t	LATCH_SDC_AMS;

	/*  */
	uint8_t	AIRsState;

	/*  */
	uint8_t	ChargingState;

	/* Error state of AMS */
	enum ECUA_StateAMS	AMSState;

	/* Fault of ACP overtemperature */
	uint8_t	FT_ACP_OT;

	/* Fault of AIRs */
	uint8_t	FT_AIRS;

	/* Fault of DCDC GLV converter */
	uint8_t	FT_DCDC;

	/* FAN1 dead */
	uint8_t	FT_FAN1;

	/* FAN2 dead */
	uint8_t	FT_FAN2;

	/* FAN3 dead */
	uint8_t	FT_FAN3;

	/* Fault HV overvoltage */
	uint8_t	FT_HV_OV;

	/* Fault HV undervoltage */
	uint8_t	FT_HV_UV;

	/* Fault of GLV (undervoltage measurement) */
	uint8_t	FT_GLV_UV;

	/* Fault of GLV (overvoltage measurement) */
	uint8_t	FT_GLV_OV;

	/* Internal ECUA/BMS faults */
	uint8_t	FT_AMS;

	/* If any error is present */
	uint8_t	FT_ANY;

	/* Warning for cell tempreature (near limits) */
	uint8_t	WARN_TEMP_Cell;

	/* Warning for dc-dc temperature (near limits) */
	uint8_t	WARN_TEMP_DCDC;

	/* Warning for balancer temperature (near limits) */
	uint8_t	WARN_TEMP_Bal;

	/* ECUB powering output is enabled */
	uint8_t	PWR_ECUB;

	/* Fans enabled */
	uint8_t	FANS_EN;

	/* Message up counter for safety */
	uint8_t	SEQ;
} ECUA_Status_t;

/*
 * ECUB Status report
 */
typedef struct ECUB_Status_t {
	/* Shutdown circuit - Front */
	uint8_t	SDC_FRONT;

	/* Shutdown circuit - Shutdown button left */
	uint8_t	SDC_SDBL;

	/* Shutdown circuit - Shutdown button right */
	uint8_t	SDC_SDBR;

	/* Shutdown circuit - High voltage disconnect */
	uint8_t	SDC_HVD;

	/* Shutdown circuit - Brake plausibility device */
	uint8_t	SDC_BSPD;

	/* Shutdown circuit - Motor controller rear */
	uint8_t	SDC_MCUR;

	/* Shutdown circuit - Accumulator management system */
	uint8_t	SDC_AMS;

	/* Shutdown circuit - Tractive system master switch */
	uint8_t	SDC_TSMS;

	/* Current vehicle state */
	enum ECUB_CarState	CarState;

	/* Reason for latest not-ready CarState */
	enum ECUB_Notready_reason	CarState_Notready;

	/* Current powering source */
	enum ECUB_GLV_PowerSource	PowerSource;

	/* Module 1 detected */
	uint8_t	Det_MOD1;

	/* Module 2 detected */
	uint8_t	Det_MOD2;

	/* Module 3 detected */
	uint8_t	Det_MOD3;

	/* Module 4 detected */
	uint8_t	Det_MOD4;

	/* Fault temperature shutdown for driver on PWR_ECUF_EN, PWR_ECUA_EN */
	uint8_t	FT_PWR1_OT;

	/* Fault temperature shutdown for driver on PWR_ECUMF_EN, PWR_ECUMR_EN */
	uint8_t	FT_PWR2_OT;

	/* Fault temperature shutdown for driver on WP1_EN, WP2_EN */
	uint8_t	FT_PWR3_OT;

	/* Fault temperature shutdown for driver on EM, FAN3_EN */
	uint8_t	FT_PWR4_OT;

	/* Fault temperature shutdown for driver on FAN1_EN, FAN2_EN */
	uint8_t	FT_PWR5_OT;

	/* Fault temperature shutdown for driver on RTDS, SDB LED R i L, BrakeLight */
	uint8_t	FT_L2_OT;

	/* If any error present */
	uint8_t	FT_ANY;

	/* Fault temperature shutdown for driver on WS, AUX1,2,3 */
	uint8_t	FT_L1_OT;

	/* Fault ECUF power (Short to VCC) */
	uint8_t	FT_PWR_ECUF_OC;

	/* Fault ECUA power (Short to VCC) */
	uint8_t	FT_PWR_ECUA_OC;

	/* Fault MCF power (Short to VCC) */
	uint8_t	FT_PWR_MCF_OC;

	/* Fault MCR power (Short to VCC) */
	uint8_t	FT_PWR_MCR_OC;

	/* Fault of CAN Bus (CAN errors or controller mode is error) */
	uint8_t	FT_CAN1;

	/* Fault of CAN Bus (CAN errors or controller mode is error) */
	uint8_t	FT_CAN2;

	/* Power to ECUF enabled */
	uint8_t	PWR_ECUF_EN;

	/* Power to ECUA enabled */
	uint8_t	PWR_ECUA_EN;

	/* Power to MCUF enabled */
	uint8_t	PWR_MCUF_EN;

	/* Power to MCUR enabled */
	uint8_t	PWR_MCUR_EN;

	/* Power to Energy Meter enabled */
	uint8_t	PWR_EM_EN;

	/* Power to Waterpump 1 enabled */
	uint8_t	PWR_WP1_EN;

	/* Power to Waterpump 2 enabled */
	uint8_t	PWR_WP2_EN;

	/* Power to FAN1 enabled */
	uint8_t	PWR_FAN1_EN;

	/* Power to FAN2 enabled */
	uint8_t	PWR_FAN2_EN;

	/* Power to FAN3 enabled */
	uint8_t	PWR_FAN3_EN;

	/* Power to Wheel speed sensor enabled */
	uint8_t	PWR_WS_EN;

	/* Power to aux1 enabled aka LIDK1 (Low I Don't Know #1) */
	uint8_t	PWR_AUX1_EN;

	/* Power to aux2 enabled aka LIDK2 */
	uint8_t	PWR_AUX2_EN;

	/* Power to aux3 enabled aka LIDK3 */
	uint8_t	PWR_AUX3_EN;

	/* Ready to drive sound enabled */
	uint8_t	RTDS_EN;

	/* Shutdown button right led enabled */
	uint8_t	SDBR_LED_EN;

	/* Shutdown button left led enabled */
	uint8_t	SDBL_LED_EN;

	/* Brakelight enabled */
	uint8_t	BrakeLight_EN;

	/* TSAL ""Test"" or ""Override"" totally not a hack enabled */
	uint8_t	TSAL_Override;

	/* Message up counter for safety check */
	uint8_t	SEQ;
} ECUB_Status_t;

/*
 * Wheel speed measurement message
 */
typedef struct ECUB_Wheelspeed_t {
	/* Wheel speed rear right */
	int16_t	WhR;

	/* Wheel speed rear left */
	int16_t	WhL;

	/* Timestamp of measurement */
	uint16_t	Timestamp;

	/* Fault of rear right wheel speed sensor */
	uint8_t	FT_WhR;

	/* Fault of rear left wheel speed sensor */
	uint8_t	FT_WhL;

	/* Message up counter for safety check */
	uint8_t	SEQ;
} ECUB_Wheelspeed_t;

/*
 * GLV Accumulator management system
 */
typedef struct ECUB_GLV_AMS_t {
	/* GLV Battery status */
	enum ECUB_Batt_code	BattState;

	/* GLV Battery fault (overvoltage, undervoltage, overcurrent, overtemperature) */
	uint8_t	FT_Batt;

	/* GLV battery AMS fault */
	uint8_t	FT_AMS;

	/* GLV battery Charger fault */
	uint8_t	FT_Charger;

	/* GLV Battery voltage */
	uint16_t	Volt;

	/* GLV Battery current */
	uint8_t	Curr;

	/* Voltage GLV battery mux */
	uint8_t	CellID;

	/* Battery cell voltage */
	uint16_t	Volt_cell;

	/* Battery cell temperature */
	uint8_t	Temp_cell;
} ECUB_GLV_AMS_t;

/*
 * PWM for each fan and waterpump
 */
typedef struct ECUB_Cooling_t {
	/* Waterpump cooler circuit PWM duty */
	uint8_t	WP1;

	/* Waterpump cooler circuit PWM duty */
	uint8_t	WP2;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN1;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN2;

	/* Cooler FAN in PWM duty */
	uint8_t	FAN3;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_FR_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_FL_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_RR_TEMP;

	/* Warning - Motor temperature over nominal threshold */
	uint8_t	WARN_MOT_RL_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_FR_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_FL_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_RR_TEMP;

	/* Warning - Motor controller temperature over nominal threshold */
	uint8_t	WARN_MCU_RL_TEMP;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_FR_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_FL_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_RR_OT;

	/* Fault - motor overtemperature */
	uint8_t	FT_MOT_RL_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_FR_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_FL_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_RR_OT;

	/* Fault - motor controller overtemperature */
	uint8_t	FT_MCU_RL_OT;
} ECUB_Cooling_t;

/*
 * Power distribution message measurement
 * U_ECUA
 * U_Service_box
 * 
 */
typedef struct ECUB_Power_dist_t {
	/* DCDC power rail voltage */
	uint8_t	Volt_DCDC;

	/* Service box 24V input voltage */
	uint8_t	Volt_SBox;

	/* Current for ECUF */
	uint8_t	Curr_ECUF;

	/* Current for ECUA */
	uint8_t	Curr_ECUA;

	/* Current for ECUMF */
	uint8_t	Curr_ECUMF;

	/* Current for ECUMR */
	uint8_t	Curr_ECUMR;
} ECUB_Power_dist_t;

/*
 * Rear suspension temperatures
 * 
 */
typedef struct ECUB_TEMPSuspR_t {
	/* Rear right brake caliper temperature */
	uint8_t	BrakeCal_RR;

	/* Rear left brake caliper temperature */
	uint8_t	BrakeCal_RL;

	/* Rear right tire inboard temperature */
	uint8_t	TireI_RR;

	/* Rear right tire center temperature */
	uint8_t	TireC_RR;

	/* Rear right tire outboard temperature */
	uint8_t	TireO_RR;

	/* Rear left tire inboard temperature */
	uint8_t	TireI_RL;

	/* Rear left tire center temperature */
	uint8_t	TireC_RL;

	/* Rear left tire inboard temperature */
	uint8_t	TireO_RL;
} ECUB_TEMPSuspR_t;

/*
 * Auxiliary temperature measurements - cooling system
 */
typedef struct ECUB_TEMPAux_t {
	/*  */
	uint8_t	Cooling1_NTC;

	/*  */
	uint8_t	Cooling2_NTC;

	/*  */
	uint8_t	Cooling3_NTC;

	/*  */
	uint8_t	Cooling4_NTC;
} ECUB_TEMPAux_t;

/*
 * Summary of unit status and faults
 */
typedef struct ECUF_Status_t {
	/* Shutdown circuit from SDB_Cockpit */
	uint8_t	SDC_SDBC;

	/* Inertia switch shutdown circuit */
	uint8_t	SDC_Inertia;

	/* Front Wheel Interlock (Connetore Italiano Supreme) */
	uint8_t	SDC_FWIL;

	/* Power to ECUP is enabled */
	uint8_t	PWR_ECUP_EN;

	/* Power to ECUG is enabled */
	uint8_t	PWR_ECUG_EN;

	/* Power to DTLG is enabled */
	uint8_t	PWR_DTLG_EN;

	/* Power to ECUS is enabled */
	uint8_t	PWR_ECUS_EN;

	/* Power to Dash is enabled */
	uint8_t	PWR_DASH_EN;

	/* Power to front brake fans is enabled */
	uint8_t	PWR_FAN_BrakeF_EN;

	/* ECUP enabled but no current is drawn */
	uint8_t	FT_PWR_ECUP;

	/* ECUG enabled but no current is drawn */
	uint8_t	FT_PWR_ECUG;

	/* ECUS enabled but no current is drawn */
	uint8_t	FT_PWR_ECUS;

	/* DTLG enabled but no current is drawn */
	uint8_t	FT_PWR_DTLG;

	/* DASH enabled but no current is drawn */
	uint8_t	FT_PWR_DASH;

	/* FAN enabled but no current is drawn */
	uint8_t	FT_PWR_FAN_BrakeF;

	/* STW sensor disconnected or destroyed */
	uint8_t	FT_STW_Sensor;

	/* Fault of steering wheel calibration */
	uint8_t	FT_STW_Cal;

	/* Fault of suspension displacement sensor FR */
	uint8_t	FT_DisFR;

	/* Fault of suspension displacement sensor FL */
	uint8_t	FT_DisFL;

	/* Fault of suspension displacement sensor RR */
	uint8_t	FT_DisRR;

	/* Fault of suspension displacement sensor RL */
	uint8_t	FT_DisRL;

	/* Fault of suspension displacement sensor FR calibration */
	uint8_t	FT_DisFR_Cal;

	/* Fault of suspension displacement sensor FL calibration */
	uint8_t	FT_DisFL_Cal;

	/* Fault of suspension displacement sensor RR calibration */
	uint8_t	FT_DisRR_Cal;

	/* Fault of suspension displacement sensor RL calibration */
	uint8_t	FT_DisRL_Cal;

	/* GLV Voltage at ECUF input */
	uint8_t	Volt_GLV_In;
} ECUF_Status_t;

/*
 * Dashboard buttons & switches, but not only that :thinking:
 */
typedef struct ECUF_Dashboard_t {
	/* 1-pressed / 0-not pressed */
	uint8_t	TSON;

	/* 1-pressed / 0-not pressed */
	uint8_t	START;

	/* 1-pressed / 0-not pressed */
	uint8_t	SW1;

	/* 1-pressed / 0-not pressed */
	uint8_t	SW2;

	/* 1-pressed / 0-not pressed */
	uint8_t	SW3;

	/* Ambient light level */
	uint8_t	AmbientLight;

	/* Ambient temperature */
	uint8_t	AmbientTemp;
} ECUF_Dashboard_t;

/*
 * ECUP Status message
 */
typedef struct ECUP_Status_t {
	/* SDC is powered after BOTS */
	uint8_t	SDC_BOTS;

	/* If any error is present */
	uint8_t	FT_ANY;

	/* Accelerator pedals plausible */
	uint8_t	APPS_Plausible;

	/* Brake Pedal Plausibility Check (software bspd) */
	uint8_t	BPPC_Latch;

	/* Brake active (Signal mainly for brakelight) */
	uint8_t	BrakeActive;

	/* Brake active for the purposes of BSPD */
	uint8_t	BrakeActive_BSPD;
} ECUP_Status_t;

/*
 * Actual settings and states, system faults.
 */
typedef struct MCF_GeneralReport_t {
	/* Shutdown circuit - point at input. */
	uint8_t	SDC_IN;

	/* Shutdown circuit - point after channel B motor's sensor connector. */
	uint8_t	SDC_MSCB;

	/* Shutdown circuit - point after channel B motor's power connector. */
	uint8_t	SDC_MPCB;

	/* Shutdown circuit - point after high voltage connector. */
	uint8_t	SDC_HVC;

	/* Shutdown circuit - point after channel A motor's power connector. */
	uint8_t	SDC_MPCA;

	/* Shutdown circuit - point after channel A motor's sensor connector. */
	uint8_t	SDC_MSCA;

	/* True means discharge resistor is connected. */
	uint8_t	DISCH;

	/* True means active auxiliary power output channel A. */
	uint8_t	POA;

	/* Power supply for POA. log.1 = 12V, log.0 = input voltage (24V). */
	uint8_t	POA_PS;

	/* True means active auxiliary power output channel B. */
	uint8_t	POB;

	/* Power supply for POB. log.1 = 12V, log.0 = input voltage (24V). */
	uint8_t	POB_PS;

	/* True means channel A PWM is active. */
	uint8_t	PWMA;

	/* True means field weakening is allowed for channel A. */
	uint8_t	FWA;

	/* True means generation mode is allowed for channel A. */
	uint8_t	GENA;

	/* Default direction for positive requests values.  */
	uint8_t	DIRA;

	/* True means channel B PWM is active. */
	uint8_t	PWMB;

	/* True means field weakening is allowed for channel B. */
	uint8_t	FWB;

	/* True means generation mode is allowed for channel B. */
	uint8_t	GENB;

	/* Default direction for positive requests values.  */
	uint8_t	DIRB;

	/* Heart beat. */
	uint8_t	HB;
} MCF_GeneralReport_t;

/*
 * Thermal measures for channel A.
 */
typedef struct MCF_ThermalMeasuresA_t {
	/* Temperature of IGBT's heat sink. */
	uint8_t	TIGBT;

	/* Temperature estimation of IGBT chip. */
	uint8_t	TIGBTJ;

	/* Temperature of motor's connector. */
	uint8_t	TMOTCON;

	/* Temperature of motor's temp. sensor. */
	uint8_t	TMOTSEN;

	/* Temperature estimation of motor's winding. */
	uint8_t	TMOTWIN;

	/* Temperature of capacitor bank. */
	uint8_t	TCAP;

	/* Motor performance capacity utilization rate. */
	uint8_t	TMOTI2T;
} MCF_ThermalMeasuresA_t;

/*
 * Thermal measures for channel B.
 */
typedef struct MCF_ThermalMeasuresB_t {
	/* Temperature of IGBT's heat sink. */
	uint8_t	TIGBT;

	/* Temperature estimation of IGBT chip. */
	uint8_t	TIGBTJ;

	/* Temperature of motor's connector. */
	uint8_t	TMOTCON;

	/* Temperature of motor's temp. sensor. */
	uint8_t	TMOTSEN;

	/* Temperature estimation of motor's winding. */
	uint8_t	TMOTWIN;

	/* Temperature of capacitor bank. */
	uint8_t	TCAP;

	/* Motor performance capacity utilization rate. */
	uint8_t	TMOTI2T;
} MCF_ThermalMeasuresB_t;

/*
 * Actual settings and states, system faults.
 */
typedef struct MCR_GeneralReport_t {
	/* Shutdown circuit - point at input. */
	uint8_t	SDC_IN;

	/* Shutdown circuit - point after channel B motor's sensor connector. */
	uint8_t	SDC_MSCB;

	/* Shutdown circuit - point after channel B motor's power connector. */
	uint8_t	SDC_MPCB;

	/* Shutdown circuit - point after high voltage connector. */
	uint8_t	SDC_HVC;

	/* Shutdown circuit - point after channel A motor's power connector. */
	uint8_t	SDC_MPCA;

	/* Shutdown circuit - point after channel A motor's sensor connector. */
	uint8_t	SDC_MSCA;

	/* True means discharge resistor is connected. */
	uint8_t	DISCH;

	/* True means active auxiliary power output channel A. */
	uint8_t	POA;

	/* Power supply for POA. log.1 = 12V, log.0 = input voltage (24V). */
	uint8_t	POA_PS;

	/* True means active auxiliary power output channel B. */
	uint8_t	POB;

	/* Power supply for POB. log.1 = 12V, log.0 = input voltage (24V). */
	uint8_t	POB_PS;

	/* True means channel A PWM is active. */
	uint8_t	PWMA;

	/* True means field weakening is allowed for channel A. */
	uint8_t	FWA;

	/* True means generation mode is allowed for channel A. */
	uint8_t	GENA;

	/* Default direction for positive requests values.  */
	uint8_t	DIRA;

	/* True means channel B PWM is active. */
	uint8_t	PWMB;

	/* True means field weakening is allowed for channel B. */
	uint8_t	FWB;

	/* True means generation mode is allowed for channel B. */
	uint8_t	GENB;

	/* Default direction for positive requests values.  */
	uint8_t	DIRB;

	/* Heart beat. */
	uint8_t	HB;
} MCR_GeneralReport_t;

/*
 * Thermal measures for channel A.
 */
typedef struct MCR_ThermalMeasuresA_t {
	/* Temperature of IGBT's heat sink. */
	uint8_t	TIGBT;

	/* Temperature estimation of IGBT chip. */
	uint8_t	TIGBTJ;

	/* Temperature of motor's connector. */
	uint8_t	TMOTCON;

	/* Temperature of motor's temp. sensor. */
	uint8_t	TMOTSEN;

	/* Temperature estimation of motor's winding. */
	uint8_t	TMOTWIN;

	/* Temperature of capacitor bank. */
	uint8_t	TCAP;

	/* Motor performance capacity utilization rate. */
	uint8_t	TMOTI2T;
} MCR_ThermalMeasuresA_t;

/*
 * Thermal measures for channel B.
 */
typedef struct MCR_ThermalMeasuresB_t {
	/* Temperature of IGBT's heat sink. */
	uint8_t	TIGBT;

	/* Temperature estimation of IGBT chip. */
	uint8_t	TIGBTJ;

	/* Temperature of motor's connector. */
	uint8_t	TMOTCON;

	/* Temperature of motor's temp. sensor. */
	uint8_t	TMOTSEN;

	/* Temperature estimation of motor's winding. */
	uint8_t	TMOTWIN;

	/* Temperature of capacitor bank. */
	uint8_t	TCAP;

	/* Motor performance capacity utilization rate. */
	uint8_t	TMOTI2T;
} MCR_ThermalMeasuresB_t;

/*
 * VDCU systems status
 */
typedef struct VDCU_Status_t {
	/*  */
	uint8_t	State;

	/* Fault of displacement sensor calibration */
	uint8_t	FT_Dis_Cal;

	/*  */
	uint8_t	FT_Sensor;

	/* Derating due to components temperature */
	uint8_t	Temp_derating;

	/* Derating due to ACP limits */
	uint8_t	ACP_derate;

	/* Discharge is active */
	uint8_t	Disch_ACT;

	/* Reverse is activated */
	uint8_t	Reverse_ACT;

	/* Torque vectoring enabled */
	uint8_t	TV_ENABLED;

	/* Slip ratio control enabled (traction control) */
	uint8_t	TC_ENABLED;

	/* Yaw controller enabled */
	uint8_t	YC_ENABLED;

	/* Slip ratio control is limiting torque */
	uint8_t	TC_ACT;

	/* Yaw controller is producing yaw torque */
	uint8_t	YC_ACT;
} VDCU_Status_t;

void candbInit(void);

int ECUA_decode_Status_s(const uint8_t* bytes, size_t length, ECUA_Status_t* data_out);
int ECUA_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_Open_out, uint8_t* SDC_HV_ILOCK_out, uint8_t* SDC_IMD_out, uint8_t* SDC_AMS_out, uint8_t* SDC_OUT_out, uint8_t* SDC_END_out, uint8_t* LATCH_SDC_AMS_out, uint8_t* AIRsState_out, uint8_t* ChargingState_out, enum ECUA_StateAMS* AMSState_out, uint8_t* FT_ACP_OT_out, uint8_t* FT_AIRS_out, uint8_t* FT_DCDC_out, uint8_t* FT_FAN1_out, uint8_t* FT_FAN2_out, uint8_t* FT_FAN3_out, uint8_t* FT_HV_OV_out, uint8_t* FT_HV_UV_out, uint8_t* FT_GLV_UV_out, uint8_t* FT_GLV_OV_out, uint8_t* FT_AMS_out, uint8_t* FT_ANY_out, uint8_t* WARN_TEMP_Cell_out, uint8_t* WARN_TEMP_DCDC_out, uint8_t* WARN_TEMP_Bal_out, uint8_t* PWR_ECUB_out, uint8_t* FANS_EN_out, uint8_t* SEQ_out);
int ECUA_get_Status(ECUA_Status_t* data_out);
void ECUA_Status_on_receive(int (*callback)(ECUA_Status_t* data));

int ECUB_send_Status_s(const ECUB_Status_t* data);
int ECUB_send_Status(uint8_t SDC_FRONT, uint8_t SDC_SDBL, uint8_t SDC_SDBR, uint8_t SDC_HVD, uint8_t SDC_BSPD, uint8_t SDC_MCUR, uint8_t SDC_AMS, uint8_t SDC_TSMS, enum ECUB_CarState CarState, enum ECUB_Notready_reason CarState_Notready, enum ECUB_GLV_PowerSource PowerSource, uint8_t Det_MOD1, uint8_t Det_MOD2, uint8_t Det_MOD3, uint8_t Det_MOD4, uint8_t FT_PWR1_OT, uint8_t FT_PWR2_OT, uint8_t FT_PWR3_OT, uint8_t FT_PWR4_OT, uint8_t FT_PWR5_OT, uint8_t FT_L2_OT, uint8_t FT_ANY, uint8_t FT_L1_OT, uint8_t FT_PWR_ECUF_OC, uint8_t FT_PWR_ECUA_OC, uint8_t FT_PWR_MCF_OC, uint8_t FT_PWR_MCR_OC, uint8_t FT_CAN1, uint8_t FT_CAN2, uint8_t PWR_ECUF_EN, uint8_t PWR_ECUA_EN, uint8_t PWR_MCUF_EN, uint8_t PWR_MCUR_EN, uint8_t PWR_EM_EN, uint8_t PWR_WP1_EN, uint8_t PWR_WP2_EN, uint8_t PWR_FAN1_EN, uint8_t PWR_FAN2_EN, uint8_t PWR_FAN3_EN, uint8_t PWR_WS_EN, uint8_t PWR_AUX1_EN, uint8_t PWR_AUX2_EN, uint8_t PWR_AUX3_EN, uint8_t RTDS_EN, uint8_t SDBR_LED_EN, uint8_t SDBL_LED_EN, uint8_t BrakeLight_EN, uint8_t TSAL_Override, uint8_t SEQ);
int ECUB_Status_need_to_send(void);

int ECUB_send_Wheelspeed_s(const ECUB_Wheelspeed_t* data);
int ECUB_send_Wheelspeed(int16_t WhR, int16_t WhL, uint16_t Timestamp, uint8_t FT_WhR, uint8_t FT_WhL, uint8_t SEQ);
int ECUB_Wheelspeed_need_to_send(void);

int ECUB_send_GLV_AMS_s(const ECUB_GLV_AMS_t* data);
int ECUB_send_GLV_AMS(enum ECUB_Batt_code BattState, uint8_t FT_Batt, uint8_t FT_AMS, uint8_t FT_Charger, uint16_t Volt, uint8_t Curr, uint8_t CellID, uint16_t Volt_cell, uint8_t Temp_cell);
int ECUB_GLV_AMS_need_to_send(void);

int ECUB_send_Cooling_s(const ECUB_Cooling_t* data);
int ECUB_send_Cooling(uint8_t WP1, uint8_t WP2, uint8_t FAN1, uint8_t FAN2, uint8_t FAN3, uint8_t WARN_MOT_FR_TEMP, uint8_t WARN_MOT_FL_TEMP, uint8_t WARN_MOT_RR_TEMP, uint8_t WARN_MOT_RL_TEMP, uint8_t WARN_MCU_FR_TEMP, uint8_t WARN_MCU_FL_TEMP, uint8_t WARN_MCU_RR_TEMP, uint8_t WARN_MCU_RL_TEMP, uint8_t FT_MOT_FR_OT, uint8_t FT_MOT_FL_OT, uint8_t FT_MOT_RR_OT, uint8_t FT_MOT_RL_OT, uint8_t FT_MCU_FR_OT, uint8_t FT_MCU_FL_OT, uint8_t FT_MCU_RR_OT, uint8_t FT_MCU_RL_OT);
int ECUB_Cooling_need_to_send(void);

int ECUB_send_Power_dist_s(const ECUB_Power_dist_t* data);
int ECUB_send_Power_dist(uint8_t Volt_DCDC, uint8_t Volt_SBox, uint8_t Curr_ECUF, uint8_t Curr_ECUA, uint8_t Curr_ECUMF, uint8_t Curr_ECUMR);
int ECUB_Power_dist_need_to_send(void);

int ECUB_send_TEMPSuspR_s(const ECUB_TEMPSuspR_t* data);
int ECUB_send_TEMPSuspR(uint8_t BrakeCal_RR, uint8_t BrakeCal_RL, uint8_t TireI_RR, uint8_t TireC_RR, uint8_t TireO_RR, uint8_t TireI_RL, uint8_t TireC_RL, uint8_t TireO_RL);
int ECUB_TEMPSuspR_need_to_send(void);

int ECUB_send_TEMPAux_s(const ECUB_TEMPAux_t* data);
int ECUB_send_TEMPAux(uint8_t Cooling1_NTC, uint8_t Cooling2_NTC, uint8_t Cooling3_NTC, uint8_t Cooling4_NTC);
int ECUB_TEMPAux_need_to_send(void);

int ECUF_decode_Status_s(const uint8_t* bytes, size_t length, ECUF_Status_t* data_out);
int ECUF_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_SDBC_out, uint8_t* SDC_Inertia_out, uint8_t* SDC_FWIL_out, uint8_t* PWR_ECUP_EN_out, uint8_t* PWR_ECUG_EN_out, uint8_t* PWR_DTLG_EN_out, uint8_t* PWR_ECUS_EN_out, uint8_t* PWR_DASH_EN_out, uint8_t* PWR_FAN_BrakeF_EN_out, uint8_t* FT_PWR_ECUP_out, uint8_t* FT_PWR_ECUG_out, uint8_t* FT_PWR_ECUS_out, uint8_t* FT_PWR_DTLG_out, uint8_t* FT_PWR_DASH_out, uint8_t* FT_PWR_FAN_BrakeF_out, uint8_t* FT_STW_Sensor_out, uint8_t* FT_STW_Cal_out, uint8_t* FT_DisFR_out, uint8_t* FT_DisFL_out, uint8_t* FT_DisRR_out, uint8_t* FT_DisRL_out, uint8_t* FT_DisFR_Cal_out, uint8_t* FT_DisFL_Cal_out, uint8_t* FT_DisRR_Cal_out, uint8_t* FT_DisRL_Cal_out, uint8_t* Volt_GLV_In_out);
int ECUF_get_Status(ECUF_Status_t* data_out);
void ECUF_Status_on_receive(int (*callback)(ECUF_Status_t* data));

int ECUF_decode_Dashboard_s(const uint8_t* bytes, size_t length, ECUF_Dashboard_t* data_out);
int ECUF_decode_Dashboard(const uint8_t* bytes, size_t length, uint8_t* TSON_out, uint8_t* START_out, uint8_t* SW1_out, uint8_t* SW2_out, uint8_t* SW3_out, uint8_t* AmbientLight_out, uint8_t* AmbientTemp_out);
int ECUF_get_Dashboard(ECUF_Dashboard_t* data_out);
void ECUF_Dashboard_on_receive(int (*callback)(ECUF_Dashboard_t* data));

int ECUP_decode_Status_s(const uint8_t* bytes, size_t length, ECUP_Status_t* data_out);
int ECUP_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_BOTS_out, uint8_t* FT_ANY_out, uint8_t* APPS_Plausible_out, uint8_t* BPPC_Latch_out, uint8_t* BrakeActive_out, uint8_t* BrakeActive_BSPD_out);
int ECUP_get_Status(ECUP_Status_t* data_out);
void ECUP_Status_on_receive(int (*callback)(ECUP_Status_t* data));

int MCF_decode_GeneralReport_s(const uint8_t* bytes, size_t length, MCF_GeneralReport_t* data_out);
int MCF_decode_GeneralReport(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_out, uint8_t* SDC_MSCB_out, uint8_t* SDC_MPCB_out, uint8_t* SDC_HVC_out, uint8_t* SDC_MPCA_out, uint8_t* SDC_MSCA_out, uint8_t* DISCH_out, uint8_t* POA_out, uint8_t* POA_PS_out, uint8_t* POB_out, uint8_t* POB_PS_out, uint8_t* PWMA_out, uint8_t* FWA_out, uint8_t* GENA_out, uint8_t* DIRA_out, uint8_t* PWMB_out, uint8_t* FWB_out, uint8_t* GENB_out, uint8_t* DIRB_out, uint8_t* HB_out);
int MCF_get_GeneralReport(MCF_GeneralReport_t* data_out);
void MCF_GeneralReport_on_receive(int (*callback)(MCF_GeneralReport_t* data));

int MCF_decode_ThermalMeasuresA_s(const uint8_t* bytes, size_t length, MCF_ThermalMeasuresA_t* data_out);
int MCF_decode_ThermalMeasuresA(const uint8_t* bytes, size_t length, uint8_t* TIGBT_out, uint8_t* TIGBTJ_out, uint8_t* TMOTCON_out, uint8_t* TMOTSEN_out, uint8_t* TMOTWIN_out, uint8_t* TCAP_out, uint8_t* TMOTI2T_out);
int MCF_get_ThermalMeasuresA(MCF_ThermalMeasuresA_t* data_out);
void MCF_ThermalMeasuresA_on_receive(int (*callback)(MCF_ThermalMeasuresA_t* data));

int MCF_decode_ThermalMeasuresB_s(const uint8_t* bytes, size_t length, MCF_ThermalMeasuresB_t* data_out);
int MCF_decode_ThermalMeasuresB(const uint8_t* bytes, size_t length, uint8_t* TIGBT_out, uint8_t* TIGBTJ_out, uint8_t* TMOTCON_out, uint8_t* TMOTSEN_out, uint8_t* TMOTWIN_out, uint8_t* TCAP_out, uint8_t* TMOTI2T_out);
int MCF_get_ThermalMeasuresB(MCF_ThermalMeasuresB_t* data_out);
void MCF_ThermalMeasuresB_on_receive(int (*callback)(MCF_ThermalMeasuresB_t* data));

int MCR_decode_GeneralReport_s(const uint8_t* bytes, size_t length, MCR_GeneralReport_t* data_out);
int MCR_decode_GeneralReport(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_out, uint8_t* SDC_MSCB_out, uint8_t* SDC_MPCB_out, uint8_t* SDC_HVC_out, uint8_t* SDC_MPCA_out, uint8_t* SDC_MSCA_out, uint8_t* DISCH_out, uint8_t* POA_out, uint8_t* POA_PS_out, uint8_t* POB_out, uint8_t* POB_PS_out, uint8_t* PWMA_out, uint8_t* FWA_out, uint8_t* GENA_out, uint8_t* DIRA_out, uint8_t* PWMB_out, uint8_t* FWB_out, uint8_t* GENB_out, uint8_t* DIRB_out, uint8_t* HB_out);
int MCR_get_GeneralReport(MCR_GeneralReport_t* data_out);
void MCR_GeneralReport_on_receive(int (*callback)(MCR_GeneralReport_t* data));

int MCR_decode_ThermalMeasuresA_s(const uint8_t* bytes, size_t length, MCR_ThermalMeasuresA_t* data_out);
int MCR_decode_ThermalMeasuresA(const uint8_t* bytes, size_t length, uint8_t* TIGBT_out, uint8_t* TIGBTJ_out, uint8_t* TMOTCON_out, uint8_t* TMOTSEN_out, uint8_t* TMOTWIN_out, uint8_t* TCAP_out, uint8_t* TMOTI2T_out);
int MCR_get_ThermalMeasuresA(MCR_ThermalMeasuresA_t* data_out);
void MCR_ThermalMeasuresA_on_receive(int (*callback)(MCR_ThermalMeasuresA_t* data));

int MCR_decode_ThermalMeasuresB_s(const uint8_t* bytes, size_t length, MCR_ThermalMeasuresB_t* data_out);
int MCR_decode_ThermalMeasuresB(const uint8_t* bytes, size_t length, uint8_t* TIGBT_out, uint8_t* TIGBTJ_out, uint8_t* TMOTCON_out, uint8_t* TMOTSEN_out, uint8_t* TMOTWIN_out, uint8_t* TCAP_out, uint8_t* TMOTI2T_out);
int MCR_get_ThermalMeasuresB(MCR_ThermalMeasuresB_t* data_out);
void MCR_ThermalMeasuresB_on_receive(int (*callback)(MCR_ThermalMeasuresB_t* data));

int VDCU_decode_Status_s(const uint8_t* bytes, size_t length, VDCU_Status_t* data_out);
int VDCU_decode_Status(const uint8_t* bytes, size_t length, uint8_t* State_out, uint8_t* FT_Dis_Cal_out, uint8_t* FT_Sensor_out, uint8_t* Temp_derating_out, uint8_t* ACP_derate_out, uint8_t* Disch_ACT_out, uint8_t* Reverse_ACT_out, uint8_t* TV_ENABLED_out, uint8_t* TC_ENABLED_out, uint8_t* YC_ENABLED_out, uint8_t* TC_ACT_out, uint8_t* YC_ACT_out);
int VDCU_get_Status(VDCU_Status_t* data_out);
void VDCU_Status_on_receive(int (*callback)(VDCU_Status_t* data));

#ifdef __cplusplus
}
#endif

#endif
