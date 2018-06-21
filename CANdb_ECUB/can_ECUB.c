#include "can_ECUB.h"
#include <string.h>

CAN_msg_status_t ECUA_Status_status;
ECUA_Status_t ECUA_Status_data;
int32_t ECUB_Status_last_sent;
int32_t ECUB_Wheelspeed_last_sent;
int32_t ECUB_GLV_AMS_last_sent;
int32_t ECUB_Cooling_last_sent;
int32_t ECUB_Power_dist_last_sent;
int32_t ECUB_TEMPSuspR_last_sent;
int32_t ECUB_TEMPAux_last_sent;
CAN_msg_status_t ECUF_Status_status;
ECUF_Status_t ECUF_Status_data;
CAN_msg_status_t ECUF_Dashboard_status;
ECUF_Dashboard_t ECUF_Dashboard_data;
CAN_msg_status_t ECUP_Status_status;
ECUP_Status_t ECUP_Status_data;
CAN_msg_status_t MCF_GeneralReport_status;
MCF_GeneralReport_t MCF_GeneralReport_data;
CAN_msg_status_t MCR_GeneralReport_status;
MCR_GeneralReport_t MCR_GeneralReport_data;
CAN_msg_status_t VDCU_Status_status;
VDCU_Status_t VDCU_Status_data;

void candbInit(void) {
    canInitMsgStatus(&ECUA_Status_status, 500);
    ECUB_Status_last_sent = -1;
    ECUB_Wheelspeed_last_sent = -1;
    ECUB_GLV_AMS_last_sent = -1;
    ECUB_Cooling_last_sent = -1;
    ECUB_Power_dist_last_sent = -1;
    ECUB_TEMPSuspR_last_sent = -1;
    ECUB_TEMPAux_last_sent = -1;
    canInitMsgStatus(&ECUF_Status_status, 1000);
    canInitMsgStatus(&ECUF_Dashboard_status, -1);
    canInitMsgStatus(&ECUP_Status_status, 500);
    canInitMsgStatus(&MCF_GeneralReport_status, -1);
    canInitMsgStatus(&MCR_GeneralReport_status, -1);
    canInitMsgStatus(&VDCU_Status_status, 500);
}

int ECUA_decode_Status_s(const uint8_t* bytes, size_t length, ECUA_Status_t* data_out) {
    if (length < 7)
        return 0;

    data_out->SDC_IN_Open = (bytes[0] & 0x01);
    data_out->SDC_HV_ILOCK = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_IMD = ((bytes[0] >> 2) & 0x01);
    data_out->SDC_AMS = ((bytes[0] >> 3) & 0x01);
    data_out->SDC_OUT = ((bytes[0] >> 4) & 0x01);
    data_out->SDC_END = ((bytes[0] >> 5) & 0x01);
    data_out->LATCH_SDC_AMS = ((bytes[0] >> 7) & 0x01);
    data_out->AIRsState = (bytes[1] & 0x0F);
    data_out->ChargingState = ((bytes[1] >> 4) & 0x0F);
    data_out->AMSState = (enum ECUA_StateAMS) (bytes[2]);
    data_out->FT_ACP_OT = (bytes[3] & 0x01);
    data_out->FT_AIRS = ((bytes[3] >> 1) & 0x01);
    data_out->FT_DCDC = ((bytes[3] >> 2) & 0x01);
    data_out->FT_FAN1 = ((bytes[3] >> 3) & 0x01);
    data_out->FT_FAN2 = ((bytes[3] >> 4) & 0x01);
    data_out->FT_FAN3 = ((bytes[3] >> 5) & 0x01);
    data_out->FT_HV_OV = ((bytes[3] >> 6) & 0x01);
    data_out->FT_HV_UV = ((bytes[3] >> 7) & 0x01);
    data_out->FT_GLV_UV = (bytes[4] & 0x01);
    data_out->FT_GLV_OV = ((bytes[4] >> 1) & 0x01);
    data_out->FT_AMS = ((bytes[4] >> 2) & 0x01);
    data_out->FT_ANY = ((bytes[4] >> 3) & 0x01);
    data_out->WARN_TEMP_Cell = (bytes[5] & 0x01);
    data_out->WARN_TEMP_DCDC = ((bytes[5] >> 1) & 0x01);
    data_out->WARN_TEMP_Bal = ((bytes[5] >> 2) & 0x01);
    data_out->PWR_ECUB = ((bytes[5] >> 6) & 0x01);
    data_out->FANS_EN = ((bytes[5] >> 7) & 0x01);
    data_out->SEQ = ((bytes[6] >> 4) & 0x0F);
    return 1;
}

int ECUA_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_Open_out, uint8_t* SDC_HV_ILOCK_out, uint8_t* SDC_IMD_out, uint8_t* SDC_AMS_out, uint8_t* SDC_OUT_out, uint8_t* SDC_END_out, uint8_t* LATCH_SDC_AMS_out, uint8_t* AIRsState_out, uint8_t* ChargingState_out, enum ECUA_StateAMS* AMSState_out, uint8_t* FT_ACP_OT_out, uint8_t* FT_AIRS_out, uint8_t* FT_DCDC_out, uint8_t* FT_FAN1_out, uint8_t* FT_FAN2_out, uint8_t* FT_FAN3_out, uint8_t* FT_HV_OV_out, uint8_t* FT_HV_UV_out, uint8_t* FT_GLV_UV_out, uint8_t* FT_GLV_OV_out, uint8_t* FT_AMS_out, uint8_t* FT_ANY_out, uint8_t* WARN_TEMP_Cell_out, uint8_t* WARN_TEMP_DCDC_out, uint8_t* WARN_TEMP_Bal_out, uint8_t* PWR_ECUB_out, uint8_t* FANS_EN_out, uint8_t* SEQ_out) {
    if (length < 7)
        return 0;

    *SDC_IN_Open_out = (bytes[0] & 0x01);
    *SDC_HV_ILOCK_out = ((bytes[0] >> 1) & 0x01);
    *SDC_IMD_out = ((bytes[0] >> 2) & 0x01);
    *SDC_AMS_out = ((bytes[0] >> 3) & 0x01);
    *SDC_OUT_out = ((bytes[0] >> 4) & 0x01);
    *SDC_END_out = ((bytes[0] >> 5) & 0x01);
    *LATCH_SDC_AMS_out = ((bytes[0] >> 7) & 0x01);
    *AIRsState_out = (bytes[1] & 0x0F);
    *ChargingState_out = ((bytes[1] >> 4) & 0x0F);
    *AMSState_out = (enum ECUA_StateAMS) (bytes[2]);
    *FT_ACP_OT_out = (bytes[3] & 0x01);
    *FT_AIRS_out = ((bytes[3] >> 1) & 0x01);
    *FT_DCDC_out = ((bytes[3] >> 2) & 0x01);
    *FT_FAN1_out = ((bytes[3] >> 3) & 0x01);
    *FT_FAN2_out = ((bytes[3] >> 4) & 0x01);
    *FT_FAN3_out = ((bytes[3] >> 5) & 0x01);
    *FT_HV_OV_out = ((bytes[3] >> 6) & 0x01);
    *FT_HV_UV_out = ((bytes[3] >> 7) & 0x01);
    *FT_GLV_UV_out = (bytes[4] & 0x01);
    *FT_GLV_OV_out = ((bytes[4] >> 1) & 0x01);
    *FT_AMS_out = ((bytes[4] >> 2) & 0x01);
    *FT_ANY_out = ((bytes[4] >> 3) & 0x01);
    *WARN_TEMP_Cell_out = (bytes[5] & 0x01);
    *WARN_TEMP_DCDC_out = ((bytes[5] >> 1) & 0x01);
    *WARN_TEMP_Bal_out = ((bytes[5] >> 2) & 0x01);
    *PWR_ECUB_out = ((bytes[5] >> 6) & 0x01);
    *FANS_EN_out = ((bytes[5] >> 7) & 0x01);
    *SEQ_out = ((bytes[6] >> 4) & 0x0F);
    return 1;
}

int ECUA_get_Status(ECUA_Status_t* data_out) {
    if (!(ECUA_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUA_Status_status.timestamp + ECUA_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUA_Status_data, sizeof(ECUA_Status_t));

    int flags = ECUA_Status_status.flags;
    ECUA_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUA_Status_on_receive(int (*callback)(ECUA_Status_t* data)) {
    ECUA_Status_status.on_receive = (void (*)(void)) callback;
}

int ECUB_send_Status_s(const ECUB_Status_t* data) {
    uint8_t buffer[8];
    buffer[0] = (data->SDC_FRONT ? 1 : 0) | (data->SDC_SDBL ? 2 : 0) | (data->SDC_SDBR ? 4 : 0) | (data->SDC_HVD ? 8 : 0) | (data->SDC_BSPD ? 16 : 0) | (data->SDC_MCUR ? 32 : 0) | (data->SDC_AMS ? 64 : 0) | (data->SDC_TSMS ? 128 : 0);
    buffer[1] = (data->CarState & 0x0F) | ((data->CarState_Notready & 0x0F) << 4);
    buffer[2] = (data->PowerSource & 0x03) | (data->Det_MOD1 ? 16 : 0) | (data->Det_MOD2 ? 32 : 0) | (data->Det_MOD3 ? 64 : 0) | (data->Det_MOD4 ? 128 : 0);
    buffer[3] = (data->FT_PWR1_OT ? 1 : 0) | (data->FT_PWR2_OT ? 2 : 0) | (data->FT_PWR3_OT ? 4 : 0) | (data->FT_PWR4_OT ? 8 : 0) | (data->FT_PWR5_OT ? 16 : 0) | (data->FT_L2_OT ? 32 : 0) | (data->FT_ANY ? 64 : 0) | (data->FT_L1_OT ? 128 : 0);
    buffer[4] = (data->FT_PWR_ECUF_OC ? 1 : 0) | (data->FT_PWR_ECUA_OC ? 2 : 0) | (data->FT_PWR_MCF_OC ? 4 : 0) | (data->FT_PWR_MCR_OC ? 8 : 0) | (data->FT_CAN1 ? 16 : 0) | (data->FT_CAN2 ? 32 : 0);
    buffer[5] = (data->PWR_ECUF_EN ? 1 : 0) | (data->PWR_ECUA_EN ? 2 : 0) | (data->PWR_MCUF_EN ? 4 : 0) | (data->PWR_MCUR_EN ? 8 : 0) | (data->PWR_EM_EN ? 16 : 0) | (data->PWR_WP1_EN ? 32 : 0) | (data->PWR_WP2_EN ? 64 : 0) | (data->PWR_FAN1_EN ? 128 : 0);
    buffer[6] = (data->PWR_FAN2_EN ? 1 : 0) | (data->PWR_FAN3_EN ? 2 : 0) | (data->PWR_WS_EN ? 4 : 0) | (data->PWR_AUX1_EN ? 8 : 0) | (data->PWR_AUX2_EN ? 16 : 0) | (data->PWR_AUX3_EN ? 32 : 0) | (data->RTDS_EN ? 64 : 0) | (data->SDBR_LED_EN ? 128 : 0);
    buffer[7] = (data->SDBL_LED_EN ? 1 : 0) | (data->BrakeLight_EN ? 2 : 0) | (data->TSAL_Override ? 4 : 0) | ((data->SEQ & 0x0F) << 4);
    ECUB_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_Status_id, buffer, sizeof(buffer));
}

int ECUB_send_Status(uint8_t SDC_FRONT, uint8_t SDC_SDBL, uint8_t SDC_SDBR, uint8_t SDC_HVD, uint8_t SDC_BSPD, uint8_t SDC_MCUR, uint8_t SDC_AMS, uint8_t SDC_TSMS, enum ECUB_CarState CarState, enum ECUB_Notready_reason CarState_Notready, enum ECUB_GLV_PowerSource PowerSource, uint8_t Det_MOD1, uint8_t Det_MOD2, uint8_t Det_MOD3, uint8_t Det_MOD4, uint8_t FT_PWR1_OT, uint8_t FT_PWR2_OT, uint8_t FT_PWR3_OT, uint8_t FT_PWR4_OT, uint8_t FT_PWR5_OT, uint8_t FT_L2_OT, uint8_t FT_ANY, uint8_t FT_L1_OT, uint8_t FT_PWR_ECUF_OC, uint8_t FT_PWR_ECUA_OC, uint8_t FT_PWR_MCF_OC, uint8_t FT_PWR_MCR_OC, uint8_t FT_CAN1, uint8_t FT_CAN2, uint8_t PWR_ECUF_EN, uint8_t PWR_ECUA_EN, uint8_t PWR_MCUF_EN, uint8_t PWR_MCUR_EN, uint8_t PWR_EM_EN, uint8_t PWR_WP1_EN, uint8_t PWR_WP2_EN, uint8_t PWR_FAN1_EN, uint8_t PWR_FAN2_EN, uint8_t PWR_FAN3_EN, uint8_t PWR_WS_EN, uint8_t PWR_AUX1_EN, uint8_t PWR_AUX2_EN, uint8_t PWR_AUX3_EN, uint8_t RTDS_EN, uint8_t SDBR_LED_EN, uint8_t SDBL_LED_EN, uint8_t BrakeLight_EN, uint8_t TSAL_Override, uint8_t SEQ) {
    uint8_t buffer[8];
    buffer[0] = (SDC_FRONT ? 1 : 0) | (SDC_SDBL ? 2 : 0) | (SDC_SDBR ? 4 : 0) | (SDC_HVD ? 8 : 0) | (SDC_BSPD ? 16 : 0) | (SDC_MCUR ? 32 : 0) | (SDC_AMS ? 64 : 0) | (SDC_TSMS ? 128 : 0);
    buffer[1] = (CarState & 0x0F) | ((CarState_Notready & 0x0F) << 4);
    buffer[2] = (PowerSource & 0x03) | (Det_MOD1 ? 16 : 0) | (Det_MOD2 ? 32 : 0) | (Det_MOD3 ? 64 : 0) | (Det_MOD4 ? 128 : 0);
    buffer[3] = (FT_PWR1_OT ? 1 : 0) | (FT_PWR2_OT ? 2 : 0) | (FT_PWR3_OT ? 4 : 0) | (FT_PWR4_OT ? 8 : 0) | (FT_PWR5_OT ? 16 : 0) | (FT_L2_OT ? 32 : 0) | (FT_ANY ? 64 : 0) | (FT_L1_OT ? 128 : 0);
    buffer[4] = (FT_PWR_ECUF_OC ? 1 : 0) | (FT_PWR_ECUA_OC ? 2 : 0) | (FT_PWR_MCF_OC ? 4 : 0) | (FT_PWR_MCR_OC ? 8 : 0) | (FT_CAN1 ? 16 : 0) | (FT_CAN2 ? 32 : 0);
    buffer[5] = (PWR_ECUF_EN ? 1 : 0) | (PWR_ECUA_EN ? 2 : 0) | (PWR_MCUF_EN ? 4 : 0) | (PWR_MCUR_EN ? 8 : 0) | (PWR_EM_EN ? 16 : 0) | (PWR_WP1_EN ? 32 : 0) | (PWR_WP2_EN ? 64 : 0) | (PWR_FAN1_EN ? 128 : 0);
    buffer[6] = (PWR_FAN2_EN ? 1 : 0) | (PWR_FAN3_EN ? 2 : 0) | (PWR_WS_EN ? 4 : 0) | (PWR_AUX1_EN ? 8 : 0) | (PWR_AUX2_EN ? 16 : 0) | (PWR_AUX3_EN ? 32 : 0) | (RTDS_EN ? 64 : 0) | (SDBR_LED_EN ? 128 : 0);
    buffer[7] = (SDBL_LED_EN ? 1 : 0) | (BrakeLight_EN ? 2 : 0) | (TSAL_Override ? 4 : 0) | ((SEQ & 0x0F) << 4);
    ECUB_Status_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_Status_id, buffer, sizeof(buffer));
}

int ECUB_Status_need_to_send(void) {
    return (ECUB_Status_last_sent == -1) || (txGetTimeMillis() >= ECUB_Status_last_sent + 200);
}

int ECUB_send_Wheelspeed_s(const ECUB_Wheelspeed_t* data) {
    uint8_t buffer[7];
    buffer[0] = data->WhR;
    buffer[1] = (data->WhR >> 8);
    buffer[2] = data->WhL;
    buffer[3] = (data->WhL >> 8);
    buffer[4] = data->Timestamp;
    buffer[5] = (data->Timestamp >> 8);
    buffer[6] = (data->FT_WhR ? 1 : 0) | (data->FT_WhL ? 2 : 0) | ((data->SEQ & 0x0F) << 4);
    ECUB_Wheelspeed_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_Wheelspeed_id, buffer, sizeof(buffer));
}

int ECUB_send_Wheelspeed(int16_t WhR, int16_t WhL, uint16_t Timestamp, uint8_t FT_WhR, uint8_t FT_WhL, uint8_t SEQ) {
    uint8_t buffer[7];
    buffer[0] = WhR;
    buffer[1] = (WhR >> 8);
    buffer[2] = WhL;
    buffer[3] = (WhL >> 8);
    buffer[4] = Timestamp;
    buffer[5] = (Timestamp >> 8);
    buffer[6] = (FT_WhR ? 1 : 0) | (FT_WhL ? 2 : 0) | ((SEQ & 0x0F) << 4);
    ECUB_Wheelspeed_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_Wheelspeed_id, buffer, sizeof(buffer));
}

int ECUB_Wheelspeed_need_to_send(void) {
    return (ECUB_Wheelspeed_last_sent == -1) || (txGetTimeMillis() >= ECUB_Wheelspeed_last_sent + 10);
}

int ECUB_send_GLV_AMS_s(const ECUB_GLV_AMS_t* data) {
    uint8_t buffer[8];
    buffer[0] = (data->BattState & 0x0F) | (data->FT_Batt ? 16 : 0) | (data->FT_AMS ? 32 : 0) | (data->FT_Charger ? 64 : 0);
    buffer[1] = data->Volt;
    buffer[2] = (data->Volt >> 8);
    buffer[3] = data->Curr;
    buffer[4] = (data->CellID & 0x0F) | ((data->TempID & 0x0F) << 4);
    buffer[5] = data->Volt_cell;
    buffer[6] = (data->Volt_cell >> 8);
    buffer[7] = data->Temp_cell;
    ECUB_GLV_AMS_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_GLV_AMS_id, buffer, sizeof(buffer));
}

int ECUB_send_GLV_AMS(enum ECUB_Batt_code BattState, uint8_t FT_Batt, uint8_t FT_AMS, uint8_t FT_Charger, uint16_t Volt, uint8_t Curr, uint8_t CellID, uint8_t TempID, uint16_t Volt_cell, uint8_t Temp_cell) {
    uint8_t buffer[8];
    buffer[0] = (BattState & 0x0F) | (FT_Batt ? 16 : 0) | (FT_AMS ? 32 : 0) | (FT_Charger ? 64 : 0);
    buffer[1] = Volt;
    buffer[2] = (Volt >> 8);
    buffer[3] = Curr;
    buffer[4] = (CellID & 0x0F) | ((TempID & 0x0F) << 4);
    buffer[5] = Volt_cell;
    buffer[6] = (Volt_cell >> 8);
    buffer[7] = Temp_cell;
    ECUB_GLV_AMS_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN1_powertrain, ECUB_GLV_AMS_id, buffer, sizeof(buffer));
}

int ECUB_GLV_AMS_need_to_send(void) {
    return (ECUB_GLV_AMS_last_sent == -1) || (txGetTimeMillis() >= ECUB_GLV_AMS_last_sent + 100);
}

int ECUB_send_Cooling_s(const ECUB_Cooling_t* data) {
    uint8_t buffer[5];
    buffer[0] = (data->WP1 & 0x0F) | ((data->WP2 & 0x0F) << 4);
    buffer[1] = (data->FAN1 & 0x0F) | ((data->FAN2 & 0x0F) << 4);
    buffer[2] = (data->FAN3 & 0x0F);
    buffer[3] = (data->WARN_MOT_FR_TEMP ? 1 : 0) | (data->WARN_MOT_FL_TEMP ? 2 : 0) | (data->WARN_MOT_RR_TEMP ? 4 : 0) | (data->WARN_MOT_RL_TEMP ? 8 : 0) | (data->WARN_MCU_FR_TEMP ? 16 : 0) | (data->WARN_MCU_FL_TEMP ? 32 : 0) | (data->WARN_MCU_RR_TEMP ? 64 : 0) | (data->WARN_MCU_RL_TEMP ? 128 : 0);
    buffer[4] = (data->FT_MOT_FR_OT ? 1 : 0) | (data->FT_MOT_FL_OT ? 2 : 0) | (data->FT_MOT_RR_OT ? 4 : 0) | (data->FT_MOT_RL_OT ? 8 : 0) | (data->FT_MCU_FR_OT ? 16 : 0) | (data->FT_MCU_FL_OT ? 32 : 0) | (data->FT_MCU_RR_OT ? 64 : 0) | (data->FT_MCU_RL_OT ? 128 : 0);
    ECUB_Cooling_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_Cooling_id, buffer, sizeof(buffer));
}

int ECUB_send_Cooling(uint8_t WP1, uint8_t WP2, uint8_t FAN1, uint8_t FAN2, uint8_t FAN3, uint8_t WARN_MOT_FR_TEMP, uint8_t WARN_MOT_FL_TEMP, uint8_t WARN_MOT_RR_TEMP, uint8_t WARN_MOT_RL_TEMP, uint8_t WARN_MCU_FR_TEMP, uint8_t WARN_MCU_FL_TEMP, uint8_t WARN_MCU_RR_TEMP, uint8_t WARN_MCU_RL_TEMP, uint8_t FT_MOT_FR_OT, uint8_t FT_MOT_FL_OT, uint8_t FT_MOT_RR_OT, uint8_t FT_MOT_RL_OT, uint8_t FT_MCU_FR_OT, uint8_t FT_MCU_FL_OT, uint8_t FT_MCU_RR_OT, uint8_t FT_MCU_RL_OT) {
    uint8_t buffer[5];
    buffer[0] = (WP1 & 0x0F) | ((WP2 & 0x0F) << 4);
    buffer[1] = (FAN1 & 0x0F) | ((FAN2 & 0x0F) << 4);
    buffer[2] = (FAN3 & 0x0F);
    buffer[3] = (WARN_MOT_FR_TEMP ? 1 : 0) | (WARN_MOT_FL_TEMP ? 2 : 0) | (WARN_MOT_RR_TEMP ? 4 : 0) | (WARN_MOT_RL_TEMP ? 8 : 0) | (WARN_MCU_FR_TEMP ? 16 : 0) | (WARN_MCU_FL_TEMP ? 32 : 0) | (WARN_MCU_RR_TEMP ? 64 : 0) | (WARN_MCU_RL_TEMP ? 128 : 0);
    buffer[4] = (FT_MOT_FR_OT ? 1 : 0) | (FT_MOT_FL_OT ? 2 : 0) | (FT_MOT_RR_OT ? 4 : 0) | (FT_MOT_RL_OT ? 8 : 0) | (FT_MCU_FR_OT ? 16 : 0) | (FT_MCU_FL_OT ? 32 : 0) | (FT_MCU_RR_OT ? 64 : 0) | (FT_MCU_RL_OT ? 128 : 0);
    ECUB_Cooling_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_Cooling_id, buffer, sizeof(buffer));
}

int ECUB_Cooling_need_to_send(void) {
    return (ECUB_Cooling_last_sent == -1) || (txGetTimeMillis() >= ECUB_Cooling_last_sent + 100);
}

int ECUB_send_Power_dist_s(const ECUB_Power_dist_t* data) {
    uint8_t buffer[6];
    buffer[0] = data->Volt_DCDC;
    buffer[1] = data->Volt_SBox;
    buffer[2] = data->Curr_ECUF;
    buffer[3] = data->Curr_ECUA;
    buffer[4] = data->Curr_ECUMF;
    buffer[5] = data->Curr_ECUMR;
    ECUB_Power_dist_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_Power_dist_id, buffer, sizeof(buffer));
}

int ECUB_send_Power_dist(uint8_t Volt_DCDC, uint8_t Volt_SBox, uint8_t Curr_ECUF, uint8_t Curr_ECUA, uint8_t Curr_ECUMF, uint8_t Curr_ECUMR) {
    uint8_t buffer[6];
    buffer[0] = Volt_DCDC;
    buffer[1] = Volt_SBox;
    buffer[2] = Curr_ECUF;
    buffer[3] = Curr_ECUA;
    buffer[4] = Curr_ECUMF;
    buffer[5] = Curr_ECUMR;
    ECUB_Power_dist_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_Power_dist_id, buffer, sizeof(buffer));
}

int ECUB_Power_dist_need_to_send(void) {
    return (ECUB_Power_dist_last_sent == -1) || (txGetTimeMillis() >= ECUB_Power_dist_last_sent + 100);
}

int ECUB_send_TEMPSuspR_s(const ECUB_TEMPSuspR_t* data) {
    uint8_t buffer[8];
    buffer[0] = data->BrakeCal_RR;
    buffer[1] = data->BrakeCal_RL;
    buffer[2] = data->TireI_RR;
    buffer[3] = data->TireC_RR;
    buffer[4] = data->TireO_RR;
    buffer[5] = data->TireI_RL;
    buffer[6] = data->TireC_RL;
    buffer[7] = data->TireO_RL;
    ECUB_TEMPSuspR_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_TEMPSuspR_id, buffer, sizeof(buffer));
}

int ECUB_send_TEMPSuspR(uint8_t BrakeCal_RR, uint8_t BrakeCal_RL, uint8_t TireI_RR, uint8_t TireC_RR, uint8_t TireO_RR, uint8_t TireI_RL, uint8_t TireC_RL, uint8_t TireO_RL) {
    uint8_t buffer[8];
    buffer[0] = BrakeCal_RR;
    buffer[1] = BrakeCal_RL;
    buffer[2] = TireI_RR;
    buffer[3] = TireC_RR;
    buffer[4] = TireO_RR;
    buffer[5] = TireI_RL;
    buffer[6] = TireC_RL;
    buffer[7] = TireO_RL;
    ECUB_TEMPSuspR_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_TEMPSuspR_id, buffer, sizeof(buffer));
}

int ECUB_TEMPSuspR_need_to_send(void) {
    return (ECUB_TEMPSuspR_last_sent == -1) || (txGetTimeMillis() >= ECUB_TEMPSuspR_last_sent + 50);
}

int ECUB_send_TEMPAux_s(const ECUB_TEMPAux_t* data) {
    uint8_t buffer[4];
    buffer[0] = data->Cooling1_NTC;
    buffer[1] = data->Cooling2_NTC;
    buffer[2] = data->Cooling3_NTC;
    buffer[3] = data->Cooling4_NTC;
    ECUB_TEMPAux_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_TEMPAux_id, buffer, sizeof(buffer));
}

int ECUB_send_TEMPAux(uint8_t Cooling1_NTC, uint8_t Cooling2_NTC, uint8_t Cooling3_NTC, uint8_t Cooling4_NTC) {
    uint8_t buffer[4];
    buffer[0] = Cooling1_NTC;
    buffer[1] = Cooling2_NTC;
    buffer[2] = Cooling3_NTC;
    buffer[3] = Cooling4_NTC;
    ECUB_TEMPAux_last_sent = txGetTimeMillis();
    return txSendCANMessage(bus_CAN2_aux, ECUB_TEMPAux_id, buffer, sizeof(buffer));
}

int ECUB_TEMPAux_need_to_send(void) {
    return (ECUB_TEMPAux_last_sent == -1) || (txGetTimeMillis() >= ECUB_TEMPAux_last_sent + 200);
}

int ECUF_decode_Status_s(const uint8_t* bytes, size_t length, ECUF_Status_t* data_out) {
    if (length < 5)
        return 0;

    data_out->SDC_SDBC = (bytes[0] & 0x01);
    data_out->SDC_Inertia = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_FWIL = ((bytes[0] >> 2) & 0x01);
    data_out->PWR_ECUP_EN = (bytes[1] & 0x01);
    data_out->PWR_ECUG_EN = ((bytes[1] >> 1) & 0x01);
    data_out->PWR_DTLG_EN = ((bytes[1] >> 2) & 0x01);
    data_out->PWR_ECUS_EN = ((bytes[1] >> 3) & 0x01);
    data_out->PWR_DASH_EN = ((bytes[1] >> 4) & 0x01);
    data_out->PWR_FAN_BrakeF_EN = ((bytes[1] >> 5) & 0x01);
    data_out->FT_PWR_ECUP = (bytes[2] & 0x01);
    data_out->FT_PWR_ECUG = ((bytes[2] >> 1) & 0x01);
    data_out->FT_PWR_ECUS = ((bytes[2] >> 2) & 0x01);
    data_out->FT_PWR_DTLG = ((bytes[2] >> 3) & 0x01);
    data_out->FT_PWR_DASH = ((bytes[2] >> 4) & 0x01);
    data_out->FT_PWR_FAN_BrakeF = ((bytes[2] >> 5) & 0x01);
    data_out->FT_STW_Sensor = ((bytes[2] >> 6) & 0x01);
    data_out->FT_STW_Cal = ((bytes[2] >> 7) & 0x01);
    data_out->FT_DisFR = (bytes[3] & 0x01);
    data_out->FT_DisFL = ((bytes[3] >> 1) & 0x01);
    data_out->FT_DisRR = ((bytes[3] >> 2) & 0x01);
    data_out->FT_DisRL = ((bytes[3] >> 3) & 0x01);
    data_out->FT_DisFR_Cal = ((bytes[3] >> 4) & 0x01);
    data_out->FT_DisFL_Cal = ((bytes[3] >> 5) & 0x01);
    data_out->FT_DisRR_Cal = ((bytes[3] >> 6) & 0x01);
    data_out->FT_DisRL_Cal = ((bytes[3] >> 7) & 0x01);
    data_out->Volt_GLV_In = bytes[4];
    return 1;
}

int ECUF_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_SDBC_out, uint8_t* SDC_Inertia_out, uint8_t* SDC_FWIL_out, uint8_t* PWR_ECUP_EN_out, uint8_t* PWR_ECUG_EN_out, uint8_t* PWR_DTLG_EN_out, uint8_t* PWR_ECUS_EN_out, uint8_t* PWR_DASH_EN_out, uint8_t* PWR_FAN_BrakeF_EN_out, uint8_t* FT_PWR_ECUP_out, uint8_t* FT_PWR_ECUG_out, uint8_t* FT_PWR_ECUS_out, uint8_t* FT_PWR_DTLG_out, uint8_t* FT_PWR_DASH_out, uint8_t* FT_PWR_FAN_BrakeF_out, uint8_t* FT_STW_Sensor_out, uint8_t* FT_STW_Cal_out, uint8_t* FT_DisFR_out, uint8_t* FT_DisFL_out, uint8_t* FT_DisRR_out, uint8_t* FT_DisRL_out, uint8_t* FT_DisFR_Cal_out, uint8_t* FT_DisFL_Cal_out, uint8_t* FT_DisRR_Cal_out, uint8_t* FT_DisRL_Cal_out, uint8_t* Volt_GLV_In_out) {
    if (length < 5)
        return 0;

    *SDC_SDBC_out = (bytes[0] & 0x01);
    *SDC_Inertia_out = ((bytes[0] >> 1) & 0x01);
    *SDC_FWIL_out = ((bytes[0] >> 2) & 0x01);
    *PWR_ECUP_EN_out = (bytes[1] & 0x01);
    *PWR_ECUG_EN_out = ((bytes[1] >> 1) & 0x01);
    *PWR_DTLG_EN_out = ((bytes[1] >> 2) & 0x01);
    *PWR_ECUS_EN_out = ((bytes[1] >> 3) & 0x01);
    *PWR_DASH_EN_out = ((bytes[1] >> 4) & 0x01);
    *PWR_FAN_BrakeF_EN_out = ((bytes[1] >> 5) & 0x01);
    *FT_PWR_ECUP_out = (bytes[2] & 0x01);
    *FT_PWR_ECUG_out = ((bytes[2] >> 1) & 0x01);
    *FT_PWR_ECUS_out = ((bytes[2] >> 2) & 0x01);
    *FT_PWR_DTLG_out = ((bytes[2] >> 3) & 0x01);
    *FT_PWR_DASH_out = ((bytes[2] >> 4) & 0x01);
    *FT_PWR_FAN_BrakeF_out = ((bytes[2] >> 5) & 0x01);
    *FT_STW_Sensor_out = ((bytes[2] >> 6) & 0x01);
    *FT_STW_Cal_out = ((bytes[2] >> 7) & 0x01);
    *FT_DisFR_out = (bytes[3] & 0x01);
    *FT_DisFL_out = ((bytes[3] >> 1) & 0x01);
    *FT_DisRR_out = ((bytes[3] >> 2) & 0x01);
    *FT_DisRL_out = ((bytes[3] >> 3) & 0x01);
    *FT_DisFR_Cal_out = ((bytes[3] >> 4) & 0x01);
    *FT_DisFL_Cal_out = ((bytes[3] >> 5) & 0x01);
    *FT_DisRR_Cal_out = ((bytes[3] >> 6) & 0x01);
    *FT_DisRL_Cal_out = ((bytes[3] >> 7) & 0x01);
    *Volt_GLV_In_out = bytes[4];
    return 1;
}

int ECUF_get_Status(ECUF_Status_t* data_out) {
    if (!(ECUF_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUF_Status_status.timestamp + ECUF_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUF_Status_data, sizeof(ECUF_Status_t));

    int flags = ECUF_Status_status.flags;
    ECUF_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUF_Status_on_receive(int (*callback)(ECUF_Status_t* data)) {
    ECUF_Status_status.on_receive = (void (*)(void)) callback;
}

int ECUF_decode_Dashboard_s(const uint8_t* bytes, size_t length, ECUF_Dashboard_t* data_out) {
    if (length < 3)
        return 0;

    data_out->TSON = (bytes[0] & 0x01);
    data_out->START = ((bytes[0] >> 1) & 0x01);
    data_out->SW1 = ((bytes[0] >> 2) & 0x01);
    data_out->SW2 = ((bytes[0] >> 3) & 0x01);
    data_out->SW3 = ((bytes[0] >> 4) & 0x01);
    data_out->AmbientLight = bytes[1];
    data_out->AmbientTemp = bytes[2];
    return 1;
}

int ECUF_decode_Dashboard(const uint8_t* bytes, size_t length, uint8_t* TSON_out, uint8_t* START_out, uint8_t* SW1_out, uint8_t* SW2_out, uint8_t* SW3_out, uint8_t* AmbientLight_out, uint8_t* AmbientTemp_out) {
    if (length < 3)
        return 0;

    *TSON_out = (bytes[0] & 0x01);
    *START_out = ((bytes[0] >> 1) & 0x01);
    *SW1_out = ((bytes[0] >> 2) & 0x01);
    *SW2_out = ((bytes[0] >> 3) & 0x01);
    *SW3_out = ((bytes[0] >> 4) & 0x01);
    *AmbientLight_out = bytes[1];
    *AmbientTemp_out = bytes[2];
    return 1;
}

int ECUF_get_Dashboard(ECUF_Dashboard_t* data_out) {
    if (!(ECUF_Dashboard_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &ECUF_Dashboard_data, sizeof(ECUF_Dashboard_t));

    int flags = ECUF_Dashboard_status.flags;
    ECUF_Dashboard_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUF_Dashboard_on_receive(int (*callback)(ECUF_Dashboard_t* data)) {
    ECUF_Dashboard_status.on_receive = (void (*)(void)) callback;
}

int ECUP_decode_Status_s(const uint8_t* bytes, size_t length, ECUP_Status_t* data_out) {
    if (length < 2)
        return 0;

    data_out->SDC_BOTS = (bytes[0] & 0x01);
    data_out->FT_ANY = (bytes[1] & 0x01);
    data_out->APPS_Plausible = ((bytes[1] >> 1) & 0x01);
    data_out->BPPC_Latch = ((bytes[1] >> 2) & 0x01);
    data_out->BrakeActive = ((bytes[1] >> 3) & 0x01);
    data_out->BrakeActive_BSPD = ((bytes[1] >> 4) & 0x01);
    return 1;
}

int ECUP_decode_Status(const uint8_t* bytes, size_t length, uint8_t* SDC_BOTS_out, uint8_t* FT_ANY_out, uint8_t* APPS_Plausible_out, uint8_t* BPPC_Latch_out, uint8_t* BrakeActive_out, uint8_t* BrakeActive_BSPD_out) {
    if (length < 2)
        return 0;

    *SDC_BOTS_out = (bytes[0] & 0x01);
    *FT_ANY_out = (bytes[1] & 0x01);
    *APPS_Plausible_out = ((bytes[1] >> 1) & 0x01);
    *BPPC_Latch_out = ((bytes[1] >> 2) & 0x01);
    *BrakeActive_out = ((bytes[1] >> 3) & 0x01);
    *BrakeActive_BSPD_out = ((bytes[1] >> 4) & 0x01);
    return 1;
}

int ECUP_get_Status(ECUP_Status_t* data_out) {
    if (!(ECUP_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > ECUP_Status_status.timestamp + ECUP_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &ECUP_Status_data, sizeof(ECUP_Status_t));

    int flags = ECUP_Status_status.flags;
    ECUP_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void ECUP_Status_on_receive(int (*callback)(ECUP_Status_t* data)) {
    ECUP_Status_status.on_receive = (void (*)(void)) callback;
}

int MCF_decode_GeneralReport_s(const uint8_t* bytes, size_t length, MCF_GeneralReport_t* data_out) {
    if (length < 8)
        return 0;

    data_out->SDC_IN = (bytes[0] & 0x01);
    data_out->SDC_MSCB = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_MPCB = ((bytes[0] >> 2) & 0x01);
    data_out->SDC_HVC = ((bytes[0] >> 3) & 0x01);
    data_out->SDC_MPCA = ((bytes[0] >> 4) & 0x01);
    data_out->SDC_MSCA = ((bytes[0] >> 5) & 0x01);
    data_out->DISCH = (bytes[1] & 0x01);
    data_out->POA = ((bytes[1] >> 1) & 0x01);
    data_out->POA_PS = ((bytes[1] >> 2) & 0x01);
    data_out->POB = ((bytes[1] >> 3) & 0x01);
    data_out->POB_PS = ((bytes[1] >> 4) & 0x01);
    data_out->PWMA = (bytes[2] & 0x01);
    data_out->FWA = ((bytes[2] >> 1) & 0x01);
    data_out->GENA = ((bytes[2] >> 2) & 0x01);
    data_out->DIRA = ((bytes[2] >> 3) & 0x01);
    data_out->PWMB = ((bytes[2] >> 4) & 0x01);
    data_out->FWB = ((bytes[2] >> 5) & 0x01);
    data_out->GENB = ((bytes[2] >> 6) & 0x01);
    data_out->DIRB = ((bytes[2] >> 7) & 0x01);
    data_out->HB = bytes[7];
    return 1;
}

int MCF_decode_GeneralReport(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_out, uint8_t* SDC_MSCB_out, uint8_t* SDC_MPCB_out, uint8_t* SDC_HVC_out, uint8_t* SDC_MPCA_out, uint8_t* SDC_MSCA_out, uint8_t* DISCH_out, uint8_t* POA_out, uint8_t* POA_PS_out, uint8_t* POB_out, uint8_t* POB_PS_out, uint8_t* PWMA_out, uint8_t* FWA_out, uint8_t* GENA_out, uint8_t* DIRA_out, uint8_t* PWMB_out, uint8_t* FWB_out, uint8_t* GENB_out, uint8_t* DIRB_out, uint8_t* HB_out) {
    if (length < 8)
        return 0;

    *SDC_IN_out = (bytes[0] & 0x01);
    *SDC_MSCB_out = ((bytes[0] >> 1) & 0x01);
    *SDC_MPCB_out = ((bytes[0] >> 2) & 0x01);
    *SDC_HVC_out = ((bytes[0] >> 3) & 0x01);
    *SDC_MPCA_out = ((bytes[0] >> 4) & 0x01);
    *SDC_MSCA_out = ((bytes[0] >> 5) & 0x01);
    *DISCH_out = (bytes[1] & 0x01);
    *POA_out = ((bytes[1] >> 1) & 0x01);
    *POA_PS_out = ((bytes[1] >> 2) & 0x01);
    *POB_out = ((bytes[1] >> 3) & 0x01);
    *POB_PS_out = ((bytes[1] >> 4) & 0x01);
    *PWMA_out = (bytes[2] & 0x01);
    *FWA_out = ((bytes[2] >> 1) & 0x01);
    *GENA_out = ((bytes[2] >> 2) & 0x01);
    *DIRA_out = ((bytes[2] >> 3) & 0x01);
    *PWMB_out = ((bytes[2] >> 4) & 0x01);
    *FWB_out = ((bytes[2] >> 5) & 0x01);
    *GENB_out = ((bytes[2] >> 6) & 0x01);
    *DIRB_out = ((bytes[2] >> 7) & 0x01);
    *HB_out = bytes[7];
    return 1;
}

int MCF_get_GeneralReport(MCF_GeneralReport_t* data_out) {
    if (!(MCF_GeneralReport_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &MCF_GeneralReport_data, sizeof(MCF_GeneralReport_t));

    int flags = MCF_GeneralReport_status.flags;
    MCF_GeneralReport_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void MCF_GeneralReport_on_receive(int (*callback)(MCF_GeneralReport_t* data)) {
    MCF_GeneralReport_status.on_receive = (void (*)(void)) callback;
}

int MCR_decode_GeneralReport_s(const uint8_t* bytes, size_t length, MCR_GeneralReport_t* data_out) {
    if (length < 8)
        return 0;

    data_out->SDC_IN = (bytes[0] & 0x01);
    data_out->SDC_MSCB = ((bytes[0] >> 1) & 0x01);
    data_out->SDC_MPCB = ((bytes[0] >> 2) & 0x01);
    data_out->SDC_HVC = ((bytes[0] >> 3) & 0x01);
    data_out->SDC_MPCA = ((bytes[0] >> 4) & 0x01);
    data_out->SDC_MSCA = ((bytes[0] >> 5) & 0x01);
    data_out->DISCH = (bytes[1] & 0x01);
    data_out->POA = ((bytes[1] >> 1) & 0x01);
    data_out->POA_PS = ((bytes[1] >> 2) & 0x01);
    data_out->POB = ((bytes[1] >> 3) & 0x01);
    data_out->POB_PS = ((bytes[1] >> 4) & 0x01);
    data_out->PWMA = (bytes[2] & 0x01);
    data_out->FWA = ((bytes[2] >> 1) & 0x01);
    data_out->GENA = ((bytes[2] >> 2) & 0x01);
    data_out->DIRA = ((bytes[2] >> 3) & 0x01);
    data_out->PWMB = ((bytes[2] >> 4) & 0x01);
    data_out->FWB = ((bytes[2] >> 5) & 0x01);
    data_out->GENB = ((bytes[2] >> 6) & 0x01);
    data_out->DIRB = ((bytes[2] >> 7) & 0x01);
    data_out->HB = bytes[7];
    return 1;
}

int MCR_decode_GeneralReport(const uint8_t* bytes, size_t length, uint8_t* SDC_IN_out, uint8_t* SDC_MSCB_out, uint8_t* SDC_MPCB_out, uint8_t* SDC_HVC_out, uint8_t* SDC_MPCA_out, uint8_t* SDC_MSCA_out, uint8_t* DISCH_out, uint8_t* POA_out, uint8_t* POA_PS_out, uint8_t* POB_out, uint8_t* POB_PS_out, uint8_t* PWMA_out, uint8_t* FWA_out, uint8_t* GENA_out, uint8_t* DIRA_out, uint8_t* PWMB_out, uint8_t* FWB_out, uint8_t* GENB_out, uint8_t* DIRB_out, uint8_t* HB_out) {
    if (length < 8)
        return 0;

    *SDC_IN_out = (bytes[0] & 0x01);
    *SDC_MSCB_out = ((bytes[0] >> 1) & 0x01);
    *SDC_MPCB_out = ((bytes[0] >> 2) & 0x01);
    *SDC_HVC_out = ((bytes[0] >> 3) & 0x01);
    *SDC_MPCA_out = ((bytes[0] >> 4) & 0x01);
    *SDC_MSCA_out = ((bytes[0] >> 5) & 0x01);
    *DISCH_out = (bytes[1] & 0x01);
    *POA_out = ((bytes[1] >> 1) & 0x01);
    *POA_PS_out = ((bytes[1] >> 2) & 0x01);
    *POB_out = ((bytes[1] >> 3) & 0x01);
    *POB_PS_out = ((bytes[1] >> 4) & 0x01);
    *PWMA_out = (bytes[2] & 0x01);
    *FWA_out = ((bytes[2] >> 1) & 0x01);
    *GENA_out = ((bytes[2] >> 2) & 0x01);
    *DIRA_out = ((bytes[2] >> 3) & 0x01);
    *PWMB_out = ((bytes[2] >> 4) & 0x01);
    *FWB_out = ((bytes[2] >> 5) & 0x01);
    *GENB_out = ((bytes[2] >> 6) & 0x01);
    *DIRB_out = ((bytes[2] >> 7) & 0x01);
    *HB_out = bytes[7];
    return 1;
}

int MCR_get_GeneralReport(MCR_GeneralReport_t* data_out) {
    if (!(MCR_GeneralReport_status.flags & CAN_MSG_RECEIVED))
        return 0;

    if (data_out)
        memcpy(data_out, &MCR_GeneralReport_data, sizeof(MCR_GeneralReport_t));

    int flags = MCR_GeneralReport_status.flags;
    MCR_GeneralReport_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void MCR_GeneralReport_on_receive(int (*callback)(MCR_GeneralReport_t* data)) {
    MCR_GeneralReport_status.on_receive = (void (*)(void)) callback;
}

int VDCU_decode_Status_s(const uint8_t* bytes, size_t length, VDCU_Status_t* data_out) {
    if (length < 4)
        return 0;

    data_out->State = (bytes[1] & 0x0F);
    data_out->FT_Dis_Cal = (bytes[2] & 0x01);
    data_out->FT_Sensor = ((bytes[2] >> 1) & 0x01);
    data_out->Temp_derating = ((bytes[2] >> 2) & 0x01);
    data_out->ACP_derate = ((bytes[2] >> 3) & 0x01);
    data_out->Disch_ACT = ((bytes[2] >> 4) & 0x01);
    data_out->Reverse_ACT = ((bytes[2] >> 5) & 0x01);
    data_out->TV_ENABLED = ((bytes[2] >> 6) & 0x01);
    data_out->TC_ENABLED = ((bytes[2] >> 7) & 0x01);
    data_out->YC_ENABLED = (bytes[3] & 0x01);
    data_out->TC_ACT = ((bytes[3] >> 1) & 0x01);
    data_out->YC_ACT = ((bytes[3] >> 2) & 0x01);
    return 1;
}

int VDCU_decode_Status(const uint8_t* bytes, size_t length, uint8_t* State_out, uint8_t* FT_Dis_Cal_out, uint8_t* FT_Sensor_out, uint8_t* Temp_derating_out, uint8_t* ACP_derate_out, uint8_t* Disch_ACT_out, uint8_t* Reverse_ACT_out, uint8_t* TV_ENABLED_out, uint8_t* TC_ENABLED_out, uint8_t* YC_ENABLED_out, uint8_t* TC_ACT_out, uint8_t* YC_ACT_out) {
    if (length < 4)
        return 0;

    *State_out = (bytes[1] & 0x0F);
    *FT_Dis_Cal_out = (bytes[2] & 0x01);
    *FT_Sensor_out = ((bytes[2] >> 1) & 0x01);
    *Temp_derating_out = ((bytes[2] >> 2) & 0x01);
    *ACP_derate_out = ((bytes[2] >> 3) & 0x01);
    *Disch_ACT_out = ((bytes[2] >> 4) & 0x01);
    *Reverse_ACT_out = ((bytes[2] >> 5) & 0x01);
    *TV_ENABLED_out = ((bytes[2] >> 6) & 0x01);
    *TC_ENABLED_out = ((bytes[2] >> 7) & 0x01);
    *YC_ENABLED_out = (bytes[3] & 0x01);
    *TC_ACT_out = ((bytes[3] >> 1) & 0x01);
    *YC_ACT_out = ((bytes[3] >> 2) & 0x01);
    return 1;
}

int VDCU_get_Status(VDCU_Status_t* data_out) {
    if (!(VDCU_Status_status.flags & CAN_MSG_RECEIVED))
        return 0;

#ifndef CANDB_IGNORE_TIMEOUTS
    if (txGetTimeMillis() > VDCU_Status_status.timestamp + VDCU_Status_timeout)
        return 0;
#endif

    if (data_out)
        memcpy(data_out, &VDCU_Status_data, sizeof(VDCU_Status_t));

    int flags = VDCU_Status_status.flags;
    VDCU_Status_status.flags &= ~CAN_MSG_PENDING;
    return flags;
}

void VDCU_Status_on_receive(int (*callback)(VDCU_Status_t* data)) {
    VDCU_Status_status.on_receive = (void (*)(void)) callback;
}

void candbHandleMessage(uint32_t timestamp, int bus, CAN_ID_t id, const uint8_t* payload, size_t payload_length) {
    switch (id) {
    case ECUA_Status_id: {
        if (!ECUA_decode_Status_s(payload, payload_length, &ECUA_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUA_Status_status, timestamp);

        if (ECUA_Status_status.on_receive)
            ((int (*)(ECUA_Status_t*)) ECUA_Status_status.on_receive)(&ECUA_Status_data);

        break;
    }
    case ECUF_Status_id: {
        if (!ECUF_decode_Status_s(payload, payload_length, &ECUF_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUF_Status_status, timestamp);

        if (ECUF_Status_status.on_receive)
            ((int (*)(ECUF_Status_t*)) ECUF_Status_status.on_receive)(&ECUF_Status_data);

        break;
    }
    case ECUF_Dashboard_id: {
        if (!ECUF_decode_Dashboard_s(payload, payload_length, &ECUF_Dashboard_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUF_Dashboard_status, timestamp);

        if (ECUF_Dashboard_status.on_receive)
            ((int (*)(ECUF_Dashboard_t*)) ECUF_Dashboard_status.on_receive)(&ECUF_Dashboard_data);

        break;
    }
    case ECUP_Status_id: {
        if (!ECUP_decode_Status_s(payload, payload_length, &ECUP_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&ECUP_Status_status, timestamp);

        if (ECUP_Status_status.on_receive)
            ((int (*)(ECUP_Status_t*)) ECUP_Status_status.on_receive)(&ECUP_Status_data);

        break;
    }
    case MCF_GeneralReport_id: {
        if (!MCF_decode_GeneralReport_s(payload, payload_length, &MCF_GeneralReport_data))
            break;

        canUpdateMsgStatusOnReceive(&MCF_GeneralReport_status, timestamp);

        if (MCF_GeneralReport_status.on_receive)
            ((int (*)(MCF_GeneralReport_t*)) MCF_GeneralReport_status.on_receive)(&MCF_GeneralReport_data);

        break;
    }
    case MCR_GeneralReport_id: {
        if (!MCR_decode_GeneralReport_s(payload, payload_length, &MCR_GeneralReport_data))
            break;

        canUpdateMsgStatusOnReceive(&MCR_GeneralReport_status, timestamp);

        if (MCR_GeneralReport_status.on_receive)
            ((int (*)(MCR_GeneralReport_t*)) MCR_GeneralReport_status.on_receive)(&MCR_GeneralReport_data);

        break;
    }
    case VDCU_Status_id: {
        if (!VDCU_decode_Status_s(payload, payload_length, &VDCU_Status_data))
            break;

        canUpdateMsgStatusOnReceive(&VDCU_Status_status, timestamp);

        if (VDCU_Status_status.on_receive)
            ((int (*)(VDCU_Status_t*)) VDCU_Status_status.on_receive)(&VDCU_Status_data);

        break;
    }
    }
}
