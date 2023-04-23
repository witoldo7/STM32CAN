// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#include <string.h>
#include "hal.h"
#include "j2534.h"
#include "j2534can.h"
#include "j2534kline.h"

uint32_t handle_connect_default(void* conn) {(void)conn; return ERR_NOT_SUPPORTED;}
uint32_t handle_disconnect_default(void* conn) {(void)conn; return ERR_NOT_SUPPORTED;}
uint32_t start_filter_default(void* conn, uint8_t* data, uint32_t* idx) {(void)conn; (void)data; (void)idx; return ERR_NOT_SUPPORTED;}
uint32_t stop_filter_default(void* conn, uint32_t idx) {(void)conn; (void)idx; return ERR_NOT_SUPPORTED;}
uint32_t write_message_default(void* conn, uint32_t timeout, uint16_t len, uint8_t* data) {
  (void)conn; (void)timeout; (void)len; (void)data; return ERR_NOT_SUPPORTED;}
uint32_t start_periodic_default_msg(void* conn, uint8_t data) {(void)conn; (void)data; return ERR_NOT_SUPPORTED;}
uint32_t stop_periodic_default_msg(void* conn, uint32_t msg) {(void)conn; (void)msg; return ERR_NOT_SUPPORTED;}
uint32_t ioctl_clear_filters_default(void* conn) {(void)conn; return ERR_NOT_SUPPORTED;}
uint32_t ioctl_datarate_default(void* conn) {(void)conn; return ERR_NOT_SUPPORTED;}
uint32_t ioctl_loopback_default(void* conn) {(void)conn; return ERR_NOT_SUPPORTED;}
uint32_t ioctl_fast_init_default(void* conn, uint8_t* in, uint8_t* out) {(void)conn; (void)in; (void)out; return ERR_NOT_SUPPORTED;}
uint32_t ioctl_five_baud_init_default(void* conn, uint8_t* in, uint8_t out) {(void)conn; (void)in; (void)out; return ERR_NOT_SUPPORTED;}

static uint8_t retBuff[64] = { 0 };
const j2534_protocol_cfg default_config = {
  .Loopback = 0,        // 0 - OFF, 1 - ON
  .NodeAddress = 0,     // N/A 0x00-0xFF
  .NetworkLine = 0,     // 0 - BUS_NORMAL, 1 - BUS_PLUS, 2 - BUS_MINUS
  .P1Min = 0,           // N/A
  .P1Max = 40,          // .5ms, 0x1-0xFFFF
  .P2Min = 0,           // N/A
  .P2Max = 0,           // N/A
  .P3Min = 110,         // .5ms per bit 110 -> 55ms 0x1-0xFFFF
  .P3Max = 0,           // N/A
  .P4Min = 10,          // .5ms 0x1-0xFFFF
  .P4Max = 0,           // N/A
  .W0a = 300,           // 1ms 0x1-0xFFFF
  .W1a = 300,           // 1ms 0x1-0xFFFF
  .W2a = 20,            // 1ms 0x1-0xFFFF
  .W3a = 20,            // 1ms 0x1-0xFFFF
  .W4a = 50,            // 1ms 0x1-0xFFFF
  .W5a = 300,           // 1ms 0x1-0xFFFF
  .Tidle = 300,         // 1ms 0x1-0xFFFF
  .Tinil = 25,          // 1ms 0x1-0xFFFF
  .Twup = 50,           // 1ms 0x1-0xFFFF
  .Parity = 0,          // 0 - NO_PARITY, 1 - ODD_PARITY, 2 - EVEN_PARITY
  .BitSamplePoint = 80, // 1% 0-100
  .SyncJumpWidth = 15,  // 1% 0-100
  .T1Max = 20,          // 1ms 0x1-0xFFFF
  .T2Max = 100,         // 1ms 0x1-0xFFFF
  .T3Max = 50,          // 1ms 0x1-0xFFFF
  .T4Max = 20,          // 1ms 0x1-0xFFFF
  .T5Max = 120,         // 1ms 0x1-0xFFFF
  .Iso15765Bs = 0,      //
  .Iso15765Stmin = 0,
  .Iso15765BsTx = 0xFFFF,
  .Iso15765StminTx = 0xFFFF,
  .DataBits = 0,       // 0 - 8 data bits, 1 - 7 data bits
  .FiveBaudMod = 0,    // 0 - Initialization as defined in ISO9141-2 and ISO14230-4
                       // 1 - ISO 9141initialization followed by interface sending inverted Key Byte 2
                       // 2 - ISO 9141 initialization followed by ECU sending inverted address
                       // 3 - Initialization as defined in ISO 9141
  .Iso15765WftMax = 0,
  .CanMixedFormat = 0,
  .J1962Pins = 0,
  .SwCanHsDataRate = 0,
  .SwCanSpeedchangeEnable = 0,
  .SwCanResSwitch = 0,
  .ActiveChannels = 0,
  .SampleRate = 0,
  .SamplesPerReading = 0,
  .ReadingsPerMsg = 0,
  .AveragingMethod = 0,
  .SampleResolution = 0,
  .InputRangeLow = 0,
  .InputRangeHigh = 0
};

j2534_protocol_cfg hscan_pcfg = default_config;
j2534_protocol_cfg swcan_pcfg = default_config;
j2534_protocol_cfg kline_pcfg = default_config;
j2534_protocol_cfg default_pcfg = default_config;

j2534_conn connHs = {
  .connect = (void*)&handle_connect_can,
  .disconnect = (void*)&handle_disconnect_can,
  .write = (void*)&write_message_can,
  .start_filter = (void*)&start_filter_can,
  .stop_filter = (void*)&stop_filter_can,
  .ioctl_lopback = (void*)ioctl_loopback_can,
  .ioctl_datarate = (void*)ioctl_datarate_can,
  .ioctl_clear_filters = (void*)ioctl_clear_filters_can,
  .ioctl_fast_init = (void*)&ioctl_fast_init_default,
  .ioctl_five_baud_init = (void*)&ioctl_five_baud_init_default,
  .pcfg = &hscan_pcfg
};

j2534_conn connSw = {
  .connect = (void*)&handle_connect_can,
  .disconnect = (void*)&handle_disconnect_can,
  .write = (void*)&write_message_can,
  .start_filter = (void*)&start_filter_can,
  .stop_filter = (void*)&stop_filter_can,
  .ioctl_lopback = (void*)ioctl_loopback_can,
  .ioctl_datarate = (void*)ioctl_datarate_can,
  .ioctl_clear_filters = (void*)ioctl_clear_filters_can,
  .ioctl_fast_init = (void*)&ioctl_fast_init_default,
  .ioctl_five_baud_init = (void*)&ioctl_five_baud_init_default,
  .pcfg = &swcan_pcfg
  };

j2534_conn connKL = {
  .connect = (void*)&handle_connect_kline,
  .disconnect = (void*)&handle_disconnect_kline,
  .write = (void*)&write_message_kline,
  .start_filter = (void*)&start_filter_kline,
  .stop_filter = (void*)&stop_filter_kline,
  .ioctl_lopback = (void*)ioctl_loopback_kline,
  .ioctl_datarate = (void*)ioctl_datarate_kline,
  .ioctl_clear_filters = (void*)&ioctl_clear_filters_kline,
  .ioctl_fast_init = (void*)&ioctl_fast_init_kline,
  .ioctl_five_baud_init = (void*)&ioctl_five_baud_init_kline,
  .pcfg = &kline_pcfg
};

j2534_conn conDefault = {
  .connect = (void*)&handle_connect_default,
  .disconnect = (void*)&handle_disconnect_default,
  .write = (void*)&write_message_default,
  .start_filter = (void*)&start_filter_default,
  .stop_filter = (void*)&stop_filter_default,
  .ioctl_lopback = (void*)ioctl_loopback_default,
  .ioctl_datarate = (void*)ioctl_datarate_default,
  .ioctl_clear_filters = (void*)ioctl_clear_filters_default,
  .ioctl_fast_init = (void*)&ioctl_fast_init_default,
  .ioctl_five_baud_init = (void*)&ioctl_five_baud_init_default,
  .pcfg = &default_pcfg
};

j2534_conn* get_connection(uint32_t channel) {
  switch (channel) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    return &connHs;
  case SW_CAN_PS:
    return &connSw;
  case ISO9141:
  case ISO14230:
    return &connKL;
  default:
    return &conDefault;
  }
}

bool j2534_connect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t protocolID, flags, bitrate, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 12) {
    goto error;
  }
  memcpy(&protocolID, rx_packet->data, 4);
  memcpy(&flags, rx_packet->data + 4, 4);
  memcpy(&bitrate, rx_packet->data + 8, 4);

  j2534_conn* conn = get_connection(protocolID);
  conn->protocol = protocolID;
  conn->flags = flags;
  conn->DataRate = bitrate;

  err = conn->connect(conn);
  if (err != STATUS_NOERROR) {
    goto error;
  }

  conn->isConnected = true;
  memcpy(retBuff, &err, 4);
  memcpy(retBuff + 4, &protocolID, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_disconnect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 4) {
    goto error;
  }
  memcpy(&channelID, rx_packet->data, 4);

  j2534_conn* conn = get_connection(channelID);
  err = conn->disconnect(conn);
  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_start_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 4) {
    goto error;
  }
  uint8_t size = 5;
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);
  uint32_t filterID = 0;
  err = conn->start_filter(conn, rx_packet->data, &filterID);
  if (err != STATUS_NOERROR) {
    goto error;
  }

  memcpy(retBuff, &err, 4);
  memcpy(retBuff+4, &filterID, 1);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, size, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_stop_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, filterID, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 8) {
    goto error;
  }
  uint8_t size = 4;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&filterID, rx_packet->data+4, 4);
  j2534_conn* conn = get_connection(channelID);
  err = conn->stop_filter(conn, filterID);
  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, size, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

uint32_t handldle_get_config(j2534_conn* conn, SCONFIG_LIST* cfgList) {
  uint32_t err = ERR_NOT_SUPPORTED;
  j2534_protocol_cfg *pcfg = conn->pcfg;
  for (uint8_t i = 0; i < cfgList->NumOfParams; i++) {
    SCONFIG *cfg = &cfgList->ConfigPtr[i];
    switch (cfg->Parameter) {
    case DATA_RATE:
      cfg->Value = conn->DataRate;
      err = STATUS_NOERROR;
      break;
    case LOOPBACK:
      cfg->Value = pcfg->Loopback;
      err = STATUS_NOERROR;
      break;
    case BIT_SAMPLE_POINT:
      cfg->Value = pcfg->BitSamplePoint;
      err = STATUS_NOERROR;
      break;
    case SYNC_JUMP_WIDTH:
      cfg->Value = pcfg->SyncJumpWidth;
      err = STATUS_NOERROR;
      break;
    case CAN_MIXED_FORMAT:
      cfg->Value = pcfg->CanMixedFormat;
      err = STATUS_NOERROR;
      break;
    case J1962_PINS:
      cfg->Value = pcfg->J1962Pins;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_HS_DATA_RATE:
      cfg->Value = pcfg->SwCanHsDataRate;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_SPEEDCHANGE_ENABLE:
      cfg->Value = pcfg->SwCanSpeedchangeEnable;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_RES_SWITCH:
      cfg->Value = pcfg->SwCanResSwitch;
      err = STATUS_NOERROR;
      break;
    case ISO15765_STMIN:
      cfg->Value = pcfg->Iso15765Stmin;
      err = STATUS_NOERROR;
      break;
    case ISO15765_BS:
      cfg->Value = pcfg->Iso15765Bs;
      err = STATUS_NOERROR;
      break;
    case ISO15765_STMIN_TX:
      cfg->Value = pcfg->Iso15765StminTx;
      err = STATUS_NOERROR;
      break;
    case ISO15765_BS_TX:
      cfg->Value = pcfg->Iso15765BsTx;
      err = STATUS_NOERROR;
      break;
    case ISO15765_WFT_MAX:
      cfg->Value = pcfg->Iso15765WftMax;
      err = STATUS_NOERROR;
      break;
    // ISO9141 or ISO14230
    case P1_MIN:
      cfg->Value = pcfg->P1Min;
      err = STATUS_NOERROR;
      break;
    case P1_MAX:
      cfg->Value = pcfg->P1Max;
      err = STATUS_NOERROR;
      break;
    case P2_MIN:
      cfg->Value = pcfg->P2Min;
      err = STATUS_NOERROR;
      break;
    case P2_MAX:
      cfg->Value = pcfg->P2Max;
      err = STATUS_NOERROR;
      break;
    case P3_MIN:
      cfg->Value = pcfg->P3Min;
      err = STATUS_NOERROR;
      break;
    case P3_MAX:
      cfg->Value = pcfg->P3Max;
      err = STATUS_NOERROR;
      break;
    case P4_MIN:
      cfg->Value = pcfg->P4Min;
      err = STATUS_NOERROR;
      break;
    case P4_MAX:
      cfg->Value = pcfg->P4Max;
      err = STATUS_NOERROR;
      break;
    case W0:
      cfg->Value = pcfg->W0a;
      err = STATUS_NOERROR;
      break;
    case W1:
      cfg->Value = pcfg->W1a;
      err = STATUS_NOERROR;
      break;
    case W2:
      cfg->Value = pcfg->W2a;
      err = STATUS_NOERROR;
      break;
    case W3:
      cfg->Value = pcfg->W3a;
      err = STATUS_NOERROR;
      break;
    case W4:
      cfg->Value = pcfg->W4a;
      err = STATUS_NOERROR;
      break;
    case W5:
      cfg->Value = pcfg->W5a;
      err = STATUS_NOERROR;
      break;
    case TIDLE:
      cfg->Value = pcfg->Tidle;
      err = STATUS_NOERROR;
      break;
    case TINIL:
      cfg->Value = pcfg->Tinil;
      err = STATUS_NOERROR;
      break;
    case TWUP:
      cfg->Value = pcfg->Twup;
      err = STATUS_NOERROR;
      break;
    case PARITY:
      cfg->Value = pcfg->Parity;
      err = STATUS_NOERROR;
      break;
    case DATA_BITS:
      cfg->Value = pcfg->DataBits;
      err = STATUS_NOERROR;
      break;
    case FIVE_BAUD_MOD:
      cfg->Value = pcfg->FiveBaudMod;
      err = STATUS_NOERROR;
    break;
    default:
      break;
    }
  }
  return err;
}

uint32_t handldle_set_config(j2534_conn* conn, SCONFIG_LIST* cfgList) {
  uint32_t err = ERR_NOT_SUPPORTED;
  j2534_protocol_cfg *pcfg = conn->pcfg;
  for (uint8_t i = 0; i < cfgList->NumOfParams; i++) {
    SCONFIG *cfg = &cfgList->ConfigPtr[i];
    switch (cfg->Parameter) {
    case DATA_RATE:
      conn->DataRate = cfg->Value;
      err = conn->ioctl_datarate(conn);
     break;
    case LOOPBACK:
      pcfg->Loopback = cfg->Value;
      err = conn->ioctl_lopback(conn);
      break;
    case CAN_MIXED_FORMAT:
      pcfg->CanMixedFormat = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case J1962_PINS:
      pcfg->J1962Pins = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_HS_DATA_RATE:
      pcfg->SwCanHsDataRate = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_SPEEDCHANGE_ENABLE:
      pcfg->SwCanSpeedchangeEnable = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_RES_SWITCH:
      pcfg->SwCanResSwitch = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case ISO15765_STMIN:
      pcfg->Iso15765Stmin = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case ISO15765_BS:
      pcfg->Iso15765Bs = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case ISO15765_STMIN_TX:
      pcfg->Iso15765StminTx = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case ISO15765_BS_TX:
      pcfg->Iso15765BsTx = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case ISO15765_WFT_MAX:
      pcfg->Iso15765WftMax = cfg->Value;
      err = STATUS_NOERROR;
      break;
      // ISO9141 or ISO14230
    case P1_MIN:
      pcfg->P1Min = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P1_MAX:
      pcfg->P1Max = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P2_MIN:
      pcfg->P2Min = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P2_MAX:
      pcfg->P2Max = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P3_MIN:
      pcfg->P3Min = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P3_MAX:
      pcfg->P3Max = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P4_MIN:
      pcfg->P4Min = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case P4_MAX:
      pcfg->P4Max = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W0:
      pcfg->W0a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W1:
      pcfg->W1a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W2:
      pcfg->W2a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W3:
      pcfg->W3a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W4:
      pcfg->W4a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case W5:
      pcfg->W5a = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case TIDLE:
      pcfg->Tidle = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case TINIL:
      pcfg->Tinil = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case TWUP:
      pcfg->Twup = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case PARITY:
      pcfg->Parity = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case DATA_BITS:
      pcfg->DataBits = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case FIVE_BAUD_MOD:
      pcfg->FiveBaudMod = cfg->Value;
      err = STATUS_NOERROR;
    break;
    default:
      break;
    }
  }
  return err;
}

bool j2534_ioctl(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, ioctl, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 8) {
    goto error;
  }
  uint8_t retSize = 4;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&ioctl, rx_packet->data + 4, 4);

  j2534_conn* conn = get_connection(channelID);

  switch(ioctl) {
  case GET_CONFIG:
  case SET_CONFIG:
    uint32_t pSize = 0;
    memcpy(&pSize, rx_packet->data + 8, 4);
    SCONFIG c[16] = {0}; //Handle better
    memcpy(c, rx_packet->data + 12, rx_packet->data_len-12);
    SCONFIG_LIST cfgList = {.NumOfParams = pSize, .ConfigPtr = c};
    if (ioctl == GET_CONFIG) {
      err = handldle_get_config(conn, &cfgList);
      memcpy(retBuff+4, &cfgList.NumOfParams, 4);
      memcpy(retBuff+8, cfgList.ConfigPtr, pSize*8);
      retSize = 4+pSize*8+4;
    } else {
      err = handldle_set_config(conn, &cfgList);
    }
    break;
  case READ_VBATT:
    uint16_t vBat = getSupplyVoltage();
    memcpy(retBuff + 4, &vBat, 2);
    retSize = 6;
    err = STATUS_NOERROR;
    break;
  case CLEAR_RX_BUFFER:
    err = STATUS_NOERROR;
    break;
  case CLEAR_MSG_FILTERS:
    err = conn->ioctl_clear_filters(conn);
    break;
  case FIVE_BAUD_INIT:
  case FAST_INIT:
    uint8_t f_out[8];
    if ((rx_packet->data_len - 8) < 5) {
      err = ERR_FAILED;
      goto error;
    }
    if (ioctl == FIVE_BAUD_INIT) {
      err = conn->ioctl_five_baud_init(conn, rx_packet->data + 8, f_out);
    } else {
      err = conn->ioctl_fast_init(conn, rx_packet->data + 8, f_out);
    }
    if (err == STATUS_NOERROR) {
      retSize += f_out[0];
      memcpy(retBuff + 4, f_out, f_out[0]);
    }
    break;
  case CLEAR_TX_BUFFER:
  case CLEAR_PERIODIC_MSGS:
  case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
  case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
  case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
  case READ_PROG_VOLTAGE:
  default:
      break;
  }

  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, retSize, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_write_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, timeout, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 4) {
    goto error;
  }
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&timeout, rx_packet->data + 4, 4);
  j2534_conn* conn = get_connection(channelID);
  err = conn->write(conn, timeout, rx_packet->data_len-8, rx_packet->data+8);

  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_start_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 4) {
    goto error;
  }
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);

  err = conn->start_periodic_msg(conn, rx_packet->data);
  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool j2534_stop_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len < 4) {
    goto error;
  }
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);

  err = conn->stop_periodic_msg(conn, 0);
  memcpy(retBuff, &err, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);

  error:
    memcpy(retBuff, &err, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_term_ack);
}

bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_j2534_connect:
    return j2534_connect(rx_packet, tx_packet);
  case cmd_j2534_disconnect:
    return j2534_disconnect(rx_packet, tx_packet);
  case cmd_j2534_ioctl:
    return j2534_ioctl(rx_packet, tx_packet);
  case cmd_j2534_filter:
    return j2534_start_filter(rx_packet, tx_packet);
  case cmd_j2534_stop_filter:
    return j2534_stop_filter(rx_packet, tx_packet);
  case cmd_j2534_write_message:
    return j2534_write_message(rx_packet, tx_packet);
  case cmd_j2534_start_periodic_message:
    return j2534_start_periodic_message(rx_packet, tx_packet);
  case cmd_j2534_stop_periodic_message:
    return j2534_stop_periodic_message(rx_packet, tx_packet);
  default:
    break;
  }
  return false;
}
