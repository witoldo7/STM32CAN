/*
 * j2534.c
 *
 *  Created on: 20 lut 2023
 *      Author: witold
 */
#include <string.h>
#include "hal.h"
#include "j2534.h"

CAN_RamAddress hscan_ram, swcan_ram;

CANRamConfig hscan_ram_cfg = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = 4,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 4,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_64,
  .RxFifo1ElmtsNbr = 4,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_64,
  .RxBuffersNbr = 4,
  .RxBufferSize = FDCAN_DATA_BYTES_64,
  .TxEventsNbr = 1,
  .TxBuffersNbr = 4,
  .TxFifoQueueElmtsNbr = 4,
  .TxElmtSize = FDCAN_DATA_BYTES_64
};

CANRamConfig swcan_ram_cfg = {
  .MessageRAMOffset = 384,
  .StdFiltersNbr = 4,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 4,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 4,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 4,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 1,
  .TxBuffersNbr = 4,
  .TxFifoQueueElmtsNbr = 4,
  .TxElmtSize = FDCAN_DATA_BYTES_8
};
static CANConfig hsCanConfig = {
  .DBTP =  0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

static CANConfig swCanConfig = {
  .DBTP = 0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

static uint8_t retBuff[32] = {0};
int MAX_SIZE = 72;
f_packet_t queue[72] = {0};
int front = 0;
int rear = 0;
int queue_size = 0;

void enqueue(f_packet_t packet) {
    int next_rear = (rear + 1) % MAX_SIZE;
    if (next_rear == front) {
        return;
    }
    queue[rear] = packet;
    rear = next_rear;
    queue_size++;
}

f_packet_t dequeue(void) {
    if (front == rear) {
        f_packet_t empty_packet = {0};
        return empty_packet;
    }
    f_packet_t packet = queue[front];
    front = (front + 1) % MAX_SIZE;
    queue_size--;
    return packet;
}

void clear(void) {
    front = 0;
    rear = 0;
    queue_size = 0;
}

int size(void) {
    return queue_size;
}

bool rx_can_msg(CANRxFrame *rxmsg, packet_t *packet) {
  (void)packet;
  f_packet_t tmprx = {0};
  tmprx.cmd_code = cmd_j2534_read_message;
  memcpy(tmprx.data, rxmsg, sizeof(CANRxFrame));
  tmprx.data_len = sizeof(CANRxFrame);
  enqueue(tmprx);
  return false;
}

uint64_t handle_hscan_connect(uint64_t flags, uint64_t baudrate) {
  (void)flags; //TODO
  clear();
  registerHsCanCallback(&rx_can_msg);
  if (!canBaudRate(&hsCanConfig, baudrate)) {
    return ERR_INVALID_BAUDRATE;
  }
  canMemorryConfig(&CAND1, &hsCanConfig, &hscan_ram_cfg, &hscan_ram);
  canGlobalFilter(&hsCanConfig, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(&CAND1, &hsCanConfig);
  return STATUS_NOERROR;
}

uint64_t handle_swcan_connect(uint64_t flags, uint64_t baudrate) {
  (void)flags; //TODO
  if (!canBaudRate(&swCanConfig, baudrate)) {
    return ERR_INVALID_BAUDRATE;
  }
  canMemorryConfig(&CAND2, &swCanConfig, &swcan_ram_cfg, &swcan_ram);
  canGlobalFilter(&swCanConfig, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(&CAND2, &swCanConfig);
  return STATUS_NOERROR;
}

bool j2534_connect(packet_t *rx_packet, packet_t *tx_packet) {
  //packet u8 ID, u16 len, 24 x u8, term u8)
  if(rx_packet->data_len != 24)
    return prepareReplyPacket(tx_packet, rx_packet, (uint8_t*)ERR_NOT_SUPPORTED, 1, cmd_j2534_ack);
  uint64_t protocolID, flags, baud, error = ERR_NOT_SUPPORTED;
  memcpy(&protocolID, rx_packet->data, 8);
  memcpy(&flags, rx_packet->data + 8, 8);
  memcpy(&baud, rx_packet->data + 16, 8);

  switch (protocolID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_connect(flags, baud);
    break;
  case SW_CAN_PS:
    error = handle_swcan_connect(flags, baud);
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);
  memcpy(retBuff + 8, &protocolID, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 16, cmd_j2534_ack);
}

uint64_t handle_hscan_disconnect(void) {
  canStop(&CAND1);
  return STATUS_NOERROR;
}

uint64_t handle_swcan_disconnect(void) {
  canStop(&CAND2);
  return STATUS_NOERROR;
}

bool j2534_disconnect(packet_t *rx_packet, packet_t *tx_packet) {
  if(rx_packet->data_len != 8)
    return prepareReplyPacket(tx_packet, rx_packet, (uint8_t*)ERR_NOT_SUPPORTED, 1, cmd_j2534_ack);
  uint64_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 8);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_disconnect();
    break;
  case SW_CAN_PS:
    error = handle_swcan_disconnect();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

uint64_t handle_hscan_filter(void) {
  canGlobalFilter(&hsCanConfig, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStop(&CAND1);
  canStart(&CAND1, &hsCanConfig);
  return STATUS_NOERROR;
}

uint64_t handle_swcan_filter(void) {
  canGlobalFilter(&swCanConfig, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStop(&CAND2);
  canStart(&CAND2, &swCanConfig);
  return STATUS_NOERROR;
}

bool j2534_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint64_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 8);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_filter();
    break;
  case SW_CAN_PS:
    error = handle_swcan_filter();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

bool j2534_ioctl(packet_t *rx_packet, packet_t *tx_packet) {
  uint64_t channelID, ioctl, error = ERR_NOT_SUPPORTED;
  uint8_t retSize = 8;
  memcpy(&channelID, rx_packet->data, 8);
  memcpy(&ioctl, rx_packet->data + 8, 8);

  switch(ioctl) {
  case GET_CONFIG:
    //const SCONFIG_LIST *inputlist = pInput;
    break;
  case SET_CONFIG:
    //const SCONFIG_LIST *inputlist = pInput;
    break;
  case READ_VBATT:
    uint16_t vBat = getSupplyVoltage();
    memcpy(retBuff + 8, &vBat, 2);
    retSize = 10;
    error = STATUS_NOERROR;
    break;
  case CLEAR_RX_BUFFER:
    clear();
    error = STATUS_NOERROR;
    break;
  case FIVE_BAUD_INIT:
  case FAST_INIT:
  case CLEAR_TX_BUFFER:
  case CLEAR_PERIODIC_MSGS:
  case CLEAR_MSG_FILTERS:
  case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
  case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
  case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
  case READ_PROG_VOLTAGE:
  default:
      break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, retSize, cmd_j2534_ack);;
}

uint64_t handle_hscan_read_message(void) {
  return ERR_NOT_SUPPORTED;
}

uint64_t handle_swcan_read_message(void) {
  return ERR_NOT_SUPPORTED;
}

bool j2534_read_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint64_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 8);
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_read_message();
    if (size() > 0) {
      f_packet_t tmp = dequeue();
      tx_packet->cmd_code = tmp.cmd_code;
      tx_packet->data_len = tmp.data_len;
      tx_packet->data = tmp.data;
      tx_packet->term = tmp.term;
      //TODO: temp
      return true;
    }
    error = ERR_BUFFER_EMPTY;
    break;
  case SW_CAN_PS:
    error = handle_swcan_read_message();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

bool j2534_write_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint64_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 8);

  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    CANTxFrame tx = {};
    memcpy(&tx, rx_packet->data + 16, sizeof(CANTxFrame));

    if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &tx, TIME_MS2I(100)) == MSG_OK) {
     error = STATUS_NOERROR;
    }
    //error = handle_hscan_filter();
    break;
  case SW_CAN_PS:
    //error = handle_swcan_filter();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

bool j2534_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint64_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 8);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    //error = handle_hscan_filter();
    break;
  case SW_CAN_PS:
    //error = handle_swcan_filter();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 8);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
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
    return j2534_filter(rx_packet, tx_packet);
  case cmd_j2534_read_message:
    return j2534_read_message(rx_packet, tx_packet);
  case cmd_j2534_write_message:
    return j2534_write_message(rx_packet, tx_packet);
  case cmd_j2534_periodic_message:
    return j2534_periodic_message(rx_packet, tx_packet);
  default:
    break;
  }
  return false;
}
