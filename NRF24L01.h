/**
 * @file NRF24L01.h
 *
 * @author Martin Olejar
 *
 * @section LICENSE
 *
 * Copyright (c) 2017 Martin Olejar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @section DESCRIPTION
 *
 * NRF24L01+ Single Chip 2.4GHz Transceiver from Nordic Semiconductor.
 *
 * Datasheet:
 *
 * http://www.nordicsemi.no/files/Product/data_sheet/NRF24L01P_Product_Specification_1_0.pdf
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

/**
 * Includes
 */
#include "mbed.h"

/**
 * Macros
 */
#define RX_FIFO_IRQ             1

/* The NRF24L01 FIFOs : RX/TX three level, 32 byte */
#define NRF24L01_TX_FIFO_SIZE   32
#define NRF24L01_RX_FIFO_SIZE   32

/**
 * NRF24L0101+ Single Chip 2.4GHz Transceiver from Nordic Semiconductor.
 */
class NRF24L01 {

public:
  enum reg_t {
    REG_CONFIG = 0x00,
    REG_EN_AA,
    REG_EN_RXADDR,
    REG_SETUP_AW,
    REG_SETUP_RETR,
    REG_RF_CH,
    REG_RF_SETUP,
    REG_STATUS,
    REG_OBSERVE_TX,
    REG_RPD,
    REG_RX_PW_P0 = 0x11,
    REG_RX_PW_P1,
    REG_RX_PW_P2,
    REG_RX_PW_P3,
    REG_RX_PW_P4,
    REG_RX_PW_P5,
    REG_FIFO_STATUS,
    REG_DYNPD = 0x1c,
    REG_FEATURE
  };

  enum diag_t {
    DIAG_RX_ERR,
    DIAG_TX_ERR,
    DIAG_ARCNT
  };

  enum mode_t {
    MODE_POWER_DOWN,
    MODE_STANDBY,
    MODE_RX,
    MODE_TX,
  };

  enum ack_t {
    ACK_DISABLED,
    ACK_HW,
    ACK_SW
  };

  enum crc_t {
    CRC_Disabled      = (0x00),
    CRC_8bit          = (0x01 << 3),
    CRC_16bit         = ((0x01 << 3) | (0x01 << 2))
  };

  enum txpwr_t {
    TXPWR_0dBm        = (0x03 << 1),
    TXPWR_Minus_6dBm  = (0x02 << 1),
    TXPWR_Minus_12dBm = (0x01 << 1),
    TXPWR_Minus_18dBm = (0x00 << 1)
  };

  enum drate_t {
    DRATE_250kBps     = (0x01 << 5),
    DRATE_1MBps       = (0),
    DRATE_2MBps       = (0x01 << 3)
  };

  enum fifo_t {
    FIFO_RX           = 0x01,
    FIFO_TX           = 0x02,
    FIFO_RXTX         = 0x03
  };

  enum stat_t {
    STAT_TX_FULL      = (0x01 << 0),
    STAT_MAX_RT       = (0x01 << 4),
    STAT_TX_DS        = (0x01 << 5),
    STAT_RX_DR        = (0x01 << 6)
  };

  enum fstat_t {
    FSTAT_RX_EMPTY     = (0x01 << 0),
    FSTAT_RX_FULL      = (0x01 << 1),
    FSTAT_TX_EMPTY     = (0x01 << 4),
    FSTAT_TX_FULL      = (0x01 << 5),
    FSTAT_TX_REUSE     = (0x01 << 6)
  };

  struct conf_t {
    crc_t    crc;
    mode_t   defMode;

    ack_t    ackMode;
    uint16_t ackDelay;
    uint8_t  ackCount;

    txpwr_t  txPwr;
    drate_t  dataRate;
    uint8_t  channel;

    uint8_t  addrWidth;
    uint8_t  onPipes;
    uint64_t rxAddrP0;
    uint64_t rxAddrP1;
    uint8_t  rxAddrP2;
    uint8_t  rxAddrP3;
    uint8_t  rxAddrP4;
    uint8_t  rxAddrP5;

    uint8_t  plSize;
  };

  /**
   * Type definition of NRF24L01 callback function
   *
   * @param obj
   * @param event
   */
  typedef void (*rfcb_t)(NRF24L01 *obj, uint8_t event);

  /**
   * Constructor.
   *
   * @param mosi mbed pin to use for MOSI line of SPI interface.
   * @param miso mbed pin to use for MISO line of SPI interface.
   * @param sck  mbed pin to use for SCK line of SPI interface.
   * @param csn  mbed pin to use for chip select line of SPI interface.
   * @param ce   mbed pin to use for the chip enable line.
   * @param irq  mbed pin to use for the interrupt request line.
   */
  NRF24L01(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName pwr = NC, PinName irq = NC);

  /**
   * Reset NRF24L01 Device
   *
   *  @return True if successful
   */
  bool Reset(void);

  /**
   * Open NRF24L01 Device
   *
   * @param conf Pointer to configuration structure
   *
   * @return True if successful
   */
  bool Open(const conf_t *conf = NULL);

  /**
   * Close NRF24L01 Device
   */
  void Close(void);

  /**
   * Set the operation mode
   *
   * @param mode
   */
  bool SetMode(mode_t mode = (mode_t)0xFF);

  /**
   * Get info about actual operation mode
   *
   * @return The value representing actual operation mode
   */
  mode_t GetActualMode(void) { return m_actualMode; }

  /**
   * Set the default operation mode
   *
   * @param mode
   */
  void SetDefaultMode(mode_t mode) { m_defaultMode = mode; }

  /**
   * Configure RX Pipe
   *
   * @param pipe     The pipe index (0 - 5)
   * @param enable   True or False (default: true)
   * @param autoAck  Auto acknowledge feature (default: false)
   * @param size     RX Payload size:
   *                 Use 0 for dynamic size or 1 - 32 bytes for fixed size
   *                 If use 0xFF, the value will be reused from conf_t struct. (default: 0xFF)
   */
  bool ConfRxPipe(uint8_t pipe, bool enable = true, bool autoAck = false, uint8_t plSize = 0xFF);

  /**
   * Read current configuration of pipes
   */
  int GetEnabledPipes(void);

  /**
   * Set the RX address for particular pipe.
   *
   * @param pipe    RX pipe (0..5)
   * @param address address value associated with the particular pipe
   *
   * Note that Pipes 0 & 1 have 3, 4 or 5 byte addresses,
   *  while Pipes 2..5 only use the lowest byte (bits 7..0) of the
   *  address provided here, and use 2, 3 or 4 bytes from Pipe 1's address.
   *  The width parameter is ignored for Pipes 2..5.
   */
  void SetRxAddr(uint8_t pipe, uint64_t address);

  /**
   * Set the TX address (width 3 - 5 bytes)
   *
   * @param address
   */
  void SetTxAddr(uint64_t address);

  /**
   * Set the TX address to particular pipe.
   *
   * @param pipe RX pipe (0..5)
   */
  void SetTxAddrToPipe(uint8_t pipe);

  /**
   * Set the channel number
   *
   * @param channel
   */
  void SetChannel(uint8_t channel);

  /**
   * Set the RF output power.
   *
   * @param power The RF output power in dBm (0, -6, -12 or -18).
   */
  void SetTxPower(txpwr_t power);

  /**
   * Set the Air data rate.
   *
   * @param rate The air data rate in kbps (250, 1M or 2M).
   */
  void SetDataRate(drate_t rate);

  /**
   * Set AutoRetransmit function
   *
   * @param delay The delay between restransmit's, in 250us steps  (250 - 4000)
   * @param count number of retransmits before generating an error (1..15)
   */
  void SetAutoRetrans(uint16_t delay_us, uint8_t count);

  /**
   * Get RX/TX FIFO status
   *
   * @return FIFO status value
   */
  uint8_t GetFifoStatus(void);

  /**
   * Flush receive FIFO
   *
   * @param val (FIFO_RX, FIFO_TX or FIFO_RXTX)
   */
  void FlushFifo(fifo_t val = FIFO_RXTX);

  /**
   * Test if any RX pipe has readable data
   *
   * @return True if RX data ready
   */
  bool IsReadable(void);

  /**
   * Read data from RX Pipe
   *
   * @param pipe the receive pipe to get data from
   * @param data pointer to an array of bytes to store the received data
   * @param len the number of bytes to read from receive buffer (1..32)
   * @return the number of bytes actually received, 0 if none are received, or -1 for an error
   */
  int RxData(uint8_t *pipe, uint8_t *data, uint8_t len = NRF24L01_RX_FIFO_SIZE);

  /**
   * Transmit data
   *
   * @param data pointer to an array of bytes to write
   * @param count the number of bytes to send (1..32)
   * @return the number of bytes actually written, or -1 for an error
   */
  int TxData(const uint8_t *data, uint8_t len = NRF24L01_TX_FIFO_SIZE, bool ack = true);

  /**
   * Register user callback function
   *
   * @param cbfunc
   * @param mask
   */
  void AttachCB(rfcb_t cbfunc, uint8_t mask) { m_cbFunc = cbfunc; m_cbMask = mask; };

  /**
   * Reuse last transmitted payload.
   */
  void ReuseTxPayload(void);

  /**
   * Set ACK payload which will transmit with every ACK message
   *
   * @param pipe Rx Pipe number (0 - 5)
   * @param data Pointer to data buffer
   * @param len Size of data buffer
   */
  void SetAckPayload(uint8_t pipe, const uint8_t *data, uint8_t len);


  /***************************************************************************************************************
   * Service Methods
   ***************************************************************************************************************/

  /**
   * RX Power Detector
   *
   * @return True if received power levels is above -64 dBm, other False
   */
  bool IsCarrierDetected(void);

  /**
   * Start the RF output test in NRF24L01
   * Can be used for measuring the RF output performance
   */
  void StartCarrierWaveTest(void);

  /**
   * Stop the RF output test in NRF24L01
   */
  void StopCarrierWaveTest(void);

  /**
   * Read NRF24L01 register
   *
   * @param reg Register address enum (see reg_t)
   * @return Register value
   */
  uint8_t ReadReg(reg_t reg) {
    uint8_t val;
    rd_nrf(reg, &val);
    return val;
  }

  /**
   * Write NRF24L01 register
   *
   * @param reg Register address enum (see reg_t)
   * @param val The value to be write
   */
  void WriteReg(reg_t reg, uint8_t val) { wr_nrf(reg, val); }

  /**
   *
   * @param val
   * @return
   */
  uint32_t GetStatus(stat_t val);

private:
  /**
   * GPIO IRQ CallBack function
   */
  void pin_irq(void);

  /**
   * Read the content of an addressable register in NRF24L01
   *
   * @param addr The address of the register
   * @param pVal The pointer to variable for read value
   * @return The content of status register
   */
  int rd_nrf(uint8_t addr, uint8_t* pVal);

  /**
   * Read the content of addressable registers in NRF24L01
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer
   * @param len Size of the buffer
   * @return The content of status register
   */
  int rd_nrf(uint8_t addr, uint8_t* pBuff, uint8_t len);

  /**
   * Write the content of an addressable register in NRF24L01
   *
   * @param addr The address of the register
   * @param val  The value to be write into the register
   * @return The content of status register
   */
  int wr_nrf(uint8_t addr, uint8_t val);

  /**
   * Write the content of addressable registers in NRF24L01
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer with the content
   * @param len Size of the buffer
   * @return The content of status register
   */
  int wr_nrf(uint8_t addr, const uint8_t* pBuff, uint8_t len);

  /**
   * Get the contents of the status register and clear IRQ flags.
   *
   * @return the contents of the status register
   */
  int get_status(void);

  /**
   * Get RX frame size
   *
   * @param pipe
   * @return
   */
  uint8_t  get_rx_frame_size(uint8_t pipe);

  /**
   * Read RX payload width for the top of FIFO
   *
   * @return size of received payload
   */
  uint8_t get_rx_payload_width(void);

  /**
   * Write value into feature register
   *
   * @param val register value
   * @return True if successful other False
   */
  bool wr_feature_reg(uint8_t val);

  /* SPI interface */
  SPI           m_spi;
  DigitalOut    m_cs;
  DigitalOut    m_ce;
  DigitalOut    m_pwr;
  InterruptIn   m_irq;

  bool          m_useIrqPin;
  bool          m_usePwrPin;
  const conf_t *m_conf;

  /* RX FIFO buffer */
  uint8_t      *m_pHead;
  uint8_t      *m_pTail;
  uint8_t      *m_pRdIndex;
  uint8_t      *m_pWrIndex;
  uint8_t       m_rxItmSize;
  uint8_t       m_rxSize;
  uint8_t       m_rxCount;

  /* Automatic Acknowledge  */
  ack_t         m_autoAck;

  /* RX CallBack function pointer */
  rfcb_t        m_cbFunc;
  uint8_t       m_cbMask;

  /* Operation Mode */
  mode_t        m_actualMode;
  mode_t        m_defaultMode;

  /* RX/TX Errors */
  uint32_t      m_rxErr;
  uint32_t      m_txErr;
  uint8_t       m_arcnt;
};

#endif /* __NRF24L01_H__ */
