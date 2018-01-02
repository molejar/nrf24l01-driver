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
 * Module configuration macros
 */

/* RX FIFO Size. Default: 210 Bytes */
#if !defined(NRF24L01_RX_FIFO_SIZE)
#define NRF24L01_RX_FIFO_SIZE     210
#endif

/* Mask Global IRQ if read/write FIFO data
 * 0) Kept enabled
 * 1) Disable (default) */
#if !defined(NRF24L01_RX_FIFO_DISIRQ)
#define NRF24L01_RX_FIFO_DISIRQ   1
#endif

/* Configuration structure type
 * 0) Raw (without adaptation to CYRF)
 * 1) New (default) */
#if !defined(NRF24L01_CONFIG_TYPE)
#define NRF24L01_CONFIG_TYPE      1
#endif

/* Runtime PIPE Configuration
 * 0) Remove
 * 1) Include */
#if !defined(NRF24L01_PIPE_RTCONF)
#define NRF24L01_PIPE_RTCONF      0
#endif

/* The NRF24L01 FIFOs : RX/TX three level, 32 byte */
#define NRF24L01_TX_HWFIFO_SIZE   32
#define NRF24L01_RX_HWFIFO_SIZE   32


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
    ACK_Disabled,
    ACK_Enabled
  };

  enum crc_t {
    CRC_Disabled      = (0x00),
    CRC_8bit          = (0x01 << 3),
    CRC_16bit         = ((0x01 << 3) | (0x01 << 2))
  };

  enum txpwr_t {
    TXPWR_Plus_0dBm   = (0x03 << 1),
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
    FIFO_RXTX         = 0x03,
    FIFO_RXSW         = 0x04
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

  enum event_t {
    EVENT_MAX_RT       = (0x01 << 4),
    EVENT_TX_DS        = (0x01 << 5),
    EVENT_RX_DR        = (0x01 << 6),
    EVENT_TX_DONE      = (0x01 << 7),
    EVENT_ANY          = (0x0F << 4)
  };

  struct conf_t {
    crc_t    crc;
    txpwr_t  txPwr;
    drate_t  dataRate;
    uint8_t  channel;
    uint8_t  plSize;

    ack_t    ackMode;
    uint16_t ackDelay;
    uint8_t  ackCount;

    uint8_t  addrWidth;
    uint8_t  onPipes;
    uint64_t rxAddrP0;
    uint64_t rxAddrP1;
    uint8_t  rxAddrP2;
    uint8_t  rxAddrP3;
    uint8_t  rxAddrP4;
    uint8_t  rxAddrP5;
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
   * @param pwr  mbed pin to use for the chip power enable (optional, in default not connected - NC).
   * @param irq  mbed pin to use for the interrupt request line (optional, in default not connected - NC).
   */
  NRF24L01(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName pwr = NC, PinName irq = NC);

  /**
   * Reset NRF24L01 Device
   *
   *  @return True if successful
   */
  bool reset(void);

  /**
   * Open NRF24L01 Device
   *
   * @param conf Pointer to configuration structure
   *
   * @return True if successful
   */
  bool open(const conf_t *conf = NULL);

  /**
   * Close NRF24L01 Device
   */
  void close(void);

  /**
   * Set the operation mode
   *
   * @param mode
   */
  void set_op_mode(mode_t mode);

  /**
   * Get info about actual operation mode
   *
   * @return The value representing actual operation mode
   */
  mode_t get_op_mode(void) { return m_opMode; }

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
  bool set_rx_pipe(uint8_t pipe, bool enable = true, bool autoAck = false, uint8_t plSize = 0xFF);

  /**
   * Read current configuration of pipes
   */
  int get_enabled_pipes(void);

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
  void set_rx_address(uint8_t pipe, uint64_t address);

  /**
   * Set the TX address (width 3 - 5 bytes)
   *
   * @param address
   */
  void set_tx_address(uint64_t address, bool ack=true);

  /**
   * Set the channel number
   *
   * @param channel
   */
  void set_channel(uint8_t channel);

  /**
   * Set the RF output power.
   *
   * @param power The RF output power in dBm (0, -6, -12 or -18).
   */
  void set_tx_power(txpwr_t power);

  /**
   * Set the Air data rate.
   *
   * @param rate The air data rate in kbps (250, 1M or 2M).
   */
  void set_data_rate(drate_t rate);

  /**
   * Set AutoRetransmit function
   *
   * @param delay The delay between restransmit's, in 250us steps  (250 - 4000)
   * @param count number of retransmits before generating an error (1..15)
   */
  void set_auto_retrans(uint16_t delay_us, uint8_t count);

  /**
   * Get RX/TX FIFO status
   *
   * @return FIFO status value
   */
  uint8_t get_fifo_status(void);

  /**
   * Flush receive FIFO
   *
   * @param val (FIFO_RX, FIFO_TX or FIFO_RXTX)
   */
  void flush_fifo(fifo_t val = FIFO_RXTX);

  /**
   * Test if any RX pipe has readable data
   *
   * @return True if RX data ready
   */
  bool readable(void);

  /**
   * Read data from RX Pipe
   *
   * @param pipe the pipe number to get data from
   * @param data pointer to an array of bytes to store the received data
   * @param len  the count of bytes read from receive buffer
   * @return the number of bytes actually received, 0 if none are received, or -1 for an error
   */
  int rx_data(uint8_t *pipe, uint8_t *data, uint8_t len);

  /**
   * Transmit data
   *
   * @param data pointer to an array of bytes to write
   * @param len  the number of bytes to send
   * @return the number of bytes actually written, or -1 for an error
   */
  int tx_data(const uint8_t *data, uint16_t len, bool ack = true);

  /**
   * Transmit data
   *
   * @param pipe the pipe number where will data send
   * @param data pointer to an array of bytes to write
   * @param len  the number of bytes to send
   * @return the number of bytes actually written, or -1 for an error
   */
  int tx_data(uint8_t pipe, const uint8_t *data, uint16_t len, bool ack = true);

  /**
   * Transmit data
   *
   * @param address
   * @param data    pointer to an array of bytes to write
   * @param count   the number of bytes to send
   * @return the number of bytes actually written, or -1 for an error
   */
  int tx_data_to_address(uint64_t address, const uint8_t *data, uint16_t len, bool ack = true);

  /**
   * Register user callback function
   *
   * @param cbfunc
   * @param mask
   */
  void attach_cb(rfcb_t cbfunc, uint8_t event=EVENT_ANY) { m_cbFunc = cbfunc; m_cbMask = event; };

  /**
   * Reuse last transmitted payload.
   */
  void reuse_tx_payload(void);

  /**
   * Set ACK payload which will transmit with every ACK message
   *
   * @param pipe Rx Pipe number (0 - 5)
   * @param data Pointer to data buffer
   * @param len Size of data buffer
   */
  int set_ack_payload(uint8_t pipe, const uint8_t *data, uint8_t len);


  /***************************************************************************************************************
   * Service Methods
   ***************************************************************************************************************/

  /**
   * RX Power Detector
   *
   * @return True if received power levels is above -64 dBm, other False
   */
  bool is_carrier_detected(void);

  /**
   * Start the RF output test in NRF24L01
   * Can be used for measuring the RF output performance
   */
  void start_carrier_wave_test(void);

  /**
   * Stop the RF output test in NRF24L01
   */
  void stop_carrier_wave_test(void);

  /**
   * Read NRF24L01 register
   *
   * @param reg Register address enum (see reg_t)
   * @return Register value
   */
  uint8_t read_reg(reg_t reg) {
    uint8_t val;
    rd_reg(reg, &val);
    return val;
  }

  /**
   * Write NRF24L01 register
   *
   * @param reg Register address enum (see reg_t)
   * @param val The value to be write
   */
  void write_reg(reg_t reg, uint8_t val) { wr_reg(reg, val); }

  /**
   *
   * @param val
   * @return
   */
  uint32_t get_status(stat_t val);

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
  int rd_reg(uint8_t addr, uint8_t* pVal);

  /**
   * Read the content of addressable registers in NRF24L01
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer
   * @param len Size of the buffer
   * @return The content of status register
   */
  int rd_reg(uint8_t addr, uint8_t* pBuff, uint8_t len);

  /**
   * Write the content of an addressable register in NRF24L01
   *
   * @param addr The address of the register
   * @param val  The value to be write into the register
   * @return The content of status register
   */
  int wr_reg(uint8_t addr, uint8_t val);

  /**
   * Write the content of addressable registers in NRF24L01
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer with the content
   * @param len Size of the buffer
   * @return The content of status register
   */
  int wr_reg(uint8_t addr, const uint8_t* pBuff, uint8_t len);

  /**
   *
   * @param pipe
   * @param pBuff
   * @param len
   * @return
   */
  int rd_data(uint8_t *pipe, uint8_t *pBuff, uint8_t len);

  /**
   *
   * @param pipe
   * @param pBuff
   * @param len
   * @param ack
   * @return
   */
  int wr_data(const uint8_t *pBuff, uint16_t len, bool ack);

  /**
   * Get the contents of the status register and clear IRQ flags.
   *
   * @return the contents of the status register
   */
  uint8_t get_status();

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
  uint8_t      *m_pBuff;
  uint8_t      *m_pBuffRdI;
  uint8_t      *m_pBuffWrI;
  uint16_t      m_buffPlCnt;
  uint16_t      m_buffSize;
  uint16_t      m_buffFree;

  /* TX Control */
  const uint8_t *m_pTxIndex;
  uint16_t      m_txLen;
  bool          m_txAck;

  /* RX CallBack function pointer */
  rfcb_t        m_cbFunc;
  uint8_t       m_cbMask;

  /* Operation Mode */
  mode_t        m_opMode;

  /* RX/TX Errors */
  uint32_t      m_rxErr;
  uint32_t      m_txErr;
  uint8_t       m_arcnt;
};

#endif /* __NRF24L01_H__ */
