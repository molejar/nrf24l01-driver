/**
 * @file NRF24L01.cpp
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
 */

#include "NRF24L01.h"

#if(NRF24L01_RX_FIFO_DISIRQ)
#define fifo_enable_irq()  __enable_irq()
#define fifo_disable_irq() __disable_irq()
#else
#define fifo_enable_irq()
#define fifo_disable_irq()
#endif

/* 0) Keep CE enabled if TX
 * 1) Generate only 10us pulse on CE if TX */
#define NRF24L01_TX_CE_OFF                 1

/* The NRF24L0101 SPI Commands */
#define NRF24L01_SPI_CMD_RD_REG            0x00
#define NRF24L01_SPI_CMD_WR_REG            0x20
#define NRF24L01_SPI_CMD_RD_RX_PAYLOAD     0x61
#define NRF24L01_SPI_CMD_WR_TX_PAYLOAD     0xa0
#define NRF24L01_SPI_CMD_FLUSH_TX          0xe1
#define NRF24L01_SPI_CMD_FLUSH_RX          0xe2
#define NRF24L01_SPI_CMD_REUSE_TX_PL       0xe3
#define NRF24L01_SPI_CMD_RD_RX_PL_WID      0x60
#define NRF24L01_SPI_CMD_WR_ACK_PAYLOAD    0xa8
#define NRF24L01_SPI_CMD_WR_TX_PNOACK      0xb0
#define NRF24L01_SPI_CMD_NOP               0xff
/* Activate Chip Features Command */
#define NRF24L01_SPI_CMD_ACTIVATE          0x50

/* The NRF24L01 Registers collection */
#define NRF24L01_REG_CONFIG                0x00
#define NRF24L01_REG_EN_AA                 0x01
#define NRF24L01_REG_EN_RXADDR             0x02
#define NRF24L01_REG_SETUP_AW              0x03
#define NRF24L01_REG_SETUP_RETR            0x04
#define NRF24L01_REG_RF_CH                 0x05
#define NRF24L01_REG_RF_SETUP              0x06
#define NRF24L01_REG_STATUS                0x07
#define NRF24L01_REG_OBSERVE_TX            0x08
#define NRF24L01_REG_RPD                   0x09
#define NRF24L01_REG_RX_ADDR_P0            0x0a
#define NRF24L01_REG_RX_ADDR_P1            0x0b
#define NRF24L01_REG_RX_ADDR_P2            0x0c
#define NRF24L01_REG_RX_ADDR_P3            0x0d
#define NRF24L01_REG_RX_ADDR_P4            0x0e
#define NRF24L01_REG_RX_ADDR_P5            0x0f
#define NRF24L01_REG_TX_ADDR               0x10
#define NRF24L01_REG_RX_PW_P0              0x11
#define NRF24L01_REG_RX_PW_P1              0x12
#define NRF24L01_REG_RX_PW_P2              0x13
#define NRF24L01_REG_RX_PW_P3              0x14
#define NRF24L01_REG_RX_PW_P4              0x15
#define NRF24L01_REG_RX_PW_P5              0x16
#define NRF24L01_REG_FIFO_STATUS           0x17
#define NRF24L01_REG_DYNPD                 0x1c
#define NRF24L01_REG_FEATURE               0x1d

#define NRF24L01_REG_ADDRESS_MASK          0x1f

/* CONFIG register flags */
#define NRF24L01_CONFIG_PRIM_RX            (1 << 0)
#define NRF24L01_CONFIG_PWR_UP             (1 << 1)
#define NRF24L01_CONFIG_CRC0               (1 << 2)
#define NRF24L01_CONFIG_EN_CRC             (1 << 3)
#define NRF24L01_CONFIG_MAX_RT_MASK        (1 << 4)
#define NRF24L01_CONFIG_TX_DS_MASK         (1 << 5)
#define NRF24L01_CONFIG_RX_DR_MASK         (1 << 6)

/* CRC options flags */
#define NRF24L01_CONFIG_CRC_MASK           (NRF24L01_CONFIG_EN_CRC|NRF24L01_CONFIG_CRC0)
#define NRF24L01_CONFIG_CRC_NONE           (0)
#define NRF24L01_CONFIG_CRC_8BIT           (NRF24L01_CONFIG_EN_CRC)
#define NRF24L01_CONFIG_CRC_16BIT          (NRF24L01_CONFIG_EN_CRC|NRF24L01_CONFIG_CRC0)

/* RF_SETUP register mask's */
#define NRF24L01_RF_SETUP_CONT_WAVE        (1 << 7)
#define NRF24L01_RF_SETUP_PLL_LOCK         (1 << 4)
#define NRF24L01_RF_SETUP_TX_PWR_MASK      (0x03 << 1)
#define NRF24L01_RF_SETUP_DR_MASK          ((1 << 3) | (1 << 5))

/* SETUP_AW */
#define NRF24L01_REG_SETUP_AW_3_BYTE       1
#define NRF24L01_REG_SETUP_AW_4_BYTE       2
#define NRF24L01_REG_SETUP_AW_5_BYTE       3

/* FIFO Status register flags */
#define NRF24L01_FIFOST_TX_REUSE           (0x01 << 6)
#define NRF24L01_FIFOST_TX_FULL            (0x01 << 5)
#define NRF24L01_FIFOST_TX_EMPTY           (0x01 << 4)
#define NRF24L01_FIFOST_RX_FULL            (0x01 << 1)
#define NRF24L01_FIFOST_RX_EMPTY           (0x01 << 0)

/* STATUS register flags */
#define NRF24L01_STATUS_TX_FULL            (0x01 << 0)
#define NRF24L01_STATUS_RX_PIPE_MASK       (0x07 << 1)
#define NRF24L01_STATUS_RX_PIPE(reg)       (((reg) >> 1) & 0x07)
#define NRF24L01_STATUS_MAX_RT             (0x01 << 4)
#define NRF24L01_STATUS_TX_DS              (0x01 << 5)
#define NRF24L01_STATUS_RX_DR              (0x01 << 6)

/* FEATURE register flags */
#define NRF24L01_FEAT_DISABLED             (0x00)
#define NRF24L01_FEAT_EN_DPL               (0x01 << 2)
#define NRF24L01_FEAT_EN_ACKP              (0x01 << 1)
#define NRF24L01_FEAT_EN_NOACK             (0x01)
#define NRF24L01_FEAT_EN_ALL               (0x07)

/* RX_PW_P0..RX_PW_P5 registers */
#define NRF24L01_RX_PW_PX_MASK             0x3F

/* Communication Delays */
#define NRF24L01_DELAY_TSTBY2A_US          130   // 130uS
#define NRF24L01_DELAY_THCE_US             10    //  10uS
#define NRF24L01_DELAY_TPD2STBY_US         4500  // 4.5mS worst case
#define NRF24L01_DELAY_TPECE2CSN_us        4     //   4uS


/* RX FIFO Buffer Declaration */
static uint8_t rxBuffer[NRF24L01_RX_FIFO_SIZE];

/* Default configuration */
static const NRF24L01::conf_t defConf = {
  /* .crc        = */ NRF24L01::CRC_16bit,
  /* .txPwr      = */ NRF24L01::TXPWR_Plus_0dBm,
  /* .dataRate   = */ NRF24L01::DRATE_2MBps,
  /* .channel    = */ 1,     /* Max: 127 */
  /* .plSize     = */ 32,    /* Min: 1; Max: 32; 0 for Dynamic PL */

  /* .ackMode    = */ NRF24L01::ACK_Enabled,
  /* .ackDelay   = */ 500,   /* Max: 4000 us */
  /* .ackCount   = */ 3,     /* Max: 15 */

  /* .addrWidth  = */ 5,     /* Min: 3; Max: 5 */
  /* .onPipes    = */ 6,     /* Min: 1; Max: 6 */
  /* .rxAddrP0   = */ 0xE7E7E7E7E7,
  /* .rxAddrP1   = */ 0xC2C2C2C2C2,
  /* .rxAddrP2   = */ 0xC3,
  /* .rxAddrP3   = */ 0xC4,
  /* .rxAddrP4   = */ 0xC5,
  /* .rxAddrP5   = */ 0xC6
};


/**
 * Class Methods
 */

NRF24L01::NRF24L01(PinName mosi,
                   PinName miso,
                   PinName sck,
                   PinName csn,
                   PinName ce,
                   PinName pwr,
                   PinName irq) : m_spi(mosi, miso, sck), m_cs(csn), m_ce(ce), m_pwr(pwr), m_irq(irq)
{
  m_conf = NULL;
  m_cbFunc = NULL;
  m_cbMask = 0;
  m_rxErr = 0;
  m_txErr = 0;
  m_arcnt = 0;
  //m_autoAck = ACK_DISABLED;
  m_opMode = MODE_POWER_DOWN;
  m_usePwrPin = (pwr != NC) ? true : false;
  m_useIrqPin = (irq != NC) ? true : false;
  // RX FIFO Buffer initialization
  m_pBuff     = rxBuffer;
  m_pBuffRdI  = m_pBuff;
  m_pBuffWrI  = m_pBuff;
  m_buffSize  = NRF24L01_RX_FIFO_SIZE;
  m_buffFree  = NRF24L01_RX_FIFO_SIZE;
  m_buffPlCnt = 0;
  // TX Control
  m_pTxIndex  = NULL;
  m_txAck     = false;
  m_txLen     = 0;
  // Configure communication bus
  if(m_usePwrPin) m_pwr = 1; // ...
  m_ce = 0;                  // Disable NRF24L01 TX/RX
  m_cs = 1;                  // Release SPI CS
  m_spi.frequency(8000000);  // Set SPI speed (max. 8MHz for NRF24L01)
  m_spi.format(8,0);         // Set SPI format (8-bit, ClockPhase = 0, ClockPolarity = 0)
}


bool NRF24L01::reset(void)
{
  uint8_t regval, fval = 0;
#if(NRF24L01_CONFIG_TYPE)
  uint8_t addr[5];
#endif

  m_rxErr = 0;
  m_txErr = 0;

  if(m_usePwrPin) {
    m_pwr = 1;
    wait_ms(5);
    m_pwr = 0;
    wait_ms(25);
    m_opMode = MODE_POWER_DOWN;
  } else {
    set_op_mode(MODE_POWER_DOWN);
  }

  // Configure CRC, IRQ and power mode
  regval = m_conf->crc;
  if(!m_useIrqPin) {
    regval |= (NRF24L01_CONFIG_MAX_RT_MASK |
               NRF24L01_CONFIG_RX_DR_MASK  |
               NRF24L01_CONFIG_TX_DS_MASK);
  }
  wr_reg(NRF24L01_REG_CONFIG, regval);

  // Set automatic retransmission delay and count
  set_auto_retrans(m_conf->ackDelay, m_conf->ackCount);

  // Set Channel
  wr_reg(NRF24L01_REG_RF_CH, m_conf->channel);

  // Set data rate and TX power
  regval  = m_conf->dataRate;
  regval |= m_conf->txPwr;
  wr_reg(NRF24L01_REG_RF_SETUP, regval);

  // Set address width in range (3 - 5)
  regval = 1;
  if (m_conf->addrWidth > 2) { regval = (m_conf->addrWidth - 2) & 0x03; }
  wr_reg(NRF24L01_REG_SETUP_AW, regval);

  // Get mask from on pipes
  regval  = ((1 << m_conf->onPipes) - 1) & 0x3F;
  regval |= 0x01; // NOTE: PIPE0 must be enabled anyway
  // Enable Rx pipes
  wr_reg(NRF24L01_REG_EN_RXADDR, regval);

  if(m_conf->ackMode == ACK_Enabled) {
    // Enable auto ACK
    fval |= (NRF24L01_FEAT_EN_NOACK|NRF24L01_FEAT_EN_ACKP);
    wr_reg(NRF24L01_REG_EN_AA, regval);
  } else {
    wr_reg(NRF24L01_REG_EN_AA, 0);
  }

  if((m_conf->plSize == 0) || (m_conf->ackMode == ACK_Enabled)) {
    // Enable dynamic payload
    fval |= NRF24L01_FEAT_EN_DPL;
    wr_reg(NRF24L01_REG_DYNPD, regval);
  } else {
    wr_reg(NRF24L01_REG_DYNPD, 0);
  }

  // Enable feature flags
  if(!wr_feature_reg(fval)) {
    return false;
  }

  // Set Default payload width
  for(int i = 0; i < 6; i++) {
    if(regval & (1 << i)) {
      wr_reg(NRF24L01_REG_RX_PW_P0 + i, m_conf->plSize);
    }
  }

  // Set NRF24L01 TX address same as PIPE0
  set_tx_address(m_conf->rxAddrP0, false);

  // Set NRF24L01 RX pipes address
  set_rx_address(0, m_conf->rxAddrP0);
  set_rx_address(1, m_conf->rxAddrP1);
  set_rx_address(2, m_conf->rxAddrP2);
  set_rx_address(3, m_conf->rxAddrP3);
  set_rx_address(4, m_conf->rxAddrP4);
  set_rx_address(5, m_conf->rxAddrP5);

  // Clear any pending interrupts
  wr_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_MAX_RT |
                              NRF24L01_STATUS_TX_DS  |
                              NRF24L01_STATUS_RX_DR);
  // Reset SW FIFO buffers
  m_buffPlCnt = 0;
  m_buffFree = m_buffSize;
  m_pBuffRdI = m_pBuffWrI = m_pBuff;

  return true;
}

bool NRF24L01::open(const conf_t *conf)
{
  m_conf = (conf == NULL) ? &defConf : conf;

  // Wait 100ms for NRF24L01 Power-on reset
  wait_ms(100);

  if(!reset())
    return false;

  if(m_useIrqPin) {
    m_irq.fall(this, &NRF24L01::pin_irq);
  }

  return true;
}

void NRF24L01::close(void)
{
  if(m_usePwrPin) {
    m_pwr = 1;
  } else {
    set_op_mode(MODE_POWER_DOWN);
  }
  if(m_useIrqPin) {
    m_irq.fall(NULL);
  }
}

void NRF24L01::set_op_mode(mode_t mode)
{
  uint8_t  regval;
  uint32_t wait_time = NRF24L01_DELAY_THCE_US;

  if(m_opMode == mode) {
    return;
  }

  m_ce = 0;   // CE must be 0
  rd_reg(NRF24L01_REG_CONFIG, &regval);

  switch(mode)
  {
    case MODE_POWER_DOWN:
      regval &= ~NRF24L01_CONFIG_PWR_UP;
      break;

    case MODE_STANDBY:
      regval |= NRF24L01_CONFIG_PWR_UP;
      if (m_opMode == MODE_POWER_DOWN) {
        wait_time = NRF24L01_DELAY_TPD2STBY_US;
      }
      break;

    case MODE_RX:
      if (m_opMode == MODE_POWER_DOWN) {
        regval |= NRF24L01_CONFIG_PWR_UP;
        wr_reg(NRF24L01_REG_CONFIG, regval);
        wait_us(NRF24L01_DELAY_TPD2STBY_US);
      }
      /* set to PRX */
      regval |= NRF24L01_CONFIG_PRIM_RX;
      wait_time = NRF24L01_DELAY_TSTBY2A_US;
      if(m_conf->ackMode == ACK_Enabled) {
        set_rx_address(0, m_conf->rxAddrP0);
      }
      break;

    case MODE_TX:
      if (m_opMode == MODE_POWER_DOWN) {
        regval |= NRF24L01_CONFIG_PWR_UP;
        wr_reg(NRF24L01_REG_CONFIG, regval);
        wait_us(NRF24L01_DELAY_TPD2STBY_US);
      }
      /* set to PTX */
      regval &= ~NRF24L01_CONFIG_PRIM_RX;
      //wait_time = NRF24L01_DELAY_TSTBY2A_US;
      break;
  }

  // Set Mode
  wr_reg(NRF24L01_REG_CONFIG, regval);
  if(mode == MODE_RX || mode == MODE_TX) {
    m_ce = 1; // Enable RX/TX mode
  }

#if(NRF24L01_TX_CE_OFF)
  if(mode == MODE_TX) {
    wait_us(NRF24L01_DELAY_THCE_US);
    m_ce = 0;
  }
#endif

  // Wait required time
  wait_us(wait_time);
  m_opMode = mode;
}

bool NRF24L01::set_rx_pipe(uint8_t pipe, bool enable, bool autoAck, uint8_t plSize)
{
  uint8_t regval, pmask = ((1 << pipe) & 0x3F);

  // Enable pipe
  rd_reg(NRF24L01_REG_EN_RXADDR, &regval);
  if(enable) { regval |=  pmask; }
  else       { regval &= ~pmask; }
  wr_reg(NRF24L01_REG_EN_RXADDR, regval);

  // Return if just disable
  if(!enable) return true;

  // Set Frame size
  if(plSize == 0xFF) plSize = m_conf->plSize;
  wr_reg(NRF24L01_REG_RX_PW_P0 + pipe, plSize & 0x3F);

  // Set auto ACK
  rd_reg(NRF24L01_REG_EN_AA, &regval);
  if(autoAck) { regval |=  pmask; }
  else        { regval &= ~pmask; }
  wr_reg(NRF24L01_REG_EN_AA, regval);

  // Set dynamic payload
  rd_reg(NRF24L01_REG_DYNPD, &regval);
  if(plSize == 0) { regval |=  pmask; }
  else               { regval &= ~pmask; }
  wr_reg(NRF24L01_REG_DYNPD, regval);

  if(autoAck || (plSize == 0)) {
    // enable dynamic payload and TX no ACK features
    pmask = NRF24L01_FEAT_EN_DPL | NRF24L01_FEAT_EN_NOACK;
    // First check if not enabled
    rd_reg(NRF24L01_REG_FEATURE, &regval);
    if((regval & pmask) != pmask) {
      wr_feature_reg(regval | pmask);
    }
  }

  return true;
}

int NRF24L01::get_enabled_pipes(void)
{
  uint8_t pipescfg;
  rd_reg(NRF24L01_REG_EN_RXADDR, &pipescfg);
  return pipescfg;
}

void NRF24L01::set_rx_address(uint8_t pipe, uint64_t address)
{
  uint8_t addr_arr[5], len = pipe > 1 ? 1 : 5;

  for(int i = 0; i < len; i ++) {
    addr_arr[i] = (uint8_t) (address >> (i * 8));
  }
  // Set RX address value
  wr_reg(NRF24L01_REG_RX_ADDR_P0 + pipe, addr_arr, len);
}

void NRF24L01::set_tx_address(uint64_t address, bool ack)
{
  uint8_t addr[5];

  for(int i = 0; i < m_conf->addrWidth; i++) {
    addr[i] = (uint8_t) (address >> (i * 8));
  }
  // Set TX address value
  wr_reg(NRF24L01_REG_TX_ADDR, addr, m_conf->addrWidth);
  // If HW ACK, then P0 address must be the same as TX address
  if(ack && m_conf->ackMode == ACK_Enabled) {
    wr_reg(NRF24L01_REG_RX_ADDR_P0, addr, m_conf->addrWidth);
  }
}

void NRF24L01::set_channel(uint8_t channel)
{
  if(channel > 127) channel = 127;
  wr_reg(NRF24L01_REG_RF_CH, channel);
}

void NRF24L01::set_tx_power(txpwr_t power)
{
  uint8_t temp;
  rd_reg(NRF24L01_REG_RF_SETUP, &temp);
  temp &= ~NRF24L01_RF_SETUP_TX_PWR_MASK;
  wr_reg(NRF24L01_REG_RF_SETUP, temp | power);
}

void NRF24L01::set_data_rate(drate_t rate)
{
  uint8_t temp;
  rd_reg(NRF24L01_REG_RF_SETUP, &temp);
  temp &= ~NRF24L01_RF_SETUP_DR_MASK;
  wr_reg(NRF24L01_REG_RF_SETUP, temp | rate);
}

void NRF24L01::set_auto_retrans(uint16_t delay_us, uint8_t count)
{
  uint8_t tmp = (delay_us / 250) & 0x0F;

  if(tmp > 0) tmp -= 1;
  wr_reg(NRF24L01_REG_SETUP_RETR, (tmp << 4) | (count & 0x0F));
}

uint8_t NRF24L01::get_fifo_status(void)
{
  uint8_t status;
  rd_reg(NRF24L01_REG_FIFO_STATUS, &status);
  return (status);
}

void NRF24L01::flush_fifo(fifo_t val)
{
  if(val & FIFO_RX) {
    wr_reg(NRF24L01_SPI_CMD_FLUSH_RX, 0);
  }

  if(val & FIFO_TX) {
    wr_reg(NRF24L01_SPI_CMD_FLUSH_TX, 0);
  }

  if(val == FIFO_RXSW) {
    fifo_disable_irq();
    m_buffPlCnt = 0;
    m_buffFree = m_buffSize;
    m_pBuffRdI = m_pBuffWrI = m_pBuff;
    fifo_enable_irq();
  }
}

bool NRF24L01::readable(void)
{
  if(m_useIrqPin) {
    return (bool)(m_buffPlCnt > 0);
  } else {
    return (bool)(get_status() & NRF24L01_STATUS_RX_DR);
  }
}

int NRF24L01::rx_data(uint8_t *pipe, uint8_t *data, uint8_t len)
{
  uint8_t status, frlen, plen = 0;

  if (len == 0) return plen;

  if(m_useIrqPin) {
    if(m_buffPlCnt > 0) {
      fifo_disable_irq();
      frlen = *m_pBuffRdI;
      for(int i = 0; i < frlen; i++) {
        switch(i) {
          case 0:                      break; // FRAME Length
          case 1: *pipe = *m_pBuffRdI; break; // PIPE Number
          default:                            // Payload Data
            if(len > (i-2)) { data[i-2] = *m_pBuffRdI; }
            break;
        }
        m_pBuffRdI++;
        if(m_pBuffRdI >= (m_pBuff + m_buffSize)) {
          m_pBuffRdI = m_pBuff;
        }
      }
      m_buffFree += frlen;
      m_buffPlCnt--;
      fifo_enable_irq();
      plen = frlen - 2;
    }
  } else {
    rd_reg(NRF24L01_REG_STATUS, &status);
    if(status & NRF24L01_STATUS_RX_DR) {
      plen = rd_data(pipe, data, len);
    }
    wr_reg(NRF24L01_REG_STATUS, status);
  }

  return plen;
}


int NRF24L01::tx_data(const uint8_t *data, uint16_t len, bool ack)
{
  int ret = 0;
  uint8_t status, n = 200;

  // return if still transmitting
  if(m_opMode == MODE_TX) {
    return -1;
  }

  // Must be in standby if change regs
  set_op_mode(MODE_STANDBY);

  // if enabled, disable the reuse of TX payload
  if (get_fifo_status() & NRF24L01_FIFOST_TX_REUSE) {
    flush_fifo(FIFO_TX);
  }

  // Write data into TX FIFO buffer
  ret = wr_data(data, len, ack);

  // Set TX Mode
  set_op_mode(MODE_TX);

  // Return, if IRQ Pin used
  if(m_useIrqPin) {
    m_txLen = len - ret;
    if(m_txLen) {
      m_txAck    = ack;
      m_pTxIndex = data + ret;
    }
    return 0;
  }

  // Pulling mode (if not used IRQ pin)
  while(n) {
    status = get_status();

    if(status & NRF24L01_STATUS_MAX_RT) {
      flush_fifo(FIFO_TX);
      if(m_txErr < 0xFFFFFFFF) m_txErr++;
      ret = -1;
      break;
    }

    if(ret & NRF24L01_STATUS_TX_DS) {
      if(m_txErr > 0) m_txErr--;
      break;
    }

    wait_us(100);
    n--;
  }

  // clear any pending IRQ flags
  wr_reg(NRF24L01_REG_STATUS, 0x70);

  // Go to standby mode
  m_ce = 0;
  m_opMode = MODE_STANDBY;

  if(n == 0) {
    ret = -1;
    reset();
  }

  return ret;
}

int NRF24L01::tx_data(uint8_t pipe, const uint8_t *data, uint16_t len, bool ack)
{
  uint8_t addr[5], p = pipe > 1 ? 1 : pipe;

  // return if still transmitting
  if(m_opMode == MODE_TX) {
    return -1;
  }

  // Must be in standby if change regs
  set_op_mode(MODE_STANDBY);

  if(pipe == 0) ack = false;

  // Read RX pipe address
  rd_reg(NRF24L01_REG_RX_ADDR_P0 + p, addr, m_conf->addrWidth);
  if(pipe > 1) rd_reg(NRF24L01_REG_RX_ADDR_P0 + pipe, addr);
  // Set TX address value
  wr_reg(NRF24L01_REG_TX_ADDR, addr, m_conf->addrWidth);
  // If HW ACK, then P0 address must be the same as TX address
  if(ack && m_conf->ackMode == ACK_Enabled) {
    wr_reg(NRF24L01_REG_RX_ADDR_P0, addr, m_conf->addrWidth);
  }

  return tx_data(data, len, ack);
}

int NRF24L01::tx_data_to_address(uint64_t address, const uint8_t *data, uint16_t len, bool ack)
{
  // return if still transmitting
  if(m_opMode == MODE_TX) {
    return -1;
  }
  // Must be in standby if update regs
  set_op_mode(MODE_STANDBY);
  // Set the TX address
  set_tx_address(address, ack);
  // Send Data
  return tx_data(data, len, ack);
}

void NRF24L01::reuse_tx_payload(void)
{
  m_cs = 0;
  m_spi.write(NRF24L01_SPI_CMD_REUSE_TX_PL);
  m_cs = 1;
}

int NRF24L01::set_ack_payload(uint8_t pipe, const uint8_t *data, uint8_t len)
{
  uint8_t rdreg;
  // Enable ACK payload feature
  rd_reg(NRF24L01_REG_FEATURE, &rdreg);
  if(!(rdreg & NRF24L01_FEAT_EN_ACKP)) {
    wr_feature_reg(rdreg | NRF24L01_FEAT_EN_ACKP);
  }
  // ...
  if(len > NRF24L01_TX_HWFIFO_SIZE) {
    len = NRF24L01_TX_HWFIFO_SIZE;
  }
  // Write ACK payload data
  wr_reg(NRF24L01_SPI_CMD_WR_ACK_PAYLOAD + pipe, data, len);
  return len;
}

/*********************************************************************************
  * Service Methods
  *********************************************************************************/

bool NRF24L01::is_carrier_detected(void)
{
  uint8_t val;
  rd_reg(NRF24L01_REG_RPD, &val);
  return (val ? true : false);
}

void NRF24L01::start_carrier_wave_test(void)
{
  uint8_t rdreg;

  set_op_mode(MODE_STANDBY);
  flush_fifo(FIFO_TX);
  rd_reg(NRF24L01_REG_RF_SETUP, &rdreg);
  rdreg |= (NRF24L01_RF_SETUP_CONT_WAVE|NRF24L01_RF_SETUP_PLL_LOCK);
  wr_reg(NRF24L01_REG_RF_SETUP, rdreg);
  m_ce = 1;
}

void NRF24L01::stop_carrier_wave_test(void)
{
  uint8_t rdreg;

  rd_reg(NRF24L01_REG_RF_SETUP, &rdreg);
  rdreg &= ~(NRF24L01_RF_SETUP_CONT_WAVE|NRF24L01_RF_SETUP_PLL_LOCK);
  wr_reg(NRF24L01_REG_RF_SETUP, rdreg);
  m_ce = 0;
}


/*********************************************************************************
 * protected
 *********************************************************************************/

void NRF24L01::pin_irq(void)
{
  uint8_t status, rega, regb;

  // Read IRQ flags
  status = get_status();

  // TX Payload NOT Transmitted
  if(status & NRF24L01_STATUS_MAX_RT) {
    if(m_txErr < 0xFFFFFFFF) m_txErr++;
    flush_fifo(FIFO_TX);
    // clear TX Max Resends IRQ flag
    wr_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_MAX_RT);
    status  |= EVENT_TX_DONE;
    m_opMode = MODE_STANDBY;
    m_txLen  = 0;
    m_ce     = 0;
  }

  // TX Payload Transmitted OK
  if(status & NRF24L01_STATUS_TX_DS) {
    if(m_txErr > 0) m_txErr--;
    // clear TX Data Send IRQ flag
    wr_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_TX_DS);
    if(m_txLen) {
      uint16_t txlen;
      // Write payload to TX buffer
      txlen = wr_data(m_pTxIndex, m_txLen, m_txAck);
#if(NRF24L01_TX_CE_OFF)
      m_ce = 1;
      wait_us(NRF24L01_DELAY_THCE_US);
      m_ce = 0;
#endif
      m_pTxIndex += txlen;
      m_txLen    -= txlen;
    } else {
      status     |= EVENT_TX_DONE;
      m_opMode    = MODE_STANDBY;
      m_ce        = 0;
    }
  }

  // Rx Payload Received
  if(status & NRF24L01_STATUS_RX_DR) {
    uint8_t pipe, plen, tmp, regval, n = 0;

    while(n < 3) {
      rd_reg(NRF24L01_REG_FIFO_STATUS, &regval);
      if(regval & NRF24L01_FIFOST_RX_EMPTY) {
        break;
      }
      // ...
      if((pipe = NRF24L01_STATUS_RX_PIPE(get_status())) > 6) {
        flush_fifo(FIFO_RX);
        break;
      }
      // ...
      tmp = 0;
      rd_reg(NRF24L01_REG_FEATURE, &regval);
      if(regval & NRF24L01_FEAT_EN_DPL) {
        rd_reg(NRF24L01_REG_DYNPD, &tmp);
      }
      // ...
      if(tmp & (1 << pipe)) {
        rd_reg(NRF24L01_SPI_CMD_RD_RX_PL_WID, &plen);
      } else {
        rd_reg(NRF24L01_REG_RX_PW_P0 + pipe, &plen);
      }
      // ...
      if((plen > 0) && (plen <= NRF24L01_RX_HWFIFO_SIZE)) {
        // Move RX data from HW FIFO to SW FIFO
        m_cs = 0;
        m_spi.write(NRF24L01_SPI_CMD_RD_RX_PAYLOAD);
        plen += 2;
        if (m_buffFree >= plen) {
          fifo_disable_irq();
          for(int i = 0; i < plen; i++) {
            switch(i) {
              case 0: // FRAME Length
                *m_pBuffWrI = plen;
                break;
              case 1: // PIPE Number and RSSI Value
                *m_pBuffWrI = pipe;
                break;
              default: // Payload Data
                *m_pBuffWrI = m_spi.write(NRF24L01_SPI_CMD_NOP);
                break;
            }
            m_pBuffWrI++;
            if(m_pBuffWrI >= (m_pBuff + m_buffSize)) {
              m_pBuffWrI = m_pBuff;
            }
          }
          m_buffFree -= plen;
          m_buffPlCnt++;
          fifo_enable_irq();
        } else {
          while (plen > 2) {
            m_spi.write(NRF24L01_SPI_CMD_NOP); plen--;
          }
        }
        m_cs = 1;
        n++;
      } else {
        flush_fifo(FIFO_RX);
        break;
      }
      // clear RX Data Ready IRQ flag
      wr_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);
    }
    //m_ce = 1;
  }

  // Call IRQ Callback
  status &= m_cbMask;
  if(m_cbFunc && status) {
    m_cbFunc(this, status);
  }
}

int NRF24L01::rd_reg(uint8_t addr, uint8_t *pVal)
{
  int status;

  m_cs = 0;
  status = m_spi.write(addr);
  *pVal = m_spi.write(NRF24L01_SPI_CMD_NOP);
  m_cs = 1;

  return status;
}

int NRF24L01::rd_reg(uint8_t addr, uint8_t *pBuff, uint8_t len)
{
  int status;

  m_cs = 0;
  status = m_spi.write(addr);
  while (len--) {
    *pBuff = m_spi.write(NRF24L01_SPI_CMD_NOP);
    pBuff++;
  }
  m_cs = 1;

  return status;
}

int NRF24L01::wr_reg(uint8_t addr, uint8_t val)
{
  int status;

  if(addr < NRF24L01_SPI_CMD_WR_REG) {
    addr |= NRF24L01_SPI_CMD_WR_REG;
  }

  m_cs = 0;
  status = m_spi.write(addr);
  m_spi.write(val);
  m_cs = 1;

  return status;
}

int NRF24L01::wr_reg(uint8_t addr, const uint8_t* pBuff, uint8_t len)
{
  int status;

  if(addr < NRF24L01_SPI_CMD_WR_REG) {
    addr |= NRF24L01_SPI_CMD_WR_REG;
  }

  m_cs = 0;
  status = m_spi.write(addr);
  while (len--) {
    m_spi.write(*pBuff);
    pBuff++;
  }
  m_cs = 1;

  return status;
}

int NRF24L01::rd_data(uint8_t *pipe, uint8_t *pBuff, uint8_t len)
{
  uint8_t plen, regval, tmp = 0;

  regval = get_status();
  // ...
  if(!(regval & NRF24L01_STATUS_RX_DR)) {
    return 0;
  }
  // ...
  if((*pipe = NRF24L01_STATUS_RX_PIPE(regval)) > 6) {
    flush_fifo(FIFO_RX);
    return -1;
  }
  // ...
  rd_reg(NRF24L01_REG_FEATURE, &regval);
  if(regval & NRF24L01_FEAT_EN_DPL) {
    rd_reg(NRF24L01_REG_DYNPD, &tmp);
  }

  if(tmp & (1 << *pipe)) {
    rd_reg(NRF24L01_SPI_CMD_RD_RX_PL_WID, &plen);
  } else {
    rd_reg(NRF24L01_REG_RX_PW_P0 + *pipe, &plen);
  }
  // ...
  if(len > plen) len = plen;
  tmp = len;
  // ...
  if((plen > 0) && (plen <= NRF24L01_RX_HWFIFO_SIZE)) {
    m_cs = 0;
    m_spi.write(NRF24L01_SPI_CMD_RD_RX_PAYLOAD);
    while (tmp--) {
      *pBuff = m_spi.write(NRF24L01_SPI_CMD_NOP); pBuff++;
    }
#if(1)
    if(len < plen) {
      tmp = plen - len;
      while (tmp--) { m_spi.write(0); }
    }
#endif
    m_cs = 1;
  } else {
    flush_fifo(FIFO_RX);
    return -1;
  }

  // Clear RX IRQ Flags
  wr_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);

  return len;
}

int NRF24L01::wr_data(const uint8_t *pBuff, uint16_t len, bool ack)
{
  uint8_t cnt, plen = NRF24L01_TX_HWFIFO_SIZE;

  // ...
#if(1)
  if(m_conf->plSize) {
    plen = m_conf->plSize;
  }
#else
  rd_reg(NRF24L01_REG_DYNPD, &plen);
  if((plen & (1 << pipe)) == 0) {
    rd_reg(NRF24L01_REG_RX_PW_P0 + pipe, &plen);
  }
#endif
  // ...
  if(len > plen) len = plen;
  cnt = len;
  // ...
  m_cs = 0;
  m_spi.write(ack ? NRF24L01_SPI_CMD_WR_TX_PAYLOAD :
                    NRF24L01_SPI_CMD_WR_TX_PNOACK);
  while (cnt--) {
    m_spi.write(*pBuff);
    pBuff++;
  }
  // ...
#if(1)
  if(len < plen) {
    cnt = plen - len;
    while (cnt--) { m_spi.write(0); }
  }
#endif
  m_cs = 1;

  return len;
}

uint8_t NRF24L01::get_status()
{
  uint8_t status;

  m_cs = 0;
  status = m_spi.write(NRF24L01_SPI_CMD_NOP);
  m_cs = 1;

  return  status;
}

uint8_t NRF24L01::get_rx_frame_size(uint8_t pipe)
{
  uint8_t fsize;
  rd_reg(NRF24L01_REG_RX_PW_P0 + pipe, &fsize);
  return fsize;
}

uint8_t NRF24L01::get_rx_payload_width(void)
{
  uint8_t width;
  rd_reg(NRF24L01_SPI_CMD_RD_RX_PL_WID, &width);
  return width;
}

bool NRF24L01::wr_feature_reg(uint8_t val) {
  uint8_t rdval;
  val &= NRF24L01_FEAT_EN_ALL;

  // Write into feature register and test if value OK
  wr_reg(NRF24L01_REG_FEATURE, val);
  rd_reg(NRF24L01_REG_FEATURE, &rdval);
  // CAUTION: Some versions of silicon may require
  // activation of feature register
  if (rdval != val)
  {
    // Activate writing into feature registers
    wr_reg(NRF24L01_SPI_CMD_ACTIVATE, 0x73);
    // Write into feature register
    wr_reg(NRF24L01_REG_FEATURE, val);
    rd_reg(NRF24L01_REG_FEATURE, &rdval);
    if (rdval != val)
    {
      return false;
    }
  }

  return true;
}

