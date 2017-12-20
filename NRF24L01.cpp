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

#if(RX_FIFO_IRQ)
#define fifo_enable_irq()  __enable_irq()
#define fifo_disable_irq() __disable_irq()
#else
#define fifo_enable_irq()
#define fifo_disable_irq()
#endif

#define RX_FIFO_SIZE                       (34*3)

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
#define NRF24L01_STATUS_RX_PIPE(reg)       ((reg & NRF24L01_STATUS_RX_PIPE_MASK) >> 1)
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

/* Default configuration */
static const NRF24L01::conf_t defConf = {
  /* .crc        = */ NRF24L01::CRC_16bit,
  /* .defMode    = */ NRF24L01::MODE_STANDBY,

  /* .ackMode    = */ NRF24L01::ACK_HW,
  /* .ackDelay   = */ 500,   /* Max: 4000 us */
  /* .ackCount   = */ 3,     /* Max: 15 */

  /* .txPwr      = */ NRF24L01::TXPWR_0dBm,
  /* .dataRate   = */ NRF24L01::DRATE_2MBps,
  /* .channel    = */ 1,     /* Max: 127 */

  /* .addrWidth  = */ 5,     /* Min: 3; Max: 5 */
  /* .onPipes    = */ 6,     /* Min: 1; Max: 6 */
  /* .rxAddrP0   = */ 0xE7E7E7E7E7,
  /* .rxAddrP1   = */ 0xC2C2C2C2C2,
  /* .rxAddrP2   = */ 0xC3,
  /* .rxAddrP3   = */ 0xC4,
  /* .rxAddrP4   = */ 0xC5,
  /* .rxAddrP5   = */ 0xC6,

  /* .plSize     = */ 32     /* Min: 1; Max: 32 */
};

/* RX Buffer Declaration */
static uint8_t rxBuffer[RX_FIFO_SIZE];

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
  // RX FIFO Buffer initialization
  m_rxCount = 0;
  m_rxItmSize = 32 + 2;
  m_rxSize = 3;
  m_pHead = rxBuffer;
  m_pTail = rxBuffer + ((32 + 2) * 3);
  m_pRdIndex = m_pHead;
  m_pWrIndex = m_pHead;
  // ...
  m_autoAck = ACK_DISABLED;
  m_actualMode = MODE_POWER_DOWN;
  m_defaultMode = MODE_STANDBY;
  m_usePwrPin = (pwr != NC) ? true : false;
  m_useIrqPin = (irq != NC) ? true : false;

  // Configure communication bus
  if(m_usePwrPin) m_pwr = 1; // ...
  m_ce = 0;                  // Disable NRF24L01 TX/RX
  m_cs = 1;                  // Release SPI CS
  m_spi.frequency(8000000);  // Set SPI speed (max. 8MHz for NRF24L01)
  m_spi.format(8,0);         // Set SPI format (8-bit, ClockPhase = 0, ClockPolarity = 0)
}


bool NRF24L01::Reset(void)
{
  uint8_t regval, fval = 0;

  m_rxErr = 0;
  m_txErr = 0;
  m_autoAck = m_conf->ackMode;
  m_defaultMode = m_conf->defMode;

  if(m_usePwrPin) {
    m_pwr = 1;
    wait_ms(25);
    m_pwr = 0;
    wait_ms(100);
  }

  SetMode(MODE_POWER_DOWN);

  // Configure CRC, IRQ and power mode
  regval = m_conf->crc;
  if(!m_useIrqPin) {
    regval |= (NRF24L01_CONFIG_MAX_RT_MASK |
               NRF24L01_CONFIG_RX_DR_MASK  |
               NRF24L01_CONFIG_TX_DS_MASK);
  }
  wr_nrf(NRF24L01_REG_CONFIG, regval);

  // Set address width in range (3 - 5)
  regval = 0;
  if (m_conf->addrWidth >= 2) { regval = (m_conf->addrWidth - 2) & 0x03; }
  wr_nrf(NRF24L01_REG_SETUP_AW, regval);

  // Set automatic retransmission delay and count
  SetAutoRetrans(m_conf->ackDelay, m_conf->ackCount);

  // Set Channel
  wr_nrf(NRF24L01_REG_RF_CH, m_conf->channel);

  // Set data rate and TX power
  regval  = m_conf->dataRate;
  regval |= m_conf->txPwr;
  wr_nrf(NRF24L01_REG_RF_SETUP, regval);

  // Get mask from on pipes
  regval  = ((1 << m_conf->onPipes) - 1) & 0x3F;
  regval |= 0x01; // NOTE: PIPE0 must be enabled anyway
  // Enable Rx pipes
  wr_nrf(NRF24L01_REG_EN_RXADDR, regval);

  if(m_autoAck == ACK_HW) {
    // Enable auto ACK
    fval |= (NRF24L01_FEAT_EN_NOACK|NRF24L01_FEAT_EN_ACKP);
    wr_nrf(NRF24L01_REG_EN_AA, regval);
  } else {
    wr_nrf(NRF24L01_REG_EN_AA, 0);
  }

  if((m_conf->plSize == 0) || (m_autoAck == ACK_HW)) {
    // Enable dynamic payload
    fval |= NRF24L01_FEAT_EN_DPL;
    wr_nrf(NRF24L01_REG_DYNPD, regval);
  } else {
    wr_nrf(NRF24L01_REG_DYNPD, 0);
  }

  // Enable feature flags
  if(!wr_feature_reg(fval)) {
    return false;
  }

  // Set Default payload width
  for(int i = 0; i < 6; i++) {
    if(regval & (1 << i)) {
      wr_nrf(NRF24L01_REG_RX_PW_P0 + i, m_conf->plSize);
    }
  }

  // Set NRF24L01 TX address same as PIPE0
  SetTxAddr(m_conf->rxAddrP0);

  // Set NRF24L01 RX pipes address
  SetRxAddr(0, m_conf->rxAddrP0);
  SetRxAddr(1, m_conf->rxAddrP1);
  SetRxAddr(2, m_conf->rxAddrP2);
  SetRxAddr(3, m_conf->rxAddrP3);
  SetRxAddr(4, m_conf->rxAddrP4);
  SetRxAddr(5, m_conf->rxAddrP5);

  // Clear any pending interrupts
  wr_nrf(NRF24L01_REG_STATUS, NRF24L01_STATUS_MAX_RT |
                              NRF24L01_STATUS_TX_DS  |
                              NRF24L01_STATUS_RX_DR);
  // Reset FIFO buffers
  FlushFifo();
  m_rxCount = 0;
  m_pRdIndex = m_pWrIndex = m_pHead;

  // Set Default Mode
  SetMode();

  return true;
}

bool NRF24L01::Open(const conf_t *conf)
{
  m_conf = (conf == NULL) ? &defConf : conf;

  // Wait 100ms for NRF24L01 Power-on reset
  wait_ms(100);

  if(!Reset())
    return false;

  if(m_useIrqPin) {
    m_irq.fall(this, &NRF24L01::pin_irq);
  }

  return true;
}

void NRF24L01::Close(void)
{
  SetMode(MODE_POWER_DOWN);
}

bool NRF24L01::SetMode(mode_t mode)
{
  uint8_t  regval;
  uint32_t wait_time = NRF24L01_DELAY_THCE_US;

  if(mode == 0xFF) mode = m_defaultMode;
  if(m_actualMode == mode) return true;

  m_ce = 0;   // CE must be 0
  rd_nrf(NRF24L01_REG_CONFIG, &regval);

  switch(mode)
  {
    case MODE_POWER_DOWN:
      regval &= ~NRF24L01_CONFIG_PWR_UP;
      break;

    case MODE_STANDBY:
      regval |= NRF24L01_CONFIG_PWR_UP;
      if (m_actualMode == MODE_POWER_DOWN) {
        wait_time = NRF24L01_DELAY_TPD2STBY_US;
      }
      break;

    case MODE_RX:
      if (m_actualMode == MODE_POWER_DOWN) {
        regval |= NRF24L01_CONFIG_PWR_UP;
        wr_nrf(NRF24L01_REG_CONFIG, regval);
        wait_us(NRF24L01_DELAY_TPD2STBY_US);
      }
      /* set to PRX */
      regval |= NRF24L01_CONFIG_PRIM_RX;
      wait_time = NRF24L01_DELAY_TSTBY2A_US;
      break;

    case MODE_TX:
      if (m_actualMode == MODE_POWER_DOWN) {
        regval |= NRF24L01_CONFIG_PWR_UP;
        wr_nrf(NRF24L01_REG_CONFIG, regval);
        wait_us(NRF24L01_DELAY_TPD2STBY_US);
      }
      /* set to PTX */
      regval &= ~NRF24L01_CONFIG_PRIM_RX;
      wait_time = NRF24L01_DELAY_TSTBY2A_US;
      break;
  }

  // Set Mode
  wr_nrf(NRF24L01_REG_CONFIG, regval);
  if(mode == MODE_RX || mode == MODE_TX) {
    m_ce = 1; // Enable RX/TX mode
  }
  if(mode == MODE_TX) {
    wait_us(NRF24L01_DELAY_THCE_US);
    m_ce = 0;
  }

  // Wait required time
  wait_us(wait_time);
  m_actualMode = mode;

  return true;
}

bool NRF24L01::ConfRxPipe(uint8_t pipe, bool enable, bool autoAck, uint8_t plSize)
{
  uint8_t regval, pmask = ((1 << pipe) & 0x3F);

  // Enable pipe
  rd_nrf(NRF24L01_REG_EN_RXADDR, &regval);
  if(enable) { regval |=  pmask; }
  else       { regval &= ~pmask; }
  wr_nrf(NRF24L01_REG_EN_RXADDR, regval);

  // Return if just disable
  if(!enable) return true;

  // Set Frame size
  if(plSize == 0xFF) plSize = m_conf->plSize;
  wr_nrf(NRF24L01_REG_RX_PW_P0 + pipe, plSize & 0x3F);

  // Set auto ACK
  rd_nrf(NRF24L01_REG_EN_AA, &regval);
  if(autoAck) { regval |=  pmask; }
  else        { regval &= ~pmask; }
  wr_nrf(NRF24L01_REG_EN_AA, regval);

  // Set dynamic payload
  rd_nrf(NRF24L01_REG_DYNPD, &regval);
  if(plSize == 0) { regval |=  pmask; }
  else               { regval &= ~pmask; }
  wr_nrf(NRF24L01_REG_DYNPD, regval);

  if(autoAck || (plSize == 0)) {
    // enable dynamic payload and TX no ACK features
    pmask = NRF24L01_FEAT_EN_DPL | NRF24L01_FEAT_EN_NOACK;
    // First check if not enabled
    rd_nrf(NRF24L01_REG_FEATURE, &regval);
    if((regval & pmask) != pmask) {
      wr_feature_reg(regval | pmask);
    }
  }

  return true;
}

int NRF24L01::GetEnabledPipes(void)
{
  uint8_t pipescfg;
  rd_nrf(NRF24L01_REG_EN_RXADDR, &pipescfg);
  return pipescfg;
}

void NRF24L01::SetRxAddr(uint8_t pipe, uint64_t address)
{
  uint8_t addr_arr[5], len = pipe > 1 ? 1 : 5;

  for(int i = 0; i < len; i ++) {
    addr_arr[i] = (uint8_t) (address >> (i * 8));
  }
  // Set RX address value
  wr_nrf(NRF24L01_REG_RX_ADDR_P0 + pipe, addr_arr, len);
}

void NRF24L01::SetTxAddr(uint64_t address)
{
  uint8_t addr[5];

  for(int i = 0; i < m_conf->addrWidth; i++) {
    addr[i] = (uint8_t) (address >> (i * 8));
  }
  // Set TX address value
  wr_nrf(NRF24L01_REG_TX_ADDR, addr, m_conf->addrWidth);
  // If HW ACK, then P0 address must be the same as TX address
  if(m_conf->ackMode == ACK_HW) {
    wr_nrf(NRF24L01_REG_RX_ADDR_P0, addr, m_conf->addrWidth);
  }
}

void NRF24L01::SetTxAddrToPipe(uint8_t pipe)
{
  uint8_t addr[5], p = pipe > 1 ? 1 : pipe;

  // Read RX pipe address
  rd_nrf(NRF24L01_REG_RX_ADDR_P0 + p, addr, m_conf->addrWidth);
  if(pipe > 1) rd_nrf(NRF24L01_REG_RX_ADDR_P0 + pipe, addr);
  // Set TX address value
  wr_nrf(NRF24L01_REG_TX_ADDR, addr, m_conf->addrWidth);
  // If HW ACK, then P0 address must be the same as TX address
  if(pipe != 0 && m_conf->ackMode == ACK_HW) {
    wr_nrf(NRF24L01_REG_RX_ADDR_P0, addr, m_conf->addrWidth);
  }
}

void NRF24L01::SetChannel(uint8_t channel)
{
  if(channel > 127) channel = 127;
  wr_nrf(NRF24L01_REG_RF_CH, channel);
}

void NRF24L01::SetTxPower(txpwr_t power)
{
  uint8_t temp;
  rd_nrf(NRF24L01_REG_RF_SETUP, &temp);
  temp &= ~NRF24L01_RF_SETUP_TX_PWR_MASK;
  wr_nrf(NRF24L01_REG_RF_SETUP, temp | power);
}

void NRF24L01::SetDataRate(drate_t rate)
{
  uint8_t temp;
  rd_nrf(NRF24L01_REG_RF_SETUP, &temp);
  temp &= ~NRF24L01_RF_SETUP_DR_MASK;
  wr_nrf(NRF24L01_REG_RF_SETUP, temp | rate);
}

void NRF24L01::SetAutoRetrans(uint16_t delay_us, uint8_t count)
{
  uint8_t tmp = (delay_us / 250) & 0x0F;

  if(tmp > 0) tmp -= 1;
  wr_nrf(NRF24L01_REG_SETUP_RETR, (tmp << 4) | (count & 0x0F));
}

uint8_t NRF24L01::GetFifoStatus(void)
{
  uint8_t status;
  rd_nrf(NRF24L01_REG_FIFO_STATUS, &status);
  return (status);
}

void NRF24L01::FlushFifo(fifo_t val)
{
  if(val & FIFO_RX) {
    wr_nrf(NRF24L01_SPI_CMD_FLUSH_RX, 0);
  }

  if(val & FIFO_TX) {
    wr_nrf(NRF24L01_SPI_CMD_FLUSH_TX, 0);
  }
}


bool NRF24L01::IsReadable(void)
{
  uint8_t status;
  bool ret;

  if(m_useIrqPin) {
    ret = (m_rxCount > 0);
  } else {
    rd_nrf(NRF24L01_REG_STATUS, &status);
    ret = (status & NRF24L01_STATUS_RX_DR);
  }

  return ret;
}

int NRF24L01::RxData(uint8_t *pipe, uint8_t *data, uint8_t len)
{
  int rxlen = 0;
  uint8_t status;

  if (len == 0)
    return rxlen;

  if(m_useIrqPin) {
    if(m_rxCount > 0) {
      fifo_disable_irq();
      *pipe = m_pRdIndex[0];
      rxlen = m_pRdIndex[1];
      memcpy(data, m_pRdIndex + 2, rxlen); // get the item from buffer
      m_pRdIndex += m_rxItmSize;
      if (m_pRdIndex >= m_pTail) {
        m_pRdIndex = m_pHead;
      }
      m_rxCount--;
      fifo_enable_irq();
    }
  } else {
    if(get_status() & NRF24L01_STATUS_RX_DR)
    {
      status = rd_nrf(NRF24L01_SPI_CMD_RD_RX_PAYLOAD, data, len);
      rxlen = len;
      // Reset IRQ flags
      status &= NRF24L01_STATUS_RX_DR;
      wr_nrf(NRF24L01_REG_STATUS, status);
    }
  }

  return (rxlen);
}


int NRF24L01::TxData(const uint8_t *data, uint8_t len, bool ack)
{
  int ret = -1, n = 10, status;

  while(n--) {
    status = GetFifoStatus();
    if(status & NRF24L01_FIFOST_TX_FULL) {
      wait_us(500);
    } else {
      n = 0;
    }
  }

  SetMode(MODE_STANDBY);

  if (status & (NRF24L01_FIFOST_TX_REUSE|NRF24L01_FIFOST_TX_FULL)) {
    FlushFifo(FIFO_TX); // Disable Reuse of TX data feature
  }

  // Write payload
  if(!ack && (m_autoAck == ACK_HW)) {
    wr_nrf(NRF24L01_SPI_CMD_WR_TX_PNOACK, data, len);
  } else {
    wr_nrf(NRF24L01_SPI_CMD_WR_TX_PAYLOAD, data, len);
  }

  // Clear any pending interrupts
  wr_nrf(NRF24L01_REG_STATUS, NRF24L01_STATUS_MAX_RT |
                              NRF24L01_STATUS_TX_DS  |
                              NRF24L01_STATUS_RX_DR);
  FlushFifo(FIFO_RX);

  // Set TX Mode
  SetMode(MODE_TX);

  if(m_useIrqPin) {
    return len;
  }

  // Pulling mode (if not used IRQ pin)
  n = 50;
  while(n--) {
    ret = get_status();

    if(ret & NRF24L01_STATUS_MAX_RT) {
      FlushFifo(FIFO_TX);
      m_txErr++;
      ret = -1;
      break;
    }

    if(ret & NRF24L01_STATUS_TX_DS) {
      ret = len;
      break;
    }

    wait_us(100);
  }

  // Set Default Mode
  SetMode();

  return ret;
}

void NRF24L01::ReuseTxPayload(void)
{
  wr_nrf(NRF24L01_SPI_CMD_REUSE_TX_PL, 0);
}

void NRF24L01::SetAckPayload(uint8_t pipe, const uint8_t *data, uint8_t len)
{
  uint8_t rdreg;
  // Enable ACK payload feature
  rd_nrf(NRF24L01_REG_FEATURE, &rdreg);
  if(!(rdreg & NRF24L01_FEAT_EN_ACKP)) {
    wr_feature_reg(rdreg | NRF24L01_FEAT_EN_ACKP);
  }
  // Write ACK payload data
  wr_nrf(NRF24L01_SPI_CMD_WR_ACK_PAYLOAD + pipe, data, len);
}

/*********************************************************************************
  * Service Methods
  *********************************************************************************/

bool NRF24L01::IsCarrierDetected(void)
{
  uint8_t val;
  rd_nrf(NRF24L01_REG_RPD, &val);
  return (val ? true : false);
}

void NRF24L01::StartCarrierWaveTest(void)
{
  uint8_t rdreg;

  SetMode(MODE_STANDBY);
  FlushFifo(FIFO_TX);
  rd_nrf(NRF24L01_REG_RF_SETUP, &rdreg);
  rdreg |= (NRF24L01_RF_SETUP_CONT_WAVE|NRF24L01_RF_SETUP_PLL_LOCK);
  wr_nrf(NRF24L01_REG_RF_SETUP, rdreg);
  m_ce = 1;
}

void NRF24L01::StopCarrierWaveTest(void)
{
  uint8_t rdreg;

  rd_nrf(NRF24L01_REG_RF_SETUP, &rdreg);
  rdreg &= ~(NRF24L01_RF_SETUP_CONT_WAVE|NRF24L01_RF_SETUP_PLL_LOCK);
  wr_nrf(NRF24L01_REG_RF_SETUP, rdreg);
  m_ce = 0;
}


/*********************************************************************************
 * protected
 *********************************************************************************/

void NRF24L01::pin_irq(void)
{
  uint8_t status, rega, regb;
  mode_t mode = m_defaultMode;

  // Switch into Standby Mode
  m_actualMode = MODE_STANDBY;
  m_ce = 0;

  // Read and clear any pending IRQ flags
  status = get_status();

  // TX Payload NOT Transmitted
  if(status & NRF24L01_STATUS_MAX_RT) {
    FlushFifo(FIFO_TX);
    m_txErr++;
  }

  // TX Payload Transmitted OK
  if(status & NRF24L01_STATUS_TX_DS) {
    rd_nrf(NRF24L01_REG_FIFO_STATUS, &regb);
    if((regb & NRF24L01_FIFOST_TX_EMPTY) != NRF24L01_FIFOST_TX_EMPTY) {
      mode == MODE_TX;
    }
  }

  // Rx Payload Received
  if(status & NRF24L01_STATUS_RX_DR) {
    bool dataErr = true;
    uint8_t pipe = NRF24L01_STATUS_RX_PIPE(rega);
    rd_nrf(NRF24L01_REG_DYNPD, &rega);
    rd_nrf(NRF24L01_REG_FEATURE, &regb);

    if(pipe < 6) {
      m_pWrIndex[0] = pipe;
      if((rega & (1 << pipe)) &&
         (regb & NRF24L01_FEAT_EN_DPL)) {
        m_pWrIndex[1] = get_rx_payload_width();
      } else {
        m_pWrIndex[1] = get_rx_frame_size(pipe);
      }

      if(m_pWrIndex[1] <= NRF24L01_RX_FIFO_SIZE) {
        fifo_disable_irq();
        rd_nrf(NRF24L01_SPI_CMD_RD_RX_PAYLOAD, m_pWrIndex+2, m_pWrIndex[1]);
        m_pWrIndex += m_rxItmSize;
        if (m_pWrIndex >= m_pTail) {
          m_pWrIndex = m_pHead;
        }
        if (m_rxCount < m_rxSize) {
          m_rxCount++;
        }
        dataErr = false;
        fifo_enable_irq();
      }
    }

    if(dataErr) {
      FlushFifo(FIFO_RX); // Flush RX FIFO if error payload
      m_rxErr++;
    }
  }

  if((m_cbFunc != NULL) && (status & m_cbMask)) {
    m_cbFunc(this, status);
  }

  // Set operation mode
  SetMode(mode);
}

int NRF24L01::rd_nrf(uint8_t addr, uint8_t *pVal)
{
  int status;

  m_cs = 0;
  status = m_spi.write(addr);
  *pVal = m_spi.write(NRF24L01_SPI_CMD_NOP);
  m_cs = 1;

  return status;
}

int NRF24L01::rd_nrf(uint8_t addr, uint8_t *pBuff, uint8_t len)
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

int NRF24L01::wr_nrf(uint8_t addr, uint8_t val)
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

int NRF24L01::wr_nrf(uint8_t addr, const uint8_t* pBuff, uint8_t len)
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

int NRF24L01::get_status(void)
{
  // Read status and clear IRQ flags
  return wr_nrf(NRF24L01_REG_STATUS, 0x70);
}

uint8_t NRF24L01::get_rx_frame_size(uint8_t pipe)
{
  uint8_t fsize;
  rd_nrf(NRF24L01_REG_RX_PW_P0 + pipe, &fsize);
  return fsize;
}

uint8_t NRF24L01::get_rx_payload_width(void)
{
  uint8_t width;
  rd_nrf(NRF24L01_SPI_CMD_RD_RX_PL_WID, &width);
  return width;
}

bool NRF24L01::wr_feature_reg(uint8_t val) {
  uint8_t rdval;
  val &= NRF24L01_FEAT_EN_ALL;

  // Write into feature register and test if value OK
  wr_nrf(NRF24L01_REG_FEATURE, val);
  rd_nrf(NRF24L01_REG_FEATURE, &rdval);
  // CAUTION: Some versions of silicon may require
  // activation of feature register
  if (rdval != val)
  {
    // Activate writing into feature registers
    wr_nrf(NRF24L01_SPI_CMD_ACTIVATE, 0x73);
    // Write into feature register
    wr_nrf(NRF24L01_REG_FEATURE, val);
    rd_nrf(NRF24L01_REG_FEATURE, &rdval);
    if (rdval != val)
    {
      return false;
    }
  }

  return true;
}

