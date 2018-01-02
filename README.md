# mbed-os driver for NRF24L01+ (2.4GHz transceiver)

The Nordic [nRF24L01+](http://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P) is a highly integrated, ultra low power (ULP) 2Mbps RF transceiver IC. 

# usage

```C
#include "mbed.h"
#include "NRF24L01.h"

/* MCU UART (Debug Console) Pins */
#define UART_TXD_PIN   PTA2
#define UART_RXD_PIN   PTA1

/* MCU RF Connection Pins */
#define RF_MISO_PIN    PTD7
#define RF_MOSI_PIN    PTD6
#define RF_SCLK_PIN    PTD5
#define RF_CS_PIN      PTD4
#define RF_IRQ_PIN     PTC7
#define RF_CE_PIN      PTC6
#define RF_PWR_PIN     NC

/* NRF24L01 configuration */
static NRF24L01::conf_t rfconf = {
  /* .crc          = */ NRF24L01::CRC_16bit,
  /* .txPwr        = */ NRF24L01::TXPWR_Plus_0dBm,
  /* .dataRate     = */ NRF24L01::DRATE_2MBps,
  /* .channel      = */ 10,    /* Max: 127 */
  /* .plSize       = */ 0,     /* Min: 1; Max: 32; or 0 for Dynamic Payload */

  /* .ackMode      = */ NRF24L01::ACK_Enabled,
  /* .ackDelay     = */ 750,   /* Max: 4000 us */
  /* .ackCount     = */ 5,     /* Max: 15 */

  /* .addrWidth    = */ 5,     /* Min: 3; Max: 5 */
  /* .onPipes      = */ 6,     /* Min: 1; Max: 6 */
  /* .rxAddrP0     = */ 0xE7E7E7E7E7, // Used as Broadcast Address
  /* .rxAddrP1     = */ 0xC2C2C2C201,
  /* .rxAddrP2     = */ 0x02,
  /* .rxAddrP3     = */ 0x03,
  /* .rxAddrP4     = */ 0x04,
  /* .rxAddrP5     = */ 0x05
};

RawSerial  uart(UART_TXD_PIN, UART_RXD_PIN);
NRF24L01   rfm(RF_MOSI_PIN, RF_MISO_PIN, RF_SCLK_PIN, RF_CS_PIN, RF_CE_PIN, RF_PWR_PIN, RF_IRQ_PIN);

/* The definition of NRF24L01 IRQ callback function */
void rfm_irq_cb(NRF24L01 *obj, uint8_t event)
{
  // TX: Data NOT Transmitted (TX_Error)
  if(event & NRF24L01::EVENT_MAX_RT) {
    // TODO: Add user code here
  }
  // TX: Data Transmitted OK
  if(event & NRF24L01::EVENT_TX_DS)  {
    // TODO: Add user code here
  }
  // RX: Data Received
  if(event & NRF24L01::EVENT_RX_DR)  {
    // TODO: Add user code here
  }
  // TX: Transmit of buffer Done
  if(event & NRF24L01::EVENT_TX_DONE) {
    rfm.set_op_mode(NRF24L01::MODE_RX);
  }
}

int main()
{
  uint8_t txBuff[32], rxBuff[32], pipe, len;

  /* Initialize RF Module */
  if(!rfm.open(&rfconf)) {
    uart.puts("NRF24L01 init. error !\n");
  }

  /* Attach callback function */
  //rfm.attach_cb(rfm_irq_cb);

  while(1)
  {
    if(uart.readable()) {
      rfm.tx_data(1, txBuff, sizeof(txBuff));
    }

    if(rfm.readable()) {
      len = rfm.rx_data(&pipe, rxBuff, sizeof(rxBuff));
    }
  }
}
```


