#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define PIN_ATT 16
#define PIN_ACK 11

int spi_fd;
struct sigaction act;
static char *spiDevice = "/dev/spidev1.0";
static uint32_t spiSpeed = 250000;
static uint16_t spiDelay = 8;

int spiOpen(char* dev)
{
  if((spi_fd = open(dev, O_RDWR)) < 0)
  {
    printf("error opening %s\n",dev);
    return -1;
  }
  return 0;
}

void readWriteByte(uint8_t txData, uint8_t *rxData)
{
  uint8_t spiBufTx [1];
  uint8_t spiBufRx [1];
  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof(spi));
  spiBufTx [0] = txData;
  spi.tx_buf =(unsigned long)spiBufTx;
  spi.rx_buf =(unsigned long)spiBufRx;
  spi.len = 1;
  spi.delay_usecs = spiDelay;
  spi.speed_hz = spiSpeed;
  spi.bits_per_word = 8;
  spi.cs_change     = 0;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
  if(rxData) *rxData = spiBufRx[0];
}

uint8_t transferBytes(uint8_t isMemoryCard, uint8_t *txBytes, uint8_t *rxBytes,                                                                                                                                                                                                                                              uint8_t *len)
{
  digitalWrite(PIN_ATT, LOW);
  uint8_t rxTemp = 0;
  readWriteByte(isMemoryCard ? 0x81 : 0x01, &rxTemp);
  uint8_t rxLen = 0;
  uint8_t lenTmp = *len;
  for(; rxLen < lenTmp; rxLen++) {
    if(rxLen < lenTmp) {
      readWriteByte(txBytes[rxLen], rxBytes + rxLen);
    }
    else {
      readWriteByte(0, rxBytes + rxLen);
    }
  }
  if(len) *len = rxLen;
  digitalWrite(PIN_ATT, HIGH);
  return TRUE;
}

void sig_handler(int signum, siginfo_t *info, void *ptr)
{
  printf("Received signal %d\n", signum);
  printf("Signal originated from process %lu\n",
    (unsigned long)info->si_pid);
  close(spi_fd);
  exit(0);
}

uint8_t readMsg[] = { 0x42 };
const uint8_t readMsgLen = 1;

void onAckReceived()
{
}

int main()
{
  wiringPiSetup();
  pinMode(PIN_ATT, OUTPUT);
  pinMode(PIN_ACK, INPUT);
  wiringPiISR(PIN_ACK, INT_EDGE_RISING, onAckReceived);
  memset(&act, 0, sizeof(act));

  act.sa_sigaction = sig_handler;
  act.sa_flags = SA_SIGINFO;
  sigaction(SIGTERM, &act, NULL);

  spiOpen(spiDevice);
  uint8_t rxBytes[255];
  memset(rxBytes, 0, 255);
  uint8_t lenTemp = 0;
  while (1)
  {
    lenTemp = readMsgLen;
    transferBytes(FALSE, readMsg, rxBytes, &lenTemp);
    sleep(1);
  }

  return 0;
}
