#include "MFRC522.h"
#include <unistd.h>

#define RSTPIN RPI_V2_GPIO_P1_22

int main(){

  if (!bcm2835_init()) {
    printf("Failed to initialize. This tool needs root access, use sudo.\n");
  }
  bcm2835_gpio_fsel(RSTPIN, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RSTPIN, LOW);

  // Set SPI bus to work with MFRC522 chip.
  setSPIConfig();

  PCD_Init();

  while(1){
    // Look for a card
    if(!PICC_IsNewCardPresent())
      continue;

    if(!PICC_ReadCardSerial())
      continue;

    // Print UID
  for (byte i = 0; i < uid.size; i++) {
    if(uid.uidByte[i] < 0x10)
	printf(" 0");
    else
	printf(" ");
    printf("%X", uid.uidByte[i]);
  }
  printf("\n");
    delay(1000);
  }
  return 0;
}

void setSPIConfig() {

  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // ~ 4 MHz
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

} // End setSPIConfig()

void PCD_Init() {
  if (bcm2835_gpio_lev(RSTPIN) == LOW) {	//The MFRC522 chip is in power down mode.
    bcm2835_gpio_write(RSTPIN, HIGH);		// Exit power down mode. This triggers a hard reset.
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    delay(50);
  }
  else { // Perform a soft reset
    PCD_Reset();
  }

  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister2A(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister2A(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
  PCD_WriteRegister2A(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister2A(TReloadRegL, 0xE8);

  PCD_WriteRegister2A(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister2A(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

int PICC_IsNewCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  byte result = PICC_RequestA(bufferATQA, &bufferSize);
  return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

int PICC_ReadCardSerial() {
  byte result = PICC_Select(&uid, 0);
  return (result == STATUS_OK);
} // End PICC_ReadCardSerial()
