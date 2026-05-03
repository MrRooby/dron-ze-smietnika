#include "radio.h"

/**
 * @brief Initialize SPI and GPIO pins for nRF24L01
 */
void NRF24_Init(void) {
    // 1. Configure CSN (Chip Select Not) as Output Push-Pull, High (Idle)
    GPIO_Init(CSN_PORT, CSN_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
    
    // 2. Configure CE (Chip Enable) as Output Push-Pull, Low (Standby)
    GPIO_Init(CE_PORT, CE_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    // 3. Configure SPI Pins (Standard for STM8S Port C)
    // PC5: SCK, PC6: MOSI, PC7: MISO
    // GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); // SCK
    // GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_FAST); // MOSI
    // GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_FL_NO_IT);      // MISO

    // 4. Enable SPI Peripheral Clock
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);

    // 5. Initialize SPI: 1MHz (Prescaler 16) is more stable for testing
    SPI_Init(
        SPI_FIRSTBIT_MSB, 
        SPI_BAUDRATEPRESCALER_16, 
        SPI_MODE_MASTER, 
        SPI_CLOCKPOLARITY_LOW, 
        SPI_CLOCKPHASE_1EDGE, 
        SPI_DATADIRECTION_2LINES_FULLDUPLEX, 
        SPI_NSS_SOFT, 
        0x07
    );

    SPI_Cmd(ENABLE);
}

/**
 * @brief Full-duplex SPI data exchange
 */
uint8_t SPI_Exchange(uint8_t data) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    SPI_SendData(data);
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    return SPI_ReceiveData();
}

/**
 * @brief Write a single byte to an nRF24 register
 */
void NRF24_Write_Reg(uint8_t reg, uint8_t value) {
    GPIO_WriteLow(CSN_PORT, CSN_PIN);
    SPI_Exchange(0x20 | reg); // 0x20 is Write Command prefix
    SPI_Exchange(value);
    GPIO_WriteHigh(CSN_PORT, CSN_PIN);
}

/**
 * @brief Read a single byte from an nRF24 register
 */
uint8_t NRF24_Read_Reg(uint8_t reg) {
    uint8_t res;
    GPIO_WriteLow(CSN_PORT, CSN_PIN);
    SPI_Exchange(reg);        // Read command is the register address
    res = SPI_Exchange(0xFF); // Send NOP to clock out data
    GPIO_WriteHigh(CSN_PORT, CSN_PIN);
    return res;
}

/**
 * @brief Full setup sequence to put nRF24 into Receiver mode
 */
void NRF24_Setup(void) {
    NRF24_Init();
    
    // Ensure CE is Low during configuration
    GPIO_WriteLow(CE_PORT, CE_PIN);

    // Set RF Channel (e.g., 2.476 GHz)
    NRF24_Write_Reg(NRF_RF_CH, 76);

    // Set Data Rate to 2Mbps and Power to 0dBm (Max)
    NRF24_Write_Reg(NRF_RF_SETUP, 0x0E);

    // Disable Auto-ACK for initial simplified testing
    NRF24_Write_Reg(NRF_EN_AA, 0x00);

    // Power Up and set as Receiver (PRX)
    // Bit 1: PWR_UP, Bit 0: PRIM_RX
    NRF24_Write_Reg(NRF_CONFIG, 0x03); 

    // Transition delay: Power Down -> Standby (max 1.5ms - 5ms)
    // If you have a delay function, use it here: Delay(5);

    // Pull CE High to activate the RX radio
    GPIO_WriteHigh(CE_PORT, CE_PIN);
}

/**
 * @brief Reads the STATUS register using the NOP command
 */
uint8_t NRF24_Get_Status(void) {
    uint8_t status;
    GPIO_WriteLow(CSN_PORT, CSN_PIN);
    status = SPI_Exchange(0xFF); // 0xFF is the NRF24 NOP command
    GPIO_WriteHigh(CSN_PORT, CSN_PIN);
    return status;
}

/**
 * @brief Reads the 5-byte TX address to verify SPI communication
 */
void NRF24_WhoAmI(void) {
    uint8_t addr[5];
    
    // 0x10 is the TX_ADDR register
    GPIO_WriteLow(CSN_PORT, CSN_PIN);
    SPI_Exchange(0x10); 
    
    for(uint8_t i = 0; i < 5; i++) {
        addr[i] = SPI_Exchange(0xFF);
    }
    GPIO_WriteHigh(CSN_PORT, CSN_PIN);

    printf("WhoAmI (TX_ADDR): %02X:%02X:%02X:%02X:%02X\n", 
            addr[0], addr[1], addr[2], addr[3], addr[4]);
    // Expect: E7:E7:E7:E7:E7 (Factory Default)
}

/**
 * @brief Legacy test function (redundant but kept for your main.c compatibility)
 */
void NRF24_Test_Connection(void) {
    NRF24_WhoAmI();
}
