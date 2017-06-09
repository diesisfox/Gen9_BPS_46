/*
 * mcp3909.h
 *
 *  Created on: Jan 2, 2017
 *      Author: frank
 */

#ifndef MCP3909_H_
#define MCP3909_H_

#include "stm32f4xx_hal.h"

// NOTE: the Tx Buffer should always have the first byte empty for the CONTROL BYTE!
// Tx and Rx buffers must both be declared and allocated globally before the functions

//#define SPI_TIMEOUT		1000 	// SPI Synchronous blocking timeout - milliseconds
#define T_POR			50		// Power on reset wait time

// Register Addresses
#define CHANNEL_0   (0x0U)
#define CHANNEL_1   (0x1U)
#define CHANNEL_2   (0x2U)
#define CHANNEL_3   (0x3U)
#define CHANNEL_4   (0x4U)
#define CHANNEL_5   (0x5U)
#define MOD         (0x6U)
#define PHASE       (0x7U)
#define GAIN        (0x8U)
#define STATUS      (0x9U)
#define CONFIG      (0xAU)

// CHANNEL REGISTER START -----------------------------
// Channel data length
#define MAX_CHANNEL_NUM   (6U)
#define MAX_CHN_SET_NUM   (3U)
#define REGS_NUM		  (11U)
//#define REG_LEN           (3U)
#define CTRL_LEN		  (1U)
#define CHN_GROUP_LEN     (2U) * REG_LEN
#define MOD_GROUP_LEN     (3U) * REG_LEN
#define STATUS_GROUP_LEN  (2U) * REG_LEN
#define CHN_TYPE_LEN      REG_LEN * MAX_CHANNEL_NUM
#define CONFIG_TYPE_LEN   MOD_GROUP_LEN + STATUS_GROUP_LEN
#define TOTAL_LEN         (11U) * REG_LEN
// CHANNEL REGISTER END   -----------------------------

// GAIN REGISTER START    -----------------------------
// PGA settings
#define PGA_1       (0U)
#define PGA_2       (1U)
#define PGA_4       (2U)
#define PGA_8       (3U)
#define PGA_16      (4U)
#define PGA_32      (5U)

#define BOOST_OFFSET_ODD     (1U)
#define BOOST_OFFSET_EVEN    (3U)
#define PGA_BOOST_LEN        (4U)

// Boost settings
#define BOOST_ON    (1U)
#define BOOST_OFF   (0U)
// GAIN REGISTER END      -----------------------------

// STATUS REGISTER START  -----------------------------
// Register read behavior
#define READ_SINGLE   (0U)
#define READ_GROUP    (1U)
#define READ_TYPE     (2U)
#define READ_ALL      (3U)

// Resolution
#define RES_24        (1U)
#define RES_16        (0U)

// 3 Cycle data latency for sinc3 filter settle time
#define DR_LTY_ON     (1U)
#define DR_LTY_OFF    (0U)

// High impedence when data NOT ready
#define DR_HIZ_ON     (0U)
#define DR_HIZ_OFF    (1U)

// Data ready link control
#define DR_LINK_ON    (1U)
#define DR_LINK_OFF   (0U)

// Data ready control Mode
#define DR_MODE_0    (0U)
#define DR_MODE_1    (1U)
#define DR_MODE_2    (2U)
#define DR_MODE_3    (3U)

// Bit offsets
#define DRSTATUS_CH_OFFSET  (0U)
#define DRA_MODE_OFFSET   (6U)
#define DRB_MODE_OFFSET   (8U)
#define DRC_MODE_OFFSET   (10U)
#define DR_LINK_OFFSET    (12U)
#define DR_HIZ_OFFSET     (13U)
#define DR_LTY_OFFSET     (14U)
#define RES_CHN_OFFSET    (15U)
#define READ_MODE_OFFSET  (22U)
// STATUS REGISTER END    -----------------------------


// CONFIG REGISTER START  -----------------------------
// Reset mode
#define RESET_ON          (1U)
#define RESET_OFF         (0U)

// Shutdown mode
#define SHUTDOWN_ON       (1U)
#define SHUTDOWN_OFF      (0U)

// Dither mode
#define DITHER_ON         (1U)
#define DITHER_OFF        (0U)

// Over Sampling ratio settings
#define OSR_32            (0U)
#define OSR_64            (1U)
#define OSR_128           (2U)
#define OSR_256           (3U)

// Prescaler settings
#define PRESCALE_1        (0U)
#define PRESCALE_2        (1U)
#define PRESCALE_4        (2U)
#define PRESCALE_8        (3U)

// External voltage reference select
#define EXTVREF_ON        (1U)
#define EXTVREF_OFF       (0U)

// External clock source select
#define EXTCLK_ON         (1U)
#define EXTCLK_OFF        (0U)

// Bit offsets
#define EXTCLK_OFFSET       (0U)
#define EXTVREF_OFFSET      (1U)
#define PRESCALE_OFFSET     (2U)
#define OSR_OFFSET          (4U)
#define DITHER_CHN_OFFSET   (6U)
#define SHUTDOWN_CHN_OFFSET (12U)
#define RESET_CHN_OFFSET    (18U)
// CONFIG REGISTER END    -----------------------------

// MCP3909 individual channel configurations
typedef struct {
  uint8_t   PGA;          // ADC gain setting
  uint8_t   shutdown;     // ADC shutdown mode
  uint8_t	reset;		  // ADC reset mode
  uint8_t   dither;       // ADC dither filter
  uint8_t   resolution;   // ADC resolution
  uint8_t   boost;        // ADC boost mode
} Channel_Conf;

// MCP3909 Handle
typedef struct {
  SPI_HandleTypeDef *	hspi;	// SPI Handle object
  uint8_t       * volatile pRxBuf;     // DMA Rx Buffer
  uint8_t       * volatile pTxBuf;     // DMA Tx Buffer ; User functions may use this buffer for transmission staging
  uint8_t		readType;		// Read single, type, group, all registers
  uint8_t       prescale;
  uint8_t       osr;
  uint8_t       extCLK;
  uint8_t       extVREF;
  uint8_t       phase[MAX_CHN_SET_NUM];
  Channel_Conf  channel[MAX_CHANNEL_NUM];
  uint32_t		registers[REGS_NUM];
} MCP3909HandleTypeDef;

// User library functions
// DO NOT USE AFTER RTOS SCHEDULER STARTS!!!!!!!!!
uint8_t mcp3909_SPI_WriteRegSync(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * data, uint8_t length, uint32_t timeout);
uint8_t mcp3909_SPI_ReadRegSync(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * buffer, uint8_t readLen, uint8_t readType, uint32_t timeout);


// USE AFTER SCHEDULER STARTS
uint8_t _mcp3909_SPI_WriteReg(MCP3909HandleTypeDef * hmcp, uint8_t address);

// SPI Utility functions
uint8_t mcp3909_SPI_WriteReg(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * data, uint8_t length); // Copies data into pTxBuf
uint8_t mcp3909_SPI_ReadReg(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * buffer, uint8_t readType, uint8_t length);  // Read data into user-defined buffer address
uint8_t mcp3909_SPI_ReadGroup(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * buffer);  // Read data into user-defined buffer address
uint8_t mcp3909_SPI_ReadType(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * buffer);
uint8_t mcp3909_SPI_ReadAll(MCP3909HandleTypeDef * hmcp, uint8_t address, uint8_t * buffer);  // Read data into user-defined buffer address

// Initialization
uint8_t mcp3909_init(MCP3909HandleTypeDef * hmcp);

// Setting verification
uint8_t mcp3909_verify(MCP3909HandleTypeDef * hmcp);

// Enter low-power mode
uint8_t mcp3909_sleep(MCP3909HandleTypeDef * hmcp);

// Exit low-power mode
uint8_t mcp3909_wakeup(MCP3909HandleTypeDef * hmcp);

// Obtain channel info
uint8_t mcp3909_readAllChannels(MCP3909HandleTypeDef * hmcp, uint8_t * buffer);

uint8_t mcp3909_readChannel(MCP3909HandleTypeDef * hmcp, uint8_t channelNum, uint8_t * buffer);

// Parse the data in the DMA Rx buffer and store to MCP Handle registers
void mcp3909_parseChannelData(MCP3909HandleTypeDef * hmcp);

//void bytesToReg(uint8_t * byte, uint32_t * reg);
uint32_t bytesToReg(uint8_t * byte);
void regToBytes(uint32_t * reg, uint8_t * bytes);

#endif /* MCP3909_H_ */
