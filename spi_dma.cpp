/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Example of writing via DMA to the SPI interface and similarly reading it back via a loopback.

#include <stdio.h>
#include <stdlib.h>
//#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "ili9341.hpp"
#include "3dconfig.hpp"
#include "hardware/vreg.h"
#define VREG_VSEL VREG_VOLTAGE_1_30

#define LCD_RST 21
#define LCD_DC 20
#define LCD_MOSI 19
#define LCD_MISO 16
#define LCD_CLK 18
#define LCD_CS 17
extern "C"{
  void multicore_launch_core1(void (*entry)(void));
  int main3d(void);
  void send_line(int ypos, uint8_t *line) ;
  void *vTask(void*);
}
LcdIli9341SPI lcd;

void send_line(int ypos, uint8_t *line){
  lcd.drawBufferu8(0,ypos,window_width,DRAW_NLINES,line);
}

int freq;

void core1_entry() {
  vTask((void*)0);
}

int main() {
  vreg_set_voltage(VREG_VSEL);
  sleep_ms(100);
  set_sys_clock_khz(300000, true);

  const LcdIli9341SPI::CONFIG cfg={
				   .spiPortNum=0,
				   .pinSck=LCD_CLK,
				   .pinMosi=LCD_MOSI,
				   .pinMiso=LCD_MISO,
				   .pinCs=LCD_CS,
				   .pinDc=LCD_DC,
				   .pinReset=LCD_RST
  };

  printf("%u kHz\n", freq=frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));
  lcd.initialize(cfg);
  // Enable UART so we can print status output
  stdio_init_all();
  printf("SPI DMA example\n");

  multicore_launch_core1(core1_entry);
  main3d();
  // while(1)
  //   lcd.test();
//     // Grab some unused dma channels
//     const uint dma_tx = dma_claim_unused_channel(true);
//     const uint dma_rx = dma_claim_unused_channel(true);

//     // Force loopback for testing (I don't have an SPI device handy)
//     hw_set_bits(&spi_get_hw(spi_default)->cr1, SPI_SSPCR1_LBM_BITS);

//     static uint8_t txbuf[TEST_SIZE];
//     static uint8_t rxbuf[TEST_SIZE];
//     for (uint i = 0; i < TEST_SIZE; ++i) {
//         txbuf[i] = rand();
//     }

//     // We set the outbound DMA to transfer from a memory buffer to the SPI transmit FIFO paced by the SPI TX FIFO DREQ
//     // The default is for the read address to increment every element (in this case 1 byte - DMA_SIZE_8)
//     // and for the write address to remain unchanged.

//     printf("Configure TX DMA\n");
//     dma_channel_config c = dma_channel_get_default_config(dma_tx);
//     channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
//     channel_config_set_dreq(&c, spi_get_index(spi_default) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
//     dma_channel_configure(dma_tx, &c,
//                           &spi_get_hw(spi_default)->dr, // write address
//                           txbuf, // read address
//                           TEST_SIZE, // element count (each element is of size transfer_data_size)
//                           false); // don't start yet

//     printf("Configure RX DMA\n");

//     // We set the inbound DMA to transfer from the SPI receive FIFO to a memory buffer paced by the SPI RX FIFO DREQ
//     // We coinfigure the read address to remain unchanged for each element, but the write
//     // address to increment (so data is written throughout the buffer)
//     c = dma_channel_get_default_config(dma_rx);
//     channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
//     channel_config_set_dreq(&c, spi_get_index(spi_default) ? DREQ_SPI1_RX : DREQ_SPI0_RX);
//     channel_config_set_read_increment(&c, false);
//     channel_config_set_write_increment(&c, true);
//     dma_channel_configure(dma_rx, &c,
//                           rxbuf, // write address
//                           &spi_get_hw(spi_default)->dr, // read address
//                           TEST_SIZE, // element count (each element is of size transfer_data_size)
//                           false); // don't start yet


//     printf("Starting DMAs...\n");
//     // start them exactly simultaneously to avoid races (in extreme cases the FIFO could overflow)
//     dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));
//     printf("Wait for RX complete...\n");
//     dma_channel_wait_for_finish_blocking(dma_rx);
//     if (dma_channel_is_busy(dma_tx)) {
//         panic("RX completed before TX");
//     }

//     printf("Done. Checking...");
//     for (uint i = 0; i < TEST_SIZE; ++i) {
//         if (rxbuf[i] != txbuf[i]) {
//             panic("Mismatch at %d/%d: expected %02x, got %02x",
//                   i, TEST_SIZE, txbuf[i], rxbuf[i]
//             );
//         }
//     }

//     printf("All good\n");
//     dma_channel_unclaim(dma_tx);
//     dma_channel_unclaim(dma_rx);
//     return 0;
// #endif
}
