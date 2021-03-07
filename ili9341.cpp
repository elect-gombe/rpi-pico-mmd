//https://github.com/iwatake2222/pico-mnist
//Apache License

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <array>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "font.h"
#include "ili9341.hpp"


typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

static int hastransfered;

/* //Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA. */
/* DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[]={ */
/*     /\* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 *\/ */
/*     {0x36, {(1<<5)|(1<<6)}, 1}, */
/*     /\* Interface Pixel Format, 16bits/pixel for RGB/MCU interface *\/ */
/*     {0x3A, {0x55}, 1}, */
/*     /\* Porch Setting *\/ */
/*     {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5}, */
/*     /\* Gate Control, Vgh=13.65V, Vgl=-10.43V *\/ */
/*     {0xB7, {0x45}, 1}, */
/*     /\* VCOM Setting, VCOM=1.175V *\/ */
/*     {0xBB, {0x2B}, 1}, */
/*     /\* LCM Control, XOR: BGR, MX, MH *\/ */
/*     {0xC0, {0x2C}, 1}, */
/*     /\* VDV and VRH Command Enable, enable=1 *\/ */
/*     {0xC2, {0x01, 0xff}, 2}, */
/*     /\* VRH Set, Vap=4.4+... *\/ */
/*     {0xC3, {0x11}, 1}, */
/*     /\* VDV Set, VDV=0 *\/ */
/*     {0xC4, {0x20}, 1}, */
/*     /\* Frame Rate Control, 60Hz, inversion=0 *\/ */
/*     {0xC6, {0x0f}, 1}, */
/*     /\* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V *\/ */
/*     {0xD0, {0xA4, 0xA1}, 1}, */
/*     /\* Positive Voltage Gamma Control *\/ */
/*     {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14}, */
/*     /\* Negative Voltage Gamma Control *\/ */
/*     {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14}, */
/*     /\* Sleep Out *\/ */
/*     {0x11, {0}, 0x80}, */
/*     /\* Display On *\/ */
/*     {0x29, {0}, 0x80}, */
/*     {0, {0}, 0xff} */
/* }; */

static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

static inline auto getSpi(int32_t spiNum)
{
	return (spiNum == 0) ? spi0 : spi1;
}

int32_t LcdIli9341SPI::initialize(const CONFIG& config)
{
	m_spiPortNum = config.spiPortNum;
	m_pinSck = config.pinSck;
	m_pinMosi = config.pinMosi;
	m_pinMiso = config.pinMiso;
	m_pinCs = config.pinCs;
	m_pinDc = config.pinDc;
	m_pinReset = config.pinReset;

	m_charPosX = 0;
	m_charPosY = 0;

	initializeIo();
	initializeDevice();
	setArea(0, 0, WIDTH, HEIGHT);

	return RET_OK;
}
int spibaud;
void LcdIli9341SPI::initializeIo(void)
{
  dma_tx = dma_claim_unused_channel(true);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(5);
    gpio_set_dir(5, GPIO_OUT);
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        300*1000 * 1000,                               // Input frequency
        150*1000*1000                                // Output (must be same as no divider)
    );

    spi_init(getSpi(m_spiPortNum), 100*1000 * 1000);
    spibaud=spi_set_baudrate(getSpi(m_spiPortNum),50*1000*1000);
	gpio_set_function(m_pinSck , GPIO_FUNC_SPI);
	gpio_set_function(m_pinMosi , GPIO_FUNC_SPI);
	gpio_set_function(m_pinMiso, GPIO_FUNC_SPI);

	// gpio_set_function(m_pinCs, GPIO_FUNC_SPI);
	gpio_init(m_pinCs);
	gpio_set_dir(m_pinCs, GPIO_OUT);
	disableCs();

	gpio_init(m_pinDc);
	gpio_set_dir(m_pinDc, GPIO_OUT);

	gpio_init(m_pinReset);
	gpio_set_dir(m_pinReset, GPIO_OUT);
	gpio_put(m_pinReset, 0);
	sleep_ms(50);
	gpio_put(m_pinReset, 1);
	sleep_ms(50);

}

void LcdIli9341SPI::initializeDevice(void)
{
  int cmd=0;
  const lcd_init_cmd_t* lcd_init_cmds;
  lcd_init_cmds = ili_init_cmds;
  while (lcd_init_cmds[cmd].databytes!=0xff) {
    writeCmd(lcd_init_cmds[cmd].cmd);
    writeData(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
    if (lcd_init_cmds[cmd].databytes&0x80) {
      sleep_ms(100);
    }
    cmd++;
  }
  // writeCmd(0x01);	// Software Reset
	// sleep_ms(50);
	// writeCmd(0x11);	// Sleep Out
	// sleep_ms(50);
	
	// uint8_t dataBuffer[4];
	// writeCmd(0xB6);	// Display Function Control
	// dataBuffer[0] = 0x0a;	// Default
	// dataBuffer[1] = 0xc2;	// G320 -> G1
	// writeData(dataBuffer, 2);

	// writeCmd(0x36);	// Memory Access Control
	// writeData(0x68);	// Row Address Order, Row / Column Exchange, BGR
	// writeCmd(0x3A);	// Pixel Format Set
	// writeData(0x55);	// 16-bit

	// writeCmd(0x29);	// Display ON
}

int32_t LcdIli9341SPI::finalize(void)
{
	spi_deinit(getSpi(m_spiPortNum));
	return RET_OK;
}

void LcdIli9341SPI::setArea(int32_t x, int32_t y, int32_t w, int32_t h)
{
	uint8_t dataBuffer[4];
	writeCmd(0x2A);
	dataBuffer[0] = (x >> 8) & 0xFF;
	dataBuffer[1] = x & 0xFF;
	dataBuffer[2] = ((x + w - 1) >> 8) & 0xFF;
	dataBuffer[3] = (x + w - 1) & 0xFF;
	writeData(dataBuffer, 4);
	writeCmd(0x2B);
	dataBuffer[0] = (y >> 8) & 0xFF;
	dataBuffer[1] = y & 0xFF;
	dataBuffer[2] = ((y + h - 1) >> 8) & 0xFF;
	dataBuffer[3] = (y + h - 1) & 0xFF;
	writeData(dataBuffer, 4);
}

void LcdIli9341SPI::putPixel(int32_t x, int32_t y, std::array<uint8_t, 2> color)
{
	setArea(x, y, 1, 1);
	writeCmd(0x2C);
	writeData(color.data(), 2);
}

void LcdIli9341SPI::drawRect(int32_t x, int32_t y, int32_t w, int32_t h, std::array<uint8_t, 2> color)
{
	setArea(x, y, w, h);
	writeCmd(0x2C);
	for (int32_t i = 0; i < w * h; i++) {
		writeData(color.data(), 2);
	}
}

void LcdIli9341SPI::drawBuffer(int32_t x, int32_t y, int32_t w, int32_t h, std::vector<uint8_t> buffer)
{
	if (w * h * 2 != buffer.size()) {
		printf("error at LcdIli9341SPI::drawBuffer\n");
		return;
	}
	setArea(x, y, w, h);
	writeCmd(0x2C);
	writeData(buffer.data(), buffer.size());
}

void LcdIli9341SPI::drawBufferu8(int32_t x, int32_t y, int32_t w, int32_t h, const uint8_t *buffer)
{
  setArea(x, y, w, h);
  writeCmd(0x2C);
  writeData(buffer, w*h*2);
}

void LcdIli9341SPI::drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t size, std::array<uint8_t, 2> color)
{
	if (x0 != x1) {
		float a = (y1 - y0) / static_cast<float>(x1 - x0);
		int32_t b = (x1 * y0 - x0 * y1) / (x1 - x0);
		for (int32_t x = std::min(x0, x1); x <= std::max(x0, x1); x++) {
			int32_t y = static_cast<int32_t>(a * x + b);
			// putPixel(x, y, color);
			drawRect(x, y, size, size, color);
		}
	} else {
		for (int32_t y = std::min(y0, y1); y <= std::max(y0, y1); y++) {
			// putPixel(x0, y, color);
			drawRect(x0, y, size, size, color);
		}
	}
}

void LcdIli9341SPI::drawChar(int32_t x, int32_t y, char c)
{
	constexpr int32_t COLOR_SIZE = 2;
	std::vector<uint8_t> charBuffer(FONT_WIDTH * FONT_DISPLAY_SIZE * FONT_HEIGHT * FONT_DISPLAY_SIZE * COLOR_SIZE);
	for (int32_t x = 0; x < FONT_WIDTH; x++ ) {
		uint8_t line = *(font + ((c & 0x7F) * FONT_WIDTH) + x);
		for (uint8_t y = 0; y < FONT_HEIGHT; y++) {
			for (uint8_t sizeX = 0; sizeX < FONT_DISPLAY_SIZE; sizeX++) {
				for (uint8_t sizeY = 0; sizeY < FONT_DISPLAY_SIZE; sizeY++) {
					int32_t index = y * (FONT_DISPLAY_SIZE * FONT_WIDTH * FONT_DISPLAY_SIZE * COLOR_SIZE) + sizeY * (FONT_WIDTH * FONT_DISPLAY_SIZE * COLOR_SIZE) + x * (FONT_DISPLAY_SIZE * COLOR_SIZE) + sizeX * COLOR_SIZE;
					if (line & 0x1) {
						for (int32_t color = 0; color < COLOR_SIZE; color++) {
							charBuffer[index + color] = COLOR_TEXT_FG[color];
						}
					} else {
						for (int32_t color = 0; color < COLOR_SIZE; color++) {
							charBuffer[index + color] = COLOR_TEXT_BG[color];
						}
					}
				}
			}
			line >>= 1;
		}
	}
	drawBuffer(x, y, FONT_WIDTH * FONT_DISPLAY_SIZE, FONT_HEIGHT * FONT_DISPLAY_SIZE, charBuffer);
}

void LcdIli9341SPI::putChar(char c)
{
	drawChar(m_charPosX, m_charPosY, c);
	m_charPosX += FONT_WIDTH * FONT_DISPLAY_SIZE;
	if (m_charPosX + FONT_WIDTH * FONT_DISPLAY_SIZE >= WIDTH) {
		m_charPosY += FONT_HEIGHT * FONT_DISPLAY_SIZE;
		m_charPosX = 0;
		if (m_charPosY + FONT_HEIGHT * FONT_DISPLAY_SIZE >= HEIGHT) {
			m_charPosY = 0;
		}
	}
}

void LcdIli9341SPI::putText(std::string text)
{
	for (auto c : text) {
		if (c == '\0') break;
		putChar(c);
	}
}

void LcdIli9341SPI::setCharPos(int32_t charPosX, int32_t charPosY)
{
	m_charPosX = charPosX;
	m_charPosY = charPosY;
}




void LcdIli9341SPI::enableCs(void)
{
	// asm volatile("nop \n nop \n nop");
	gpio_put(m_pinCs, 0);	// Active low
	// asm volatile("nop \n nop \n nop");
}

void LcdIli9341SPI::disableCs(void)
{
	// asm volatile("nop \n nop \n nop");
	gpio_put(m_pinCs, 1);	// Active low
	// asm volatile("nop \n nop \n nop");
}

void LcdIli9341SPI::writeCmd(uint8_t cmd)
{
	waitForFinish();
	gpio_put(m_pinDc, 0);
	enableCs();
	spi_write_blocking(getSpi(m_spiPortNum), &cmd, 1);
	disableCs();
}

void LcdIli9341SPI::writeData(uint8_t data)
{
	waitForFinish();
	gpio_put(m_pinDc, 1);
	enableCs();
	spi_write_blocking(getSpi(m_spiPortNum), &data, 1);
	disableCs();
}

void LcdIli9341SPI::waitForFinish(){
  spi_inst_t* spi=getSpi(m_spiPortNum);
  if(hastransfered){
    gpio_put(PICO_DEFAULT_LED_PIN,1);
    gpio_put(0,1);
  dma_channel_wait_for_finish_blocking(dma_tx);
    // Drain RX FIFO, then wait for shifting to finish (which may be *after*
    // TX FIFO drains), then drain RX FIFO again
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;
    while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;

    // Don't leave overrun flag set
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;
    disableCs();
    hastransfered=0;
    gpio_put(PICO_DEFAULT_LED_PIN,0);
    gpio_put(0,0);
  }
}

void LcdIli9341SPI::writeData(const uint8_t dataBuffer[], int32_t len)
{
  
	waitForFinish();
  gpio_put(m_pinDc, 1);
  enableCs();
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, m_spiPortNum ? DREQ_SPI1_TX : DREQ_SPI0_TX);
    dma_channel_configure(dma_tx, &c,
                          &spi_get_hw(spi_default)->dr, // write address
                          dataBuffer, // read address
                          len, // element count (each element is of size transfer_data_size)
                          true); // let spi start 
    hastransfered=1;
    //	spi_write_blocking(getSpi(m_spiPortNum), dataBuffer, len);
    //    disableCs();
}

void LcdIli9341SPI::readData(uint8_t cmd, uint8_t dataBuffer[], int32_t len)
{
    gpio_put(m_pinDc, 0);
    enableCs();
    spi_write_blocking(getSpi(m_spiPortNum), &cmd, 1);
    
    gpio_put(m_pinDc, 1);
    // spi_read_blocking(getSpi(m_spiPortNum), 0, dataBuffer, len);
}

void LcdIli9341SPI::test()
{
	std::array<uint8_t, 2> colorBg = { 0x00, 0x1F };
	drawRect(0, 0, WIDTH, HEIGHT, colorBg);

	std::array<uint8_t, 2> color = { 0xF8, 0x00 };
	for(int32_t y = 100; y < 200; y++) {
		for(int32_t x = 100; x < 200; x++) {
			putPixel(x, y, color);
		}
	}

	std::vector<uint8_t> colorBuffer;
	for(int32_t i = 0; i < 20 * 20; i++) {
		colorBuffer.push_back(0x07);
		colorBuffer.push_back(0xE0);
	}
	drawBuffer(10, 10, 20, 20, colorBuffer);

	std::array<uint8_t, 2> colorLine = { 0x00, 0x00 };
	drawLine(50, 50, 100, 100, 2, colorLine);
	drawLine(100, 100, 150, 50, 2, colorLine);
	drawLine(150, 50, 100, 0, 2, colorLine);
	drawLine(100, 0, 50, 50, 2, colorLine);
	drawLine(50, 50, 150, 50, 2, colorLine);
	drawLine(50, 50,  50, 150, 2, colorLine);

	putText("ABC");
}
