# rpi-pico-mmd
rp2040でMMD（OC済み、無保証ですー）

<img src="https://raw.githubusercontent.com/elect-gombe/rpi-pico-mmd/master/out.gif" alt="Miku" width="200">
# requirement

- RP2040(Raspberry pi pico)
- LCD(ILI9341 in this test)

wireing:
```c
#define LCD_RST 21
#define LCD_DC 20
#define LCD_MOSI 19
#define LCD_MISO 16
#define LCD_CLK 18
#define LCD_CS 17
```

# performance
Using DMA and utilize dual core CPU.

about 10fps.

すでにデュアルコア使って、DMAのオフロード入れてます。OCもしてみましたが、FPUないので厳しいですねぇ。
