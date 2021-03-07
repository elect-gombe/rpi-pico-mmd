# rpi-pico-mmd
rp2040でMMD（OC済み、無保証ですー）

<img src="https://raw.githubusercontent.com/elect-gombe/rpi-pico-mmd/master/out.gif" alt="Miku" width="200">
# what to use
- RP2040(Raspberry pi pico)
- LCD(ILI9341 in this test)

# performance
Using DMA and utilize dual core CPU.
10fpsくらいです。デュアルコア使って、DMAのオフロード入れてます。
