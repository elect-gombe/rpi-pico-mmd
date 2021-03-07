project(spi_dma)
add_executable(spi_dma
  spi_dma.cpp
  ili9341.cpp
  font.c
  3dmain.cpp
  fvector2.cpp
  fvector3.cpp
  kbhit.c
  main.c
  matrix4.cpp
  texturepoly.cpp
  )

target_link_libraries(spi_dma pico_stdlib hardware_spi hardware_dma pico_multicore pico_sync)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_dma)

# add url via pico_set_program_url
example_auto_set_url(spi_dma)


target_compile_definitions(spi_dma  INTERFACE
  PICO_MULTICORE=1
  )

