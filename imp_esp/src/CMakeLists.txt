# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# idf_component_register(SRCS ${app_sources})

idf_component_register(SRCS "main.c" "API/driver/MAX30102_ESP32C3.c" "API/max30102.c" "api_ssd1306/ssd1306.c" "api_ssd1306/ssd1306_spi.c" "api_ssd1306/ssd1306_i2c.c" INCLUDE_DIRS "." "./API/" "./API/driver/" "./api_ssd1306")
