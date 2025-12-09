idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_uart driver spi_flash esp_adc esp_timer
                    )
