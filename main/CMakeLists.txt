idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_lcd esp_wifi esp_netif
             esp_event nvs_flash esp_http_server esp_timer esp_rom lwip
    EMBED_TXTFILES "index.html"
)
