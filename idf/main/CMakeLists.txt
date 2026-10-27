idf_component_register(
    SRCS "main.c" "logic.c" "mailbox.c" "motors.c" "net.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_http_server esp_adc nvs_flash esp_timer esp_netif esp_event driver esp_hw_support lwip
    EMBED_TXTFILES "../../web/index.html" "../../web/styles.css" "../../web/app.js"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
