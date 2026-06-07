#!/usr/bin/env -S just --justfile

PORT := env("PORT", "/dev/ttyUSB0")
ESP_IDF := env("ESP_IDF", "${HOME}/playground/esp-idf")


init:
    #!/bin/sh
    . "{{ESP_IDF}}/export.sh"
    idf.py set-target esp32

build:
    #!/bin/sh
    . "{{ESP_IDF}}/export.sh"
    xxd -i assets/index.html > main/frontend.h
    sed -i '1i\static const ' main/frontend.h
    xxd -i assets/setup.html > main/setup_frontend.h
    sed -i '1i\static const ' main/setup_frontend.h
    xxd -i assets/favicon.svg > main/favicon_svg.h
    sed -i '1i\static const ' main/favicon_svg.h
    idf.py build


alias flash := upload
upload:
    #!/bin/sh
    . "{{ESP_IDF}}/export.sh"
    idf.py -p "{{PORT}}" flash


alias term := monitor
monitor:
    #!/bin/sh
    . "{{ESP_IDF}}/export.sh"
    idf.py -p "{{PORT}}" monitor
