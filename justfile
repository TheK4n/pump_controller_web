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
