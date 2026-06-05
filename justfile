#!/usr/bin/env -S just --justfile

PORT := env("PORT", "/dev/ttyUSB0")
ESPRESSIF := env("ESPRESSIF", "${HOME}/playground/esp-idf")


init:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py set-target esp32

build:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    xxd -i assets/index.html > main/frontend.h
    idf.py build


alias flash := upload
upload:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" flash


alias term := monitor
monitor:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" monitor
