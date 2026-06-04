#!/usr/bin/env -S just --justfile

PORT := env("PORT", "/dev/ttyUSB0")
ESPRESSIF := env("ESPRESSIF", "${HOME}/playground/espressif")


init:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py set-target esp32

build:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    xxd -i assets/index.html > main/frontend.h
    idf.py build


upload:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" flash


term:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" monitor
