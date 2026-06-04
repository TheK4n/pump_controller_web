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
    idf.py


upload:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" flash


term:
    #!/bin/sh
    . "{{ESPRESSIF}}/export.sh"
    idf.py -p "{{PORT}}" monitor
