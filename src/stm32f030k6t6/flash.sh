#!/bin/bash

openocd -d3 -f ./openocd.cfg -c "program build/stm32f030k6t6.elf verify reset exit"