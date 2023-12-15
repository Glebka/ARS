#!/bin/bash

openocd -d2 -f ./openocd.cfg -c "init reset exit"