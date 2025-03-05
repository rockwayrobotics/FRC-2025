#!/bin/bash

if [ "$1" = "0" ]; then
  pinctrl 5 op dl
  sleep 0.2
  pinctrl 14 op dh
else
  pinctrl 14 op dl
  sleep 0.2
  pinctrl 5 op dh
fi
