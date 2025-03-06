#!/bin/bash

# GPIO5 is "dh" (driven high) when right sensor active
# GPIO14 is "dh" when left sensor active
# both will be "dl" (driven low) when inactive
watch -n 0.3 pinctrl 5,14
