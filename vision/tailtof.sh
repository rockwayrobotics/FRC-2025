#!/bin/bash

NAME="$(ls logs/*.log | tail -1)"
echo File: $NAME
tail -f "$NAME"
