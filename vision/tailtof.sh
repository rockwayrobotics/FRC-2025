#!/bin/bash

tail -f $(ls logs/*.log | tail -1)
