#!/bin/bash
less build/log/"`ls -lrt build/log/ | tail -n 1 | cut -d: -f2- | cut -d' ' -f2-`"
