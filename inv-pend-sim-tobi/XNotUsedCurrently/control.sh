#!/bin/bash
# launch python 3 with the control.py script

#source activate python3
ret=`python -c 'import sys; print("%i" % (sys.hexversion>0x03000000))'`
if [ $ret -eq 0 ]; then
    echo 'requires python version >3 (did you forget to "source activate" correct version?)'
else 
    echo "python version is >3; ok for running control.py"
fi
python control.py

