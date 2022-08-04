#!/bin/bash
sudo rmmod wqcan
sudo make uninstall
make link
sudo make install
sudo modprobe wqcan
