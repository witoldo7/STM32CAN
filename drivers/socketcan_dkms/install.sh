#!/bin/bash
sudo rmmod wqcan
sudo make uninstall
sudo make link
sudo make install
sudo modprobe wqcan
