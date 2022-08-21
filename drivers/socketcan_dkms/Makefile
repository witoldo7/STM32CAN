VERSION=0.0.1
obj-m+=wqcan.o

link:
	sudo dkms add `pwd`

build:
	sudo dkms build wqcan/$(VERSION)

install:
	sudo dkms install wqcan/$(VERSION)

all:	build install

uninstall:
	sudo dkms uninstall wqcan/$(VERSION)
	sudo dkms remove wqcan/$(VERSION) --all

