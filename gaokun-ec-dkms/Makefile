# SPDX-License-Identifier: GPL-2.0-only
MODNAME = gaokun-ec
MODVER = 0.1

obj-m += $(MODNAME).o
$(MODNAME)-y += $(MODNAME)-main.o

obj-m += gaokun-battery.o \
	 gaokun-wmi.o     \
	 gaokun-ucsi.o

gaokun-ucsi-y := gaokun-ucsi-simple.o


KVER = $(shell uname -r)
KDIR = /lib/modules/$(KVER)/build
MDIR = /usr/src/$(MODNAME)-$(MODVER)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

dkmsinstall:
	cp -R . /usr/src/$(MODNAME)-$(MODVER)
	dkms install -m $(MODNAME) -v $(MODVER)

dkmsremove:
	dkms remove -m $(MODNAME) -v $(MODVER) --all
	rm -rf /usr/src/$(MODNAME)-$(MODVER)
