# Utility functions
WHOAMI = $(shell whoami)
checkIfRoot = @if [ $(WHOAMI) != root ]; then echo "ERROR: you must be root (or use sudo)!"; false; fi


CC=gcc
CFLAGS=-c -Wall -O2
LDFLAGS=
CSOURCES=read_batt.c
COBJECTS=$(CSOURCES:.c=.o)
EXECUTABLE=ccub_batteryd

all: $(CSOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(COBJECTS) 
	$(CC) $(LDFLAGS) $(COBJECTS) -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@


install:
	$(call checkIfRoot)
	cp -f $(EXECUTABLE) /usr/sbin
	cp ccub_battery.sh /etc/init.d
	update-rc.d ccub_battery.sh defaults
	
clean:
	rm -f $(COBJECTS) $(EXECUTABLE)

