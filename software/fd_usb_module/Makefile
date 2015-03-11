# Select operating system {UNIX, WINDOWS}
OS=		UNIX

# Use the following lines on Unix and Mac OS X:
CC=		gcc
OBJS=		fd_usb_module.o hiddata.o
SRCS = 		fd_usb_module.c hiddata.c
PROGRAM=	fd_usb_module.so
CFLAGS=		-I/usr/include/python2.7 -lusb -O -Wall `libusb-config --cflags` -shared -fPIC

all: $(PROGRAM)

$(PROGRAM): fd_usb_module.c
	# Compiling the module for USB communication with the device... 
	$(CC) $(ARCH_COMPILE) $(SRCS) $(CFLAGS) -D$(OS) -o $(PROGRAM)

clean:
	rm -f $(OBJS) $(PROGRAM)
