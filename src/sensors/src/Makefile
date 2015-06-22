#
# Robovero samples/lib Makefile
# David St-Onge 04/2014
#
.PHONY: clean

# Cross compiler
CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++
CXX = g++

SRCS	= $(wildcard *.c)
OBJS	= $(patsubst %.c,%.o,${SRCS})
#LIBS	= -lpthread -lm

CFLAGS = -O2
CXXFLAGS = -O2

# for debugging
#CFLAGS 	= -g

TARGET	= robovero_test

.PHONY: all clean depend

robovero_test: main.o ahrs.o robocomm.o
	@echo "Building $@"
	@$(CXX) -o $@ $? $(LIBS) $(INCS) $(CXXFLAGS)
	#scp robovero_test linaro@192.168.10.241:robovero_test
	
all: 	${TARGET}

clean : 
	@echo "Cleaning"
	@rm -f *.o .depend ${TARGET} *~

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} ${INCS} -Y -f .depend ${SRCS}

%.o:	%.c
	@echo "Compiling $@"
	$(CC) $(INCS) -c $(CFLAGS) $< -o $@

ifeq (.depend,$(wildcard .depend))
include .depend 
endif
