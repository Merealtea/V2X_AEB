CROSS_COMPILE =

.PHONY : clean

AS              = $(CROSS_COMPILE)as
LD              = $(CROSS_COMPILE)ld
CC              = $(CROSS_COMPILE)gcc
CPP             = $(CROSS_COMPILE)g++ -fPIC
AR              = $(CROSS_COMPILE)ar
NM              = $(CROSS_COMPILE)nm
STRIP           = $(CROSS_COMPILE)strip
OBJCOPY         = $(CROSS_COMPILE)objcopy
OBJDUMP         = $(CROSS_COMPILE)OBJDUMP
RANLIB			= $(CROSS_COMPILE)ranlib
CFLAGS			=  -I /usr/local/include/libCanCmd/
LDFLAGS		= -L /usr/lib/ -lCanCmd 
OBJS 			= 
all:	clean  testLikeCan
	
testLikeCan:
	$(CPP)  -g -o testLikeCan testLikeCan.c -lpthread  -lCanCmd
%.o: %.c
	$(CPP) $(CFLAGS) -g -fPIC -c  $< -o $@ 

clean :
	rm -fr *.o libCanCmd.so testLikeCan
install:
	cp dll/libCanCmd.so /usr/lib/
	ldconfig
uninstall:
	rm -f /usr/lib/libCanCmd.so

