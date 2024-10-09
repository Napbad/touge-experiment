cd /home/CQUCS/ucore-loongarch32-cqu2022_noanswer

cat << 'EOF' > Makefile
# LAB CONFIG BEGIN

# Warning: If you changed anything in Makefile, 
# you should execute `make clean` first.

LAB1    := -DLAB1_EX2 -DLAB1_EX3 -D_SHOW_100_TICKS -D_SHOW_SERIAL_INPUT
# LAB2  := -DLAB2_EX1 -DLAB2_EX2 -DLAB2_EX3
# LAB3  := -DLAB3_EX1 -DLAB3_EX2
# LAB4  := -DLAB4_EX1 -DLAB4_EX2

# LAB CONFIG END

ifdef LAB4
	USER_OBJ_MODE   := initrd
else
	USER_OBJ_MODE   := piggy
endif

EMPTY   :=
SPACE   := $(EMPTY) $(EMPTY)
SLASH   := /

V       := @

GCCPREFIX:= loongarch32r-linux-gnusf-

QEMU:= qemu-system-loongarch32
QEMUOPTS:= -M ls3a5k32 -m 32 -nographic

# use the same qemu as use_for_linux

TERMINAL := gnome-terminal
TERMINALOPT := -e

# eliminate default suffix rules
.SUFFIXES: .c .S .h

# define compiler and flags

HOSTCC	  := gcc
HOSTCFLAGS      := -g -Wall -O2

GDB	     := loongarch32r-linux-gnusf-gdb

CC :=$(GCCPREFIX)gcc
LAB_FLA := $(LAB1) $(LAB2) $(LAB3) $(LAB4)
CFLAGS  := $(LAB_FLA) -fno-builtin-fprintf -fno-builtin -nostdlib  -nostdinc -g -G0 -Wa,-O0 -fno-pic -msoft-float -ggdb -gstabs 
CTYPE   := c S

LD      := $(GCCPREFIX)ld
AR      := $(GCCPREFIX)ar
LDFLAGS += -nostdlib -m elf32loongarch

OBJCOPY := $(GCCPREFIX)objcopy
OBJDUMP := $(GCCPREFIX)objdump

COPY    := cp
MKDIR   := mkdir -p
MV	      := mv
RM	      := rm -f
AWK	     := awk
SED	     := sed
SH	      := sh
TR	      := tr
TOUCH   := touch -c

TAR	     := tar
ZIP	     := gzip

OBJDIR  := obj
BINDIR  := bin
SRCDIR  := kern
DEPDIR  := dep

MODULES   := init driver libs trap mm debug sync process schedule syscall fs fs/vfs fs/sfs fs/devs

SRC_DIR   := $(addprefix $(SRCDIR)/,$(MODULES))
BUILD_DIR := $(addprefix $(OBJDIR)/,$(MODULES))
DEP_DIR   := $(addprefix $(DEPDIR)/,$(MODULES))
VPATH     += $(SRC_DIR)

SRC       := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
OBJ       := $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(SRC))
ASMSRC    := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.S))
OBJ       += $(patsubst $(SRCDIR)/%.S, $(OBJDIR)/%.o, $(ASMSRC))
INCLUDES  := $(addprefix -I,$(SRC_DIR))
INCLUDES  += -I$(SRCDIR)/include

override ON_FPGA ?= n

ifeq ($(ON_FPGA), y)
	USER_APPLIST:= sh ls cat
	INITRD_BLOCK_CNT:=700 
else
	USER_APPLIST:= pwd cat sh ls forktest yield hello faultreadkernel faultread badarg waitkill pgdir exit sleep echo spin forktree
	INITRD_BLOCK_CNT:=4000
endif

USER_SRCDIR := user
USER_OBJDIR := $(OBJDIR)/$(USER_SRCDIR)
USER_LIB_OBJDIR := $(USER_OBJDIR)/libs
USER_INCLUDE := -I$(USER_SRCDIR)/libs	

USER_APP_BINS:= $(addprefix $(USER_OBJDIR)/, $(USER_APPLIST))

USER_LIB_SRCDIR := $(USER_SRCDIR)/libs
USER_LIB_SRC := $(foreach sdir,$(USER_LIB_SRCDIR),$(wildcard $(sdir)/*.c))
USER_LIB_OBJ := $(patsubst $(USER_LIB_SRCDIR)/%.c, $(USER_LIB_OBJDIR)/%.o, $(USER_LIB_SRC))
USER_LIB_OBJ += $(USER_LIB_OBJDIR)/initcode.o
USER_LIB    := $(USER_OBJDIR)/libuser.a

BUILD_DIR   += $(USER_LIB_OBJDIR)
BUILD_DIR   += $(USER_OBJDIR)


DEPENDS := $(patsubst $(SRCDIR)/%.c, $(DEPDIR)/%.d, $(SRC))

MAKEDEPEND = $(CLANG) -M $(CFLAGS) $(INCLUDES) -o $(DEPDIR)/$*.d $<
#vpath %.c $(SRC_DIR)
#vpath %.S $(SRC_DIR)

.PHONY: all checkdirs clean qemu debug

all: checkdirs $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)

$(shell mkdir -p $(DEP_DIR))

$(OBJDIR)/ucore-kernel:  checkdirs $(OBJ) tools/kernel.ld
	$(V)$(LD) -nostdlib -n -G 0 -static -T tools/kernel.ld $(OBJ) -o $@

$(OBJDIR)/ucore-kernel-piggy: $(BUILD_DIR)  $(OBJ) $(USER_APP_BINS) tools/kernel.ld
	$(V)$(LD) -nostdlib -n -G 0 -static -T tools/kernel.ld $(OBJ) \
					$(addsuffix .piggy.o, $(USER_APP_BINS)) -o $@
	$(V)$(OBJDUMP) -S $@ > $(OBJDIR)/kernel.asm
	$(V)$(OBJDUMP) -t $@ | $(SED) '1,/SYMBOL TABLE/d; s/ .* / /; /^$$/d' > $(OBJDIR)/kernel.sym

$(DEPDIR)/%.d: $(SRCDIR)/%.c
	@echo DEP $<
	$(V)set -e; rm -f $@; \
		$(CC) -MM -MT "$(OBJDIR)/$*.o $@" $(CFLAGS) $(INCLUDES) $< > $@; 

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(V)$(CC) -c $(INCLUDES) $(CFLAGS) $(MACH_DEF) $<  -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.S
	$(V)$(CC) -c -fno-pic -D__ASSEMBLY__ $(MACH_DEF) $(INCLUDES) -g -G0  $<  -o $@

checkdirs: $(BUILD_DIR) $(DEP_DIR)

$(BUILD_DIR):
	@mkdir -p $@

$(DEP_DIR):
	@mkdir -p $@

clean:
	-rm -rf $(DEPDIR)
	-rm -rf boot/loader.o boot/loader boot/loader.bin
	-rm -rf $(OBJDIR)

qemu: $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)
	$(V)$(QEMU) $(QEMUOPTS) -kernel $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)

debug: $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)
	$(V)$(QEMU) $(QEMUOPTS) -kernel $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE) -S -s

gdb: $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)
	$(V)$(GDB) $(OBJDIR)/ucore-kernel-$(USER_OBJ_MODE)

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPENDS)
endif

#user lib

$(USER_LIB): $(BUILD_DIR) $(USER_LIB_OBJ)
	$(V)$(AR) rcs $@ $(USER_LIB_OBJ)

#user applications
define make-user-app
$1: $(BUILD_DIR) $(addsuffix .o,$1) $(USER_LIB)
	$(V)$(LD) -T $(USER_LIB_SRCDIR)/user.ld  $(addsuffix .o,$1) --whole-archive $(USER_LIB) -o $$@
	$(V)$(SED) 's/$$$$FILE/$(notdir $1)/g' tools/piggy.S.in > $1.S
	$(V)$(CC) -c $1.S -o $$@.piggy.o
endef

$(foreach bdir,$(USER_APP_BINS),$(eval $(call make-user-app,$(bdir))))

$(USER_OBJDIR)/%.o: $(USER_SRCDIR)/%.c
	$(V)$(CC) -c  $(USER_INCLUDE) -I$(SRCDIR)/include $(CFLAGS)  $<  -o $@

$(USER_OBJDIR)/%.o: $(USER_SRCDIR)/%.S
	$(V)$(CC) -c -fno-pic -D__ASSEMBLY__ $(USER_INCLUDE) -I$(SRCDIR)/include -g -G0  $<  -o $@


# filesystem
TOOL_MKSFS := tools/mksfs
ROOTFS_DIR:= $(OBJDIR)/rootfs
ROOTFS_IMG:= $(OBJDIR)/initrd.img
$(TOOL_MKSFS): tools/mksfs.c
	$(HOSTCC) $(HOSTCFLAGS) -o $@ $^

$(OBJDIR)/initrd.img.o: $(TOOL_MKSFS) $(USER_APP_BINS)
	$(V)rm -rf $(ROOTFS_DIR) $(ROOTFS_IMG)
	$(V)mkdir $(ROOTFS_DIR)
	$(V)cp $(USER_APP_BINS) $(ROOTFS_DIR)
	$(V)cp -r $(USER_SRCDIR)/_archive/* $(ROOTFS_DIR)/
	$(V)dd if=/dev/zero of=$(ROOTFS_IMG) count=$(INITRD_BLOCK_CNT)
	$(V)$(TOOL_MKSFS) $(ROOTFS_IMG) $(ROOTFS_DIR)
	$(V)$(SED) 's%_FILE_%$(ROOTFS_IMG)%g' tools/initrd_piggy.S.in > $(USER_OBJDIR)/initrd_piggy.S
	$(V)$(CC) -c $(USER_OBJDIR)/initrd_piggy.S -o $(OBJDIR)/initrd.img.o

$(OBJDIR)/ucore-kernel-initrd: $(BUILD_DIR) $(OBJ) tools/kernel.ld $(OBJDIR)/initrd.img.o
	$(V)$(LD) -nostdlib -n -G 0 -static -T tools/kernel.ld $(OBJ) \
				 $(OBJDIR)/initrd.img.o -o $@
	$(V)$(OBJDUMP) -S $@ > $(OBJDIR)/kernel.asm
	$(V)$(OBJDUMP) -t $@ | $(SED) '1,/SYMBOL TABLE/d; s/ .* / /; /^$$/d' > $(OBJDIR)/kernel.sym

EOF

cd /home/CQUCS/ucore-loongarch32-cqu2022_noanswer/kern/driver

cat << 'EOF' > console.c
#include <defs.h>
#include <loongarch.h>
#include <stdio.h>
#include <string.h>
#include <picirq.h>
#include <intr.h>

/* stupid I/O delay routine necessitated by historical PC design flaws */
static void
delay(void) {
}

/***** Serial I/O code *****/

#define COM_RX          0       // In:  Receive buffer (DLAB=0)
#define COM_TX          0       // Out: Transmit buffer (DLAB=0)
#define COM_DLL         0       // Out: Divisor Latch Low (DLAB=1)
#define COM_DLM         1       // Out: Divisor Latch High (DLAB=1)
#define COM_IER         1       // Out: Interrupt Enable Register
#define COM_IER_RDI     0x01    // Enable receiver data interrupt
#define COM_IIR         2       // In:  Interrupt ID Register
#define COM_FCR         2       // Out: FIFO Control Register
#define COM_LCR         3       // Out: Line Control Register
#define COM_LCR_DLAB    0x80    // Divisor latch access bit
#define COM_LCR_WLEN8   0x03    // Wordlength: 8 bits
#define COM_MCR         4       // Out: Modem Control Register
#define COM_MCR_RTS     0x02    // RTS complement
#define COM_MCR_DTR     0x01    // DTR complement
#define COM_MCR_OUT2    0x08    // Out2 complement
#define COM_LSR         5       // In:  Line Status Register
#define COM_LSR_DATA    0x01    // Data available
#define COM_LSR_TXRDY   0x20    // Transmit buffer avail
#define COM_LSR_TSRE    0x40    // Transmitter off


static bool serial_exists = 0;

static void
serial_init(void) {
    volatile unsigned char *uart = (unsigned char*)COM1;
    if(serial_exists)
      return ;
    serial_exists = 1;
    // Turn off the FIFO
    outb(COM1 + COM_FCR, 0);
    // Set speed; requires DLAB latch
    outb(COM1 + COM_LCR, COM_LCR_DLAB);
    // outb(COM1 + COM_DLL, (uint8_t) (COM1_BAUD_DDL));
    // don't change the baud rate set by pmon
    outb(COM1 + COM_DLM, 0);

    // 8 data bits, 1 stop bit, parity off; turn off DLAB latch
    outb(COM1 + COM_LCR, COM_LCR_WLEN8 & ~COM_LCR_DLAB);

    // No modem controls
    outb(COM1 + COM_MCR, 0);
    // Enable rcv interrupts
    outb(COM1 + COM_IER, COM_IER_RDI);

    pic_enable(COM1_IRQ);
}


static void
serial_putc_sub(int c) {
    while (!(inb(COM1 + COM_LSR) & COM_LSR_TXRDY)) {
    }
    outb(COM1 + COM_TX, c);
}

/* serial_putc - print character to serial port */
static void
serial_putc(int c) {
    if (c == '\n') {
        serial_putc_sub('\r');
        serial_putc_sub('\n');
    }else {
        serial_putc_sub(c);
    }
}

/* serial_proc_data - get data from serial port */
static int
serial_proc_data(void) {
    int c;
    if (!(inb(COM1 + COM_LSR) & COM_LSR_DATA)) {
        return -1;
    }
    c = inb(COM1 + COM_RX);
    return c;
}


void serial_int_handler(void *opaque)
{
    unsigned char id = inb(COM1+COM_IIR);
    if(id & 0x01)
        return ;
    //int c = serial_proc_data();
    int c = cons_getc();
#if defined(LAB1_EX3) && defined(_SHOW_SERIAL_INPUT)
    // LAB1 EXERCISE3: YOUR CODE
        kprintf("got input %c \n", c);
	
#endif
#ifdef LAB4_EX2
    extern void dev_stdin_write(char c);
    dev_stdin_write(c);
#endif
}

/* *
 * Here we manage the console input buffer, where we stash characters
 * received from the keyboard or serial port whenever the corresponding
 * interrupt occurs.
 * */

#define CONSBUFSIZE 512

static struct {
    uint8_t buf[CONSBUFSIZE];
    uint32_t rpos;
    uint32_t wpos;
} cons;

/* *
 * cons_intr - called by device interrupt routines to feed input
 * characters into the circular console input buffer.
 * */
static void
cons_intr(int (*proc)(void)) {
    int c;
    while ((c = (*proc)()) != -1) {
        if (c != 0) {
            cons.buf[cons.wpos ++] = c;
            if (cons.wpos == CONSBUFSIZE) {
                cons.wpos = 0;
            }
        }
    }
}

/* serial_intr - try to feed input characters from serial port */
void
serial_intr(void) {
    if (serial_exists) {
        cons_intr(serial_proc_data);
    }
}

/* cons_init - initializes the console devices */
void
cons_init(void) {
    serial_init();
    //cons.rpos = cons.wpos = 0;
    if (!serial_exists) {
        kprintf("serial port does not exist!!\n");
    }
}

/* cons_putc - print a single character @c to console devices */
void
cons_putc(int c) {
    long intr_flag;
    local_intr_save(intr_flag);
    {
        serial_putc(c);
    }
    local_intr_restore(intr_flag);
}

/* *
 * cons_getc - return the next input character from console,
 * or 0 if none waiting.
 * */
int
cons_getc(void) {
    int c = 0;
    long intr_flag;
    local_intr_save(intr_flag);
    {
        // poll for any pending input characters,
        // so that this function works even when interrupts are disabled
        // (e.g., when called from the kernel monitor).
        serial_intr();

        // grab the next character from the input buffer.
        if (cons.rpos != cons.wpos) {
            c = cons.buf[cons.rpos ++];
            if (cons.rpos == CONSBUFSIZE) {
                cons.rpos = 0;
            }
        }
    }
    local_intr_restore(intr_flag);
    return c;
}
EOF

cd /home/CQUCS/ucore-loongarch32-cqu2022_noanswer 
make clean
make qemu

