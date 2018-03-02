#
# Makefile for the ISDB-T FC8180 drivers.
#

# Each configuration option enables a list of files.
# LGE
obj-y		+= bbm.o
obj-y		+= fc8180_drv_api.o
obj-y		+= fc8180_bb.o
obj-y		+= fci_hpi.o
obj-y		+= fc8180_isr.o
obj-y		+= fc8180_i2c.o
obj-y		+= fc8180_ppi.o
obj-y		+= fc8180_spi.o
obj-y		+= fc8180_spib.o
obj-y		+= fc8180_tun.o
obj-y		+= fci_hal.o
obj-y		+= fci_i2c.o
obj-y		+= fci_oal.o
obj-y		+= fci_bypass.o
obj-y		+= fci_ringbuffer.o
obj-y		+= fci_tun.o

#MTK
#EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/broadcast/oneseg/fc8180
#EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/broadcast/oneseg/fc8180/drv

#QCT
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/fc8180
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/fc8180/drv
