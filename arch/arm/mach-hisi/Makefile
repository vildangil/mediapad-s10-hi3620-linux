#
# Makefile for Hisilicon processors family
#

CFLAGS_platmcpm.o	:= -march=armv7-a

obj-y	+= hisilicon.o
obj-$(CONFIG_ARCH_HI3xxx)	+= hi3620-usb.o
obj-$(CONFIG_MCPM)		+= platmcpm.o
obj-$(CONFIG_SMP)		+= platsmp.o hotplug.o
