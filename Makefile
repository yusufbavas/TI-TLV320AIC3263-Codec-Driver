# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for TI's shared transport line discipline
# and its protocol drivers (BT, FM, GPS)
#
snd-soc-tlv320aic326x-objs			:= tlv320aic3xxx-core.o \
													tlv320aic3xxx-i2c.o \
													tlv320aic3xxx-irq.o \
													tlv320aic326x.o \
													aic3xxx/aic3xxx_cfw_ops.o
obj-$(CONFIG_SND_SOC_AIC326X) 		+= snd-soc-tlv320aic326x.o

