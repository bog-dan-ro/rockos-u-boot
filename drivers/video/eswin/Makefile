# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright 2024 Beijing ESWIN Computing Technology Co., Ltd.

ifeq ($(CONFIG_DRM_ESWIN_VO_DEBUG), y)
	ccflags-y += -DVO_DEBUG
endif

obj-y += eswin_display.o eswin_crtc.o \
		eswin_dc.o eswin_dc_reg.o eswin_dc_test.o bmp_helper.o eswin_dc_mmu.o

obj-$(CONFIG_DRM_ESWIN_DW_MIPI_DSI) += eswin_dw_mipi_dsi.o dw_mipi_dsi.o
obj-$(CONFIG_DRM_ESWIN_DW_HDMI) += eswin_dw_hdmi.o dw_hdmi.o eswin_edid.o eswin_modes.o
obj-$(CONFIG_DRM_ESWIN_PANEL) += eswin_panel.o
