# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_FLASH_MCHP_MEC5_QSPI_NOR)
zephyr_library()
zephyr_library_sources(flash_mec5_qspi_nor.c)
endif()
