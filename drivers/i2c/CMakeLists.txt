# Copyright (c) 2025 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MCHP_MEC5_I2C_NL)
zephyr_library()
zephyr_library_sources(i2c_mchp_mec5_nl.c)
endif()

if(CONFIG_MCHP_MEC5_I2C_NL_ZD)
zephyr_library()
zephyr_library_sources(i2c_mchp_mec5_nl_zd.c)
endif()
