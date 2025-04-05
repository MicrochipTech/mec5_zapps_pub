# mec5_zapps_pub
Zephyr external application framework for Microchip SCPG MEC5 parts. This framework is based upon the Zephyr framework at:
https://github.com/zephyrproject-rtos/example-application

This repo is currently used for development of new drivers
and applications.

## app_i2c
Sample application for developing the MEC5 I2C network
layer driver. The I2C-NL driver makes use of our MEC5
Zephyr DMA driver.

## app_mt
Sample application demonstrating how to create and start
Zephyr threads.

## app_qspi_flash
Sample application for developing the a Zephyr flash driver
using MEC5 QSPI controller. Zephyr's SPI driver does not
easily support SPI dual and quad modes. The Zephyr flash driver
was created for communicating with SPI flash devices.

## app_zbus
Sample application using Zephyr's ZBUS messaging framework.

