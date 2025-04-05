
import os
import sys
import argparse
import re
import string
import subprocess

def get_gpio_bank_name(gpio_name_str):
    gpio_num = int(gpio_name_str[4:], base=8)
    if gpio_num < 32:
        return "gpio_000_036"
    elif gpio_num < 64:
        return "gpio_040_076"
    elif gpio_num < 96:
        return "gpio_100_136"
    elif gpio_num < 128:
        return "gpio_140_176"
    elif gpio_num < 160:
        return "gpio_200_236"
    else:
        return "gpio_240_276"

def get_gpio_bitpos(gpio_name_str):
    gpio_num = int(gpio_name_str[4:], base=8)
    return (gpio_num % 32)

# gpio flags examples
# if item[2] exists
#   'GPIO,OUT,OD,S5=1,INIT=1'
#   'GPIO_OUT,PP,S5=0,INIT=0'
#   'GPIO,OUT,OD,S5=x,INIT=1'
#   'GPIO,IN,HL'
# zephyr/include/zephyr/driver/gpio.h
# GPIO_INPUT (1 << 16)
# GPIO_OUTPUT (1 << 17)
# GPIO_DISCONNECTED 0
# GPIO_OUTPUT_INIT_LOW (1 << 18)
# GPIO_OUTPUT_INIT_HIGH (1 << 19)
# GPIO_OUTPUT_INIT_LOGICAL (1 << 20) What does this mean?
#
# GPIO_OUTPUT_LOW (GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW)
# GPIO_OUTPUT_HIGH (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH)
# GPIO_OUTPUT_INACTIVE (GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_LOGICAL)
# GPIO_OUTPUT_ACTIVE (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOGICAL)
#
# GPIO_INT_DISABLE (1 << 21)
# GPIO_INT_ENABLE (1 << 22)
# GPIO_INT_LEVELS_LOGICAL (1 << 23)
# GPIO_INT_EDGE (1 << 24)
# GPIO_INT_LOW_0 (1 << 25)
# GPIO_INT_HIGH_1 (1 << 26)
# GPIO_INT_ENABLE_DISABLE_ONLY (1 << 27) disable/enable interrupt without changing
#                                        related regs such as pending
# GPIO_INT_MASK
# GPIO_INT_EDGE_RISING (GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_HIGH_1)
# GPIO_INT_EDGE_FALLING (GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_LOW_0)
# GPIO_INT_EDGE_BOTH (GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)
# GPIO_INT_LEVEL_LOW (GPIO_INT_ENABLE | GPIO_INT_LOW_0)
# GPIO_INT_LEVEL_HIGH (GPIO_INT_ENABLE | GPIO_INT_HIGH_1)
#

# flags_str contains comma separate attributes. Examples:
# 'ALT1,OD'
# 'GPIO,OUT,OD,S5=1,INIT=1'
# 'GPIO,OUT,PP,S5=0,INIT=0'
# 'GPIO,OUT,PP,S5=x,INIT=x'
# =0 or =1 means set the pin output value to this state
# =x means preserved current output state
# 'GPIO,IN,EE'
# 'GPIO,IN,HL'
# EE=enable both edge interrupt: GPIO_INT_EDGE_BOTH
# HL=high-to-low (falling edge) interrupt: GPIO_INT_EDGE_FALLING
# LH=low-to-high (rising edge interrupt: GPIO_INT_EDGE_RISING
#
# pm.yaml
# wakeup-source
#
# CroS added all the other GPIO flags to gpio_defines.h in their
# local binding directory.
# We need to do the same OR add another property to OR in these
# flags. Also check the size of the gpio_dt_spec flags member size.
# Is it big enough for the extra flags?
#
def build_gpio_flags(flags_str):
    #attrs = flags_str.split(',')
    flags = "GPIO_ACTIVE_HIGH"
    # flags = "GPIO_INPUT"
    if "OUT" in flags_str:
        if "INIT=1" in flags_str:
            flags = flags + " | GPIO_OUTPUT_HIGH"
        elif "INIT=0" in flags_str:
            flags = flags + " | GPIO_OUTPUT_LOW"
        else:
            flags = flags + " | GPIO_OUTPUT"
    elif "IN" in flags_str:
            flags = flags + " | GPIO_INPUT"

    if "PP" in flags_str:
        flags = flags + " | GPIO_PUSH_PULL"
    if "OD" in flags_str:
        flags = flags + " | GPIO_OPEN_DRAIN"

    if "PU" in flags_str:
        flags = flags + " | GPIO_PULL_UP"
    if "PD" in flags_str:
        flags = flags + " | GPIO_PULL_DOWN"

    if "EE" in flags_str:
        flags = flags + " | GPIO_INT_EDGE_BOTH"
    elif "HL" in flags_str:
        flags = flags + " | GPIO_INT_EDGE_FALLING"
    elif "LH" in flags_str:
        flags = flags + " | GPIO_INT_EDGE_RISING"

    if "|" in flags:
        flags = "(" + flags + ")"

    return flags

def build_s5_action(flags_str):
    if "S5=x" in flags_str:
        return "preserve"
    elif "S5=0" in flags_str:
        return "drive_low"
    elif "S5=1" in flags_str:
        return "drive_high"

    return None

# Dell GPIO_NUMBER(inst, chip, regno, bitno)
# #define nUSB_POWERSHARE_EN   GPIO_NUMBER(0,CHIP_EC,4,6)
# const dword _nUSB_POWERSHARE_EN   = nUSB_POWERSHARE_EN;
# #define nUSB_POWERSHARE_EN   GPIO_NUMBER(0,CHIP_EC,4,6)
#
def build_ec_gpio_dts(list_ec_gpios):
    # build device tree array of EC GPIOs
    print("ec-gpios {")
    print("\tcompatible = \"dell,nb-gpios\";\n")
    for item in list_ec_gpios:
        gp = item[0].split("/")
        gpio_name = gp[0]
        if "GPIO" not in gpio_name:
            continue
        gpio_bank_name = get_gpio_bank_name(gpio_name)
        gpio_bitpos = get_gpio_bitpos(gpio_name)
        gpio_flags = build_gpio_flags(item[2])
        s5_action = build_s5_action(item[2])
        
        pin_label = item[1].lower()
        if "#" in pin_label:
            pin_label = pin_label.replace("#", "")
            pin_label = "n_" + pin_label

        pin_name = pin_label.replace("_", "-")
        print("\t{0}: {1} {{".format(pin_label, pin_name))
        print("\t\tgpios = <&{0} {1} {2}>;".format(gpio_bank_name, gpio_bitpos, gpio_flags))
        if s5_action != None:
            print('\t\ts5-action = "{0}";'.format(s5_action))
        print("\t}}; /* {0} */\n".format(gpio_name))
    print("};")

def main():
    ### Generate Device Tree ec_gpios table from Dell legacy EC
    ### firmware GpioDef.txt
    ### This is a work-in-progress

    parser = argparse.ArgumentParser()
    parser.add_argument("--gpiodef", type=str, default="GpioDef.txt",
                        help="Dell GpioDef.txt file path")
    parser.add_argument("--soc", type=str, default="mec5200mlj",
                        help="MCHP SoC and package: mec5200mlj is default")

    args = parser.parse_args()
    infile_name = args.gpiodef
    chip_pinctrl_fname = args.soc + "-D0-pinctrl.dtsi"

    print("SoC = {0}".format(args.soc))
    print("SoC PINCTRL file = {0}".format(chip_pinctrl_fname))
    print("SoC GpioDef file = {0}".format(infile_name))

    headers = ["EC", "ECE_KSB", "PCH_VIRTUAL_WIRE" ]

    # GPIO = GPIO mode
    # ALT1, ALT2, ALT3, ALT4, ALT5 = alternate function
    # S5=0, S5=1, S5=x, INIT=0, INIT=1, INIT=x
    # INIT=0 initialize pin value to 0
    # INIT=1 initialize pin value to 1
    # INIT=x preserve pin state on initialization
    # S5=0 set pin state to 0 on S5 entry
    # S5=1 set pin state to 1 on S5 entry
    # S5=x preserve pin state on S5 entry
    # OD = open-drain
    # PP = push-pull
    # IN or INPUT = input direction
    # OUT or OUTPUT = output direction
    # PU = internal pull-up enabled
    # PD = internal pull-down enabled
    # EE = Interrupt enable both edges
    # HL = Interrupt enable, high level
    # LL = Interrupt enable, low level
    column2_properties = ["OD", "PP"]
    
    ws_re = re.compile(r"\s+")

    ec_items = []
    ece_ksb_items = []
    pch_vw_items = []

    state = 0
    with open(infile_name, mode='r') as fin:
        line_no = 1
        for line in fin:
            line = line.strip()
            # print("line {0} = {1}".format(line_no, line))
            if line == headers[0]:
                state = 1
                continue
            elif line == headers[1]:
                state = 2
                continue
            elif line == headers[2]:
                state = 3
                continue
            else:
                if line == "" or line[0:2] == "//":
                    continue
            
            if state == 1:
                # line1 = re.sub("\s+", ",", line)
                ec_items.append(line.split())
            elif state == 2:
                ece_ksb_items.append(line.split())
            elif state == 3:
                pch_vw_items.append(line.split())

            line_no = line_no + 1

    print("len(ec_gpios) = {0}".format(len(ec_items)))
    [print(item) for item in ec_items]

    print("")
    print("len(ec_ksb_gpios) = {0}".format(len(ece_ksb_items)))
    [print(item) for item in ece_ksb_items]

    print("")
    print("len(pch_vwires) = {0}".format(len(pch_vw_items)))
    [print(item) for item in pch_vw_items]

    ec_gpios = []
    ec_gpios_s5 = []
    ec_alt_funcs = []
    ec_ncs = []
    ec_nps = []
    for i in ec_items:
        if i[1] == "NC":
            ec_ncs.append(i)
            continue
        if i[1] == "NP":
            ec_nps.append(i)
            continue
        
        if "ALT" in i[2]:
            ec_alt_funcs.append(i)
        else:
            ec_gpios.append(i)

        if "S5" in i[2]:
            ec_gpios_s5.append(i)
                
    print("\nEC GPIOs")
    [print(item) for item in ec_gpios]
    print("\nEC GPIOS with S5 flag")
    [print(item) for item in ec_gpios_s5]
    print("\nEC ALT Funcs")
    [print(item) for item in ec_alt_funcs]
    print("\nEC No Connects: GPIO Input with internal pull-up enabled")
    [print(item) for item in ec_ncs]
    print("\nEC No Power pins")
    [print(item) for item in ec_nps]

    build_ec_gpio_dts(ec_gpios)

    print("Done")
# end main

if __name__ == '__main__':
    main()



