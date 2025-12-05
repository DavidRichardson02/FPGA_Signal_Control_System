################################################################################
# File    : nexys_a7_pinout.xdc
# Board   : Digilent Nexys A7-100T (XC7A100T-1CSG324C)
# Project : FPGA_Spatial_Mapping_Temperature_Control_Project
#
# Role of this constraints file
# ------------------------------------------------------------------------------
#   Bind the top-level RTL ports of spatial_mapping_temperature_control_top
#   to the physical pins of the Nexys A7-100T board, and define the
#   corresponding electrical properties.
#
# High-level groups:
#   - System clock (100 MHz).
#   - Human interface: buttons, seven-segment display, user LEDs, RGB LEDs.
#   - UART link to FT2232 USB-UART bridge.
#   - PWM “DAC” output for analog observability.
#   - XADC VAUX differential input (temperature/analog path).
#   - Fan control outputs + PIR motion input.
#   - I²C open-drain SCL/SDA to Pmod ToF (ISL29501).
#   - Front-panel slide switches for fan/survey enable masks.
#   - ToF IRQ_n and SS pins.
#   - On-board temperature sensor I²C + interrupts.
#   - Rotary encoder Pmod JC.
#   - Configuration bank voltage.
################################################################################


################################################################################
# System Clock (100 MHz from on-board oscillator)
################################################################################
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports clk_100MHz]

create_clock -period 10.000 -name clk_100MHz -waveform {0.000 5.000} \
    [get_ports clk_100MHz]


################################################################################
# Pushbuttons (active-HIGH)
################################################################################
# btnc             : Center button → global reset (debounced in RTL).
# btn_survey_start : Start survey.
# btn_survey_stop  : Stop survey.
# btnd             : Manual fan toggle.
################################################################################
set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33 }               [get_ports btnc]
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVCMOS33 }               [get_ports btn_survey_start]
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 }               [get_ports btn_survey_stop]
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports btnd]


################################################################################
# Seven-Segment Display (ACTIVE-LOW common-anode)
################################################################################
# segment[6:0] : A..G  (active-LOW)
# anode[7:0]   : digit enables (anode[0] rightmost ... anode[7] leftmost)
# dp           : decimal point (active-LOW)
################################################################################
# Segment outputs A..G (ACTIVE-LOW)
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { segment[0] }] ; # A
set_property -dict { PACKAGE_PIN R10 IOSTANDARD LVCMOS33 } [get_ports { segment[1] }] ; # B
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33 } [get_ports { segment[2] }] ; # C
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports { segment[3] }] ; # D
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { segment[4] }] ; # E
set_property -dict { PACKAGE_PIN T11 IOSTANDARD LVCMOS33 } [get_ports { segment[5] }] ; # F
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { segment[6] }] ; # G

# Digit anodes (ACTIVE-LOW), anode[0] = rightmost … anode[7] = leftmost
set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS33 } [get_ports { anode[0] }]
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 } [get_ports { anode[1] }]
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { anode[2] }]
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports { anode[3] }]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { anode[4] }]
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 } [get_ports { anode[5] }]
set_property -dict { PACKAGE_PIN K2  IOSTANDARD LVCMOS33 } [get_ports { anode[6] }]
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33 } [get_ports { anode[7] }]

# Decimal point (ACTIVE-LOW)
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports dp]


################################################################################
# User LEDs (single-color)
################################################################################
# led0..led7  : Lower LED row (UART activity etc.).
# led9..led15 : Upper LED row (fan/PIR/debug).
################################################################################
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { led0  }]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { led1  }]
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { led2  }]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { led3  }]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { led4  }]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { led5  }]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { led6  }]
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { led7  }]
# LED[8] is intentionally unused
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports { led9  }]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { led10 }]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { led11 }]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { led12 }]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports { led13 }]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { led14 }]
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33 } [get_ports { led15 }]


################################################################################
# RGB LEDs
################################################################################
# led0_r/g/b : RGB LED0 channels.
# led1_r/g/b : RGB LED1 channels.
################################################################################
set_property -dict { PACKAGE_PIN R12 IOSTANDARD LVCMOS33 } [get_ports { led0_b }]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { led0_g }]
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports { led0_r }]

set_property -dict { PACKAGE_PIN G14 IOSTANDARD LVCMOS33 } [get_ports { led1_b }]
set_property -dict { PACKAGE_PIN R11 IOSTANDARD LVCMOS33 } [get_ports { led1_g }]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports { led1_r }]


################################################################################
# UART TX to on-board FT2232 USB-UART (J6)
################################################################################
set_property -dict { PACKAGE_PIN D4 IOSTANDARD LVCMOS33 } [get_ports uart_txo]
set_property SLEW SLOW [get_ports uart_txo]

set_property PACKAGE_PIN C17           [get_ports { uart_txo_debug }]
set_property IOSTANDARD LVCMOS33      [get_ports { uart_txo_debug }]
set_property SLEW SLOW                [get_ports { uart_txo_debug }]


################################################################################
# PWM "DAC" Output
################################################################################
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports pwm_out]
set_property SLEW SLOW [get_ports pwm_out]


################################################################################
# XADC VAUX Differential Input
################################################################################
# Using VAUX3 pair: vauxp_in (B16), vauxn_in (B17)
################################################################################
set_property PACKAGE_PIN B16 [get_ports { vauxp_in }]   ; # XA_P[3]
set_property PACKAGE_PIN B17 [get_ports { vauxn_in }]   ; # XA_N[3]
set_property IOSTANDARD LVCMOS33 [get_ports { vauxp_in vauxn_in }]


################################################################################
# Fan Control Outputs (JA3 / JA4)
################################################################################
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports fan_pwm_ja3] ; # JA3
set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 } [get_ports fan_en_ja4 ] ; # JA4
set_property SLEW SLOW [get_ports { fan_pwm_ja3 fan_en_ja4 }]


################################################################################
# PIR Motion Input (HC-SR501 OUT → FPGA pin)
################################################################################
set_property -dict { PACKAGE_PIN D17 IOSTANDARD LVCMOS33 PULLDOWN TRUE } [get_ports pir_raw]


################################################################################
# I²C Open-Drain (SCL / SDA) to Pmod ToF
################################################################################
set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS33 } [get_ports { SCL }] ; # jb[3]
set_property -dict { PACKAGE_PIN H14 IOSTANDARD LVCMOS33 } [get_ports { SDA }] ; # jb[4]

set_property PULLUP TRUE [get_ports { SCL SDA }]
set_property SLEW  SLOW  [get_ports { SCL SDA }]


################################################################################
# Front-Panel Slide Switches
################################################################################
set_property -dict { PACKAGE_PIN U12 IOSTANDARD LVCMOS33 } [get_ports { sw_temp_en      }]
set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { sw_manual_en    }]
set_property -dict { PACKAGE_PIN V10 IOSTANDARD LVCMOS33 } [get_ports { sw_pir_en       }]
set_property -dict { PACKAGE_PIN H6  IOSTANDARD LVCMOS33 } [get_ports { sw_dbg_mode     }]
set_property -dict { PACKAGE_PIN T13 IOSTANDARD LVCMOS33 } [get_ports { sw_survey_manual}]
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 } [get_ports { sw_logo_sel}]

################################################################################
# ToF Sensor Control / Status (Pmod ToF ISL29501)
################################################################################
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { tof_irq_n }]
set_property PULLUP TRUE [get_ports { tof_irq_n }]

set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 } [get_ports { tof_ss }]
set_property DRIVE 8   [get_ports { tof_ss }]
set_property SLEW  SLOW [get_ports { tof_ss }]


################################################################################
# VGA Connector
################################################################################
set_property -dict { PACKAGE_PIN A3  IOSTANDARD LVCMOS33 } [get_ports { vga_red[0]  }]
set_property -dict { PACKAGE_PIN B4  IOSTANDARD LVCMOS33 } [get_ports { vga_red[1]  }]
set_property -dict { PACKAGE_PIN C5  IOSTANDARD LVCMOS33 } [get_ports { vga_red[2]  }]
set_property -dict { PACKAGE_PIN A4  IOSTANDARD LVCMOS33 } [get_ports { vga_red[3]  }]

set_property -dict { PACKAGE_PIN C6  IOSTANDARD LVCMOS33 } [get_ports { vga_grn[0]  }]
set_property -dict { PACKAGE_PIN A5  IOSTANDARD LVCMOS33 } [get_ports { vga_grn[1]  }]
set_property -dict { PACKAGE_PIN B6  IOSTANDARD LVCMOS33 } [get_ports { vga_grn[2]  }]
set_property -dict { PACKAGE_PIN A6  IOSTANDARD LVCMOS33 } [get_ports { vga_grn[3]  }]

set_property -dict { PACKAGE_PIN B7  IOSTANDARD LVCMOS33 } [get_ports { vga_blu[0]  }]
set_property -dict { PACKAGE_PIN C7  IOSTANDARD LVCMOS33 } [get_ports { vga_blu[1]  }]
set_property -dict { PACKAGE_PIN D7  IOSTANDARD LVCMOS33 } [get_ports { vga_blu[2]  }]
set_property -dict { PACKAGE_PIN D8  IOSTANDARD LVCMOS33 } [get_ports { vga_blu[3]  }]

set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports { vga_hsync_n }]
set_property -dict { PACKAGE_PIN B12 IOSTANDARD LVCMOS33 } [get_ports { vga_vsync_n }]


################################################################################
# On-board Temperature Sensor (TMP)
################################################################################
set_property -dict { PACKAGE_PIN C14 IOSTANDARD LVCMOS33 } [get_ports { TMP_SCL }] ; # tmp_scl
set_property -dict { PACKAGE_PIN C15 IOSTANDARD LVCMOS33 } [get_ports { TMP_SDA }] ; # tmp_sda
set_property -dict { PACKAGE_PIN D13 IOSTANDARD LVCMOS33 } [get_ports { TMP_INT }] ; # tmp_int
set_property -dict { PACKAGE_PIN B14 IOSTANDARD LVCMOS33 } [get_ports { TMP_CT  }] ; # tmp_ct


################################################################################
# Pmod Header JC (Rotary Encoder)
################################################################################
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { jc_rot_a }] ; # jc[1]
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { jc_rot_b }] ; # jc[2]
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { jc_rot_sw}] ; # jc[3]


################################################################################
# Configuration Bank Voltage (Bank 0)
################################################################################
set_property CFGBVS         VCCO  [current_design]
set_property CONFIG_VOLTAGE 3.3  [current_design]
