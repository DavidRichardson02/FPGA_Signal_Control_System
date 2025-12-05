# FPGA_Signal_Control_System

FPGA-based **spatial mapping and temperature control lab** implemented on a
Digilent **Nexys A7-100T**.  
The design combines:

- Time-of-Flight distance sensing
- Temperature + PIR-based fan control
- Real-time VGA HUD visualization
- High-speed UART telemetry to MATLAB for analysis and mapping

Everything runs in hardware in a **single 100 MHz clock domain** – no soft CPU.

---

## Project Overview

This project started as an ECE 375 “Introduction to Computer Architecture”
final project and evolved into a full **mixed-signal physics control
laboratory** on an FPGA.

The system implements three tightly-coupled subsystems:

1. **Sensing**
   - Pmod ToF (ISL29501) distance sensor over I²C
   - On-chip XADC temperature monitoring (Q1.15 fixed-point)
   - PIR motion sensor (warm-up + hold-time conditioning)
   - Rotary encoder / surveyor input (manual and automatic sweep)

2. **Control**
   - Temperature + PIR + manual **fan controller FSM**
   - PWM fan drive with hysteresis and contribution flags
   - Unified Q1.15 representation for temperature, duty cycle, and angle
   - Ready/valid style interfaces between blocks

3. **Visualization & Telemetry**
   - 640×480 VGA pipeline with:
     - ToF range plot in the left half of the screen
     - Telemetry HUD on the right (temperature bar, fan tiles, PIR activity,
       UART counters, rotary spinner bar, etc.)
     - 320×240 **double-buffered logo/graphics viewport**
   - Mapper packetizer + UART stream at **2 Mbit/s**
   - MATLAB tools for live decoding, plotting, and logo/image streaming

---

## Key Features

- **Single-clock architecture**  
  All RTL runs from the 100 MHz board oscillator. Slow behavior (1 s ticks,
  ToF cadence, UART baud, debounce windows) is derived from tick-enables, not
  generated clocks.

- **ToF Surveyor + Angle Indexer**
  - Surveyor FSM generates step pulses and directions.
  - Angle indexer maintains both an integer index and `theta_q15`
    (Q1.15 turns → 0..1 mapped to 0..360°).

- **I²C Time-of-Flight Engine**
  - Self-contained `tof_sensor` module uses a generic `i2c_master`
    to configure the ISL29501 and read distance + status.
  - IRQ + timeout handling and error/status aggregation.

- **Fan Control Path**
  - XADC → `xadc_sampler` → `temp_fan_ctrl` → `pwm_dac` → MOSFET → fan.
  - Three contributions: temperature, PIR occupancy, manual override.
  - Per-source contribution flags + LEDs and HUD tiles.

- **VGA Range Plot + HUD**
  - `vga_range_plot_top` generates timing, framebuffer readout, and
    a 256×256 bit-plane ToF map in the left half of the display.
  - `vga_status_overlay` draws the right-panel HUD, temperature bar,
    fan tiles, PIR “motion streak”, UART counters, angle indicator, and
    rotary spinner bar.
  - `image_dualbuf_320x240_rgb12` stores two logo frames and swaps
    at frame boundaries without tearing.

- **Telemetry & MATLAB Integration**
  - `mapper_packetizer` frames each sample as:

    ```text
    0x55 0xAA |
    timestamp[31:0] |
    theta_q15[15:0] |
    dist_mm[15:0] |
    temp_q15[15:0] |
    duty_q15[15:0] |
    status[7:0] |
    crc16[15:0]
    ```

  - MATLAB scripts:
    - resynchronize on the 0x55 0xAA header
    - verify CRC
    - plot distance vs time and polar `(r, θ)` maps
    - export CSV logs
    - convert arbitrary images → 320×240 RGB444 → `.hex` files or UART streams
      for the logo framebuffer

---

## Repository Layout

```text
rtl/        # Verilog RTL: top-level, sensor/actuator interfaces, VGA, packetizer
constr/     # XDC pin constraints for Nexys A7-100T
scripts/    # Vivado Tcl scripts (optional, for project regeneration)
matlab/     # Telemetry decoder, plotting tools, logo conversion utilities
docs/       # Report, poster, cheat sheet, and supplementary documentation
images/     # Photos and screenshots used in the README
vivado/     # (Optional) Vivado project files; can be regenerated from scripts
