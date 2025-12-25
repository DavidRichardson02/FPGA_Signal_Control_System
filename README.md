# FPGA_Signal_Control_System

FPGA-based **spatial mapping and temperature control lab** implemented on a
Digilent **Nexys A7-100T**.  
The design combines:

- Time-of-Flight distance sensing
- Temperature + PIR-based fan control
- Real-time VGA HUD visualization
- High-speed UART telemetry to MATLAB for analysis and mapping

Everything runs in hardware in a **single 100 MHz clock domain** – no soft CPU.


flowchart LR
%% ============================================================================
%% PHYSICAL INPUTS
%% ============================================================================
subgraph ENV["Physical Signals"]
  TOF_HW["ToF Sensor\n(ISL29501)"]
  PIR_HW["PIR Motion"]
  ENC_HW["Rotary Encoder"]
  THERM_HW["Board / Ambient Temp"]
end

%% ============================================================================
%% LOW-LEVEL INTERFACES (100 MHz SYS)
%% ============================================================================
subgraph IFACE["Sensor Interfaces (sys_clk = 100 MHz)"]
  adxl["adxl362_if (optional accel)"]
  tof_if["isl29501_if\nI²C master + sequencer"]
  pir_if["pir_sync + debounce"]
  enc_if["rotary_encoder_if"]
  xadc_if["xadc_temp_reader"]
end

TOF_HW --> tof_if
PIR_HW --> pir_if
ENC_HW --> enc_if
THERM_HW --> xadc_if

%% ============================================================================
%% ACQUISITION + CDC
%% ============================================================================
subgraph CDC_SYS["Acquisition + CDC"]
  sample_valid["sample_valid strobes"]
  accel_cdc["accel_cdc_bridge"]
  sensor_cdc["sensor_cdc_bridge"]
end

tof_if --> sample_valid
pir_if --> sample_valid
enc_if --> sample_valid
xadc_if --> sample_valid

sample_valid --> sensor_cdc
sample_valid --> accel_cdc

%% ============================================================================
%% STATE + MAPPING (SYS DOMAIN)
%% ============================================================================
subgraph MAP_SYS["Mapping / State (100 MHz)"]
  angle_accum["angle_accumulator"]
  spatial_map["spatial_mapper"]
  map_filter["range_filter + validity gates"]
end

sensor_cdc --> angle_accum
sensor_cdc --> map_filter
angle_accum --> spatial_map
map_filter --> spatial_map

%% ============================================================================
%% CONTROL
%% ============================================================================
subgraph CTRL["Control Logic"]
  mode_fsm["survey_mode_fsm"]
  fan_ctrl["fan_pwm_controller"]
  safety["safety_interlocks"]
end

sensor_cdc --> mode_fsm
sensor_cdc --> fan_ctrl
safety --> fan_ctrl
mode_fsm --> fan_ctrl

%% ============================================================================
%% VGA PIPELINE (PIXEL DOMAIN)
%% ============================================================================
subgraph VGA["VGA Subsystem (pix_clk = 25 MHz)"]
  vga_timing["vga_timing_640x480"]
  range_plot["vga_range_plot"]
  status_ovl["vga_status_overlay"]
  compose["vga_frame_compositor"]
end

spatial_map --> range_plot
sensor_cdc --> status_ovl
vga_timing --> compose
range_plot --> compose
status_ovl --> compose

%% ============================================================================
%% TELEMETRY
%% ============================================================================
subgraph UART["Telemetry Output"]
  packetizer["mapper_packetizer"]
  uart_tx["uart_stream_tx"]
end

spatial_map --> packetizer
sensor_cdc --> packetizer
fan_ctrl --> packetizer
packetizer --> uart_tx

%% ============================================================================
%% PC-SIDE ANALYSIS
%% ============================================================================
subgraph PC["PC Capture & Analysis"]
  capture["UART Capture\n(TeraTerm / Python / MATLAB)"]
  parser["CSV / Binary Parser"]
  analysis["Analysis + Plots\n(calibration, error, maps)"]
end

uart_tx --> capture
capture --> parser
parser --> analysis
analysis --> packetizer



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



# FPGA Signal Control System — Project Walkthrough

<p align="center">
  <img 
       src="https://raw.githubusercontent.com/DavidRichardson02/FPGA_Signal_Control_System/main/docs/FPGA_Walkthrough_Cinematic.gif"
       width="800"
       alt="FPGA Signal Control System — Full Cinematic Walkthrough">
</p>

This cinematic walkthrough illustrates the complete end-to-end signal-processing,
control, and visualization pipeline of the FPGA Signal Control System.

### Walkthrough Sequence
1. **Hardware Setup** — Nexys A7, ToF sensor, PIR, fan PWM stage, rotary encoder.  
2. **Logic Analyzer** — Real I²C, IRQ, and PWM timing validation.  
3. **MATLAB Telemetry** — Decoding, logging, and visualization of UART frames.  
4. **VGA System** — Polar mapping, HUD overlays, temperature bars, fan logic tiles.  
5. **Showcase Demo** — Full system running live during competition.  
6. **Routing Map** — Post-implementation FPGA layout visualization.

