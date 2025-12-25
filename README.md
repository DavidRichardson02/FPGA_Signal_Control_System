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
  %% =========================
  %% STAGE 0: PHYSICAL WORLD
  %% =========================
  subgraph P0["Physical Environment"]
    TGT["Scene / Target Geometry"]
    OCC["Occupancy / Motion (People)"]
    THERM["Thermal State (Board / Ambient)"]
  end

  %% =========================
  %% STAGE 1: SENSING
  %% =========================
  subgraph S1["Sensing Layer"]
    TOF["ToF Distance Sensor (ISL29501)\nI²C samples: range + quality"]
    PIR["PIR Motion Detector\nDigital occupancy"]
    XADC["FPGA XADC\nDie temp / supply telemetry"]
    ENC["Rotary Encoder\nAngle / sweep control"]
  end

  TGT --> TOF
  OCC --> PIR
  THERM --> XADC
  TGT --> ENC

  %% =========================
  %% STAGE 2: FPGA ACQUISITION + CDC
  %% =========================
  subgraph F2["FPGA Acquisition + Clock-Domain Management"]
    I2CIF["I²C Master + Register Sequencer\n(init, calibrate, read)"]
    DEGLITCH["Debounce / De-glitch\n(PIR + Encoder)"]
    CDC["CDC Bridges\n100 MHz sys ↔ 25 MHz pix"]
    TS["Timestamping / Sample Valid Strobes"]
  end

  TOF --> I2CIF
  PIR --> DEGLITCH
  ENC --> DEGLITCH
  XADC --> TS
  I2CIF --> TS
  DEGLITCH --> TS
  TS --> CDC

  %% =========================
  %% STAGE 3: STATE ESTIMATION / MAPPING
  %% =========================
  subgraph F3["FPGA State + Mapping"]
    ANG["Angle Integrator\n(encoder → θ)"]
    MAP["Spatial Mapper\n(θ, range) → bins / pixels"]
    FILTER["Filtering\n(median/EMA/outlier reject)"]
    QUAL["Quality Gates\n(valid range / saturation / timeouts)"]
  end

  CDC --> ANG
  CDC --> QUAL
  QUAL --> FILTER
  FILTER --> MAP
  ANG --> MAP

  %% =========================
  %% STAGE 4: CONTROL
  %% =========================
  subgraph F4["FPGA Control Layer"]
    MODE["Mode FSM\n(auto survey / manual / hold)"]
    FAN["Fan Controller\nPWM + hysteresis + override"]
    SAFE["Safety Interlocks\n(temp limits / sensor fault)"]
  end

  MAP --> MODE
  CDC --> MODE
  CDC --> FAN
  CDC --> SAFE
  SAFE --> FAN
  MODE --> FAN

  %% =========================
  %% STAGE 5: PRESENTATION (VGA HUD)
  %% =========================
  subgraph V5["Real-Time VGA Visualization (25 MHz Pixel Domain)"]
    VGA["VGA Timing + Raster\n640×480@60"]
    HUD["HUD Overlay\nwidgets: range plot, temp, occupancy, θ"]
    FRAME["Frame Composition\n(background map + overlays)"]
  end

  MAP --> FRAME
  CDC --> HUD
  VGA --> FRAME
  HUD --> FRAME

  %% =========================
  %% STAGE 6: TELEMETRY OUT (UART)
  %% =========================
  subgraph U6["Telemetry + Logging"]
    PKT["Packetizer\n(frames: sync, id, payload, CRC)"]
    UART["UART Stream TX\n(binary or CSV-like)"]
  end

  MAP --> PKT
  CDC --> PKT
  FAN --> PKT
  PKT --> UART

  %% =========================
  %% STAGE 7: PC ANALYSIS PIPELINE
  %% =========================
  subgraph PC7["PC Side: Capture → Parse → Analyze → Report"]
    CAP["Serial Capture\n(TeraTerm / Python / MATLAB)"]
    PARSE["Parser / Decoder\n(sync, fields, scaling)"]
    CLEAN["Cleaning\n(dropouts, resample, align clocks)"]
    ANALYZE["Analysis\ncalibration fits, error metrics, plots"]
    REPORT["Artifacts\nCSV, figures, LaTeX/PDF notes"]
  end

  UART --> CAP
  CAP --> PARSE
  PARSE --> CLEAN
  CLEAN --> ANALYZE
  ANALYZE --> REPORT

  %% =========================
  %% STAGE 8: FEEDBACK LOOP (DESIGN ITERATION)
  %% =========================
  subgraph FB8["Engineering Feedback Loop"]
    CAL["Calibration Updates\n(mapping curves, offsets, LUTs)"]
    TUNE["Control Tuning\nthresholds, hysteresis, modes"]
    RTL["RTL Revisions\nfilters, CDC, packet format, HUD"]
    CONSTR["Constraints + Implementation\nXDC, timing closure"]
  end

  ANALYZE --> CAL
  ANALYZE --> TUNE
  CAL --> RTL
  TUNE --> RTL
  RTL --> CONSTR
  CONSTR --> F2
  CONSTR --> F3
  CONSTR --> F4
  CONSTR --> V5
  CONSTR --> U6





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

