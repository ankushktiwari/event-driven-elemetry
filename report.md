# Modular Battery Management System: Project Report

## 1. Objective
Design a modular Battery Management System (BMS) engine for an ESP32, simulated in Wokwi, that:
- works on an array of cells whose count is set by one compile-time constant;
- continuously finds the weakest and strongest cell and calculates the voltage imbalance;
- tracks whether the imbalance is increasing or decreasing;
- uses adaptive imbalance thresholds based on State of Charge (SoC) instead of a fixed limit;
- exposes all battery information through one interface that later tasks can reuse;
- explains how the design scales from 4 to 16 cells in memory, execution time and data structures.

## 2. Hardware (Wokwi)
| Part | Pin | Purpose |
|---|---|---|
| ESP32 DevKit | n/a | Controller |
| 4 potentiometers | GPIO 34, 35, 32, 33 | Cell voltages, 0 to 3.3 V |
| I2C LCD 16x2 | SDA 21, SCL 22 | Local display |
| Red / green / yellow LED | GPIO 2 / 4 / 5 | Fault / normal / warning |
| Buzzer | GPIO 18 | Audible alert |
| Relay module (active LOW) | GPIO 19 | Load switch: LOW = connected, HIGH = cut off |

The battery cells are potentiometers, so a cell voltage can be changed by hand while the simulation runs.

## 3. Software design

### 3.1 Scalable cell count
```cpp
#define NUM_CELLS 4
#define REAL_CELLS 4
```
Every array and loop is sized from `NUM_CELLS`, so changing that one constant resizes the whole engine. Cells beyond the real potentiometers are simulated from the average of the real cells plus a small fixed offset, so a 16-cell test needs no extra wiring.

### 3.2 One interface for all battery data
```cpp
struct BmsData {
  float v[NUM_CELLS];     // cell voltages
  int   pct[NUM_CELLS];   // cell charge %
  int   weakest, strongest;
  int   avgPct;           // pack SoC %
  float imbalance;        // strongest minus weakest, in volts
  float threshold;        // adaptive limit, in volts
  char  trend;            // 'U' rising, 'D' falling, 'S' steady
};
void analyzeBms();        // reads the cells and fills BmsData
```
The decision logic, the LCD and the Serial output only read this structure. A later task can reuse the analysis by calling `analyzeBms()` and reading the struct, without copying any logic.

### 3.3 Analysis (once per second)
1. **Read:** each cell voltage and its percentage (0 to 3.3 V maps to 0 to 100%).
2. **Weakest and strongest:** a single pass over the cells keeps the index of the minimum and maximum voltage.
3. **Imbalance:** strongest voltage minus weakest voltage.
4. **Trend:** the imbalance is compared with the previous second. A change above +0.02 V is increasing, below -0.02 V is decreasing, otherwise steady. The 0.02 V dead-band stops the trend flickering on small changes.

### 3.4 Adaptive threshold
The allowed imbalance depends on the pack's SoC:

`threshold = 0.10 + 0.20 x (100 - SoC) / 100` volts

| SoC | Threshold |
|---|---|
| 100% | 0.10 V |
| 50% | 0.20 V |
| 0% | 0.30 V |

A battery's voltage curve is steeper near empty, so the same charge difference shows up as a larger voltage difference. A tight limit when full and a looser limit when low reduces false alarms compared with one fixed number. (Only SoC is used, because the project has no current sensor.)

### 3.5 Decision logic (priority order)
| Condition | Status | Outputs |
|---|---|---|
| Any cell under 30% | WEAK CELL | Red LED, buzzer, relay cut |
| All cells over 80% | HIGH VOLT | Yellow LED, buzzer, relay cut |
| Imbalance above the adaptive threshold | IMBALANCE | Red and yellow LEDs, buzzer; relay cut only if the trend is increasing |
| Otherwise | NORMAL | Green LED, relay connected |

### 3.6 Display
- **Serial Monitor:** every cell voltage, SoC, weakest and strongest cell, imbalance, threshold, trend and status.
- **LCD:** three pages that change every second: (1) the four cell voltages, (2) weakest and strongest cell, trend, imbalance and threshold, (3) status and SoC.

## 4. Scaling from 4 to 16 cells
The figures below are calculated or estimated. They are not measured on hardware.

### 4.1 Memory
On the ESP32 an `int` and a `float` are both 4 bytes, so `BmsData` uses 8 bytes per cell plus about 24 bytes of fixed fields.

| | 4 cells | 16 cells |
|---|---|---|
| Per-cell arrays (`v`, `pct`) | 32 B | 128 B |
| Fixed fields (indices, SoC, imbalance, threshold, trend) | about 24 B | about 24 B |
| **Total `BmsData`** | **about 56 B** | **about 152 B** |

Memory grows linearly. Even at 16 cells the data is far below 1% of the ESP32's RAM. If memory ever mattered, voltages could be stored as 16-bit millivolts and percentages as 8-bit values, about 3 bytes per cell instead of 8.

### 4.2 Execution time
- **Algorithm:** one pass over the cells finds the weakest, strongest and sum together, so the analysis is **O(N)**. The threshold, SoC and trend calculations run once per cycle, not per cell, so they do not grow with N.
- **Estimated cost:** at 240 MHz the arithmetic for 16 cells should take microseconds. The `analogRead` calls (tens of microseconds each) dominate, so reading 16 real cells takes about four times as long as reading 4.
- **Simulated cells:** each re-reads the real pots to compute their average. This is a simulation shortcut and would not exist on real hardware.
- **Serial output** grows linearly with N, at roughly 1 ms per cell at 115200 baud.
- The loop ends with `delay(1000)`, so the analysis time does not change the update rate.

### 4.3 Data structures
- Fixed-size arrays sized at compile time: no dynamic memory and no fragmentation.
- Parallel arrays for per-cell data, and one struct for every derived result.
- The weakest and strongest cells are stored as indices, not copied data.
- One struct and one function form the interface, so the model grows by changing one constant.

### 4.4 What changes at 16 cells
- **LCD:** it shows only C1 to C4, so 16 cells need more pages or a paging loop.
- **Inputs:** real cell readings would come from an analog multiplexer or a battery monitor IC, since the ESP32 has few ADC pins.
- **Logic:** none of the analysis code changes.

## 5. Test scenarios in Wokwi
1. All pots equal and mid-range: NORMAL, green LED, relay connected.
2. One pot well away from the others: IMBALANCE; the relay cuts off if the trend is increasing.
3. Any pot under 30%: WEAK CELL.
4. All pots over 80%: HIGH VOLT.
5. Move a pot back toward the others: the trend shows decreasing and the status returns to NORMAL.
6. Lower all pots together: the threshold rises as SoC falls (LCD page 2).

This project was confirmed working in Wokwi by its author. The sketch has no automated tests, unlike the later projects.

## 6. Limitations and future work
- The threshold uses SoC only; a discharge-rate input (a current sensor) would add a second adaptive term.
- SoC is a linear voltage estimate, not a real Li-ion curve.
- Potentiometers hold a fixed voltage, so a running load is not modelled.
- The BMS detects imbalance but does not balance cells.
- Possible additions: a relay reset button, data logging, current sensing.

## 7. Conclusion
The project meets the requirements. It finds the weakest and strongest cells, calculates the imbalance and its trend, adapts the threshold to SoC, and exposes everything through one struct and one function. The cell count is a single constant, and the design stays small at 16 cells.
