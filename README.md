# Event-Driven Telemetry and Live Blynk Dashboard (ESP32 + Wokwi)

A battery and relay monitor that sends data to a Blynk dashboard **only when something
meaningful happens**. While Wi-Fi or Blynk is down, events are stored in a fixed-size
queue and sent **in the original order** once the connection returns. Reconnection runs
as a non-blocking state machine, and Wi-Fi signal strength (RSSI) is monitored and used
to adapt the sending rate.

▶ **Run it live:** <https://wokwi.com/projects/477235424791559169> <https://blynk.cloud/dashboard/773467/global/devices/308601/organization/773467/devices/2317011/dashboard>
🎥 **Demo video:** <https://lnkd.in/p/gpW9E9df>



## Features
- **Event-driven:** a cell voltage is reported only after it moves 50 mV; state, relay and link changes are reported when they happen; an idle system sends one slow heartbeat every 5 minutes
- **Fixed-size offline queue:** 32 records of 24 bytes (768 bytes), no dynamic memory
- **In-order delivery:** every record has a sequence number and is sent oldest first
- **Overflow policy:** when full, the oldest non-critical record is dropped first; fault, relay and network records are kept; a `GAP` marker shows where data is missing
- **Non-blocking Wi-Fi state machine** with exponential backoff (1 s up to 30 s)
- **RSSI monitoring:** smoothed, classified (EXCELLENT to BAD) with hysteresis; a poor link slows telemetry
- **Live vs queued is always visible:** every journal line is tagged `[LIVE]` or `[Q -42s]`, plus a mode label and a queue-depth gauge
- **Paced for Blynk:** at most 8 writes per second (Blynk's guideline is 10)

## Files
| File | Purpose |
|---|---|
| `sketch.ino` | Firmware |
| `diagram.json` | Wokwi circuit |
| `libraries.txt` | Wokwi libraries (`Blynk`, `LiquidCrystal I2C`) |
| `REPORT.md` | Design report |
| `BLYNK_SETUP.md` | Step-by-step Blynk setup for beginners |
| `LICENSE` | MIT license |

## Hardware (Wokwi)
| Part | Pin | Role |
|---|---|---|
| Pot 1, 2, 3 | GPIO 34, 35, 32 | Battery cells 1 to 3 (2.8 to 4.4 V) |
| Pot 4 | GPIO 33 | Signal attenuation, 0 to 45 dB (simulated RSSI drop) |
| Relay module (active LOW) | IN GPIO 19 | Opens on a fault |
| I2C LCD 16x2 | SDA 21, SCL 22 | State, queue depth, mode, Wi-Fi health |
| Green / yellow / red LED | GPIO 4 / 5 / 2 | Live / replaying or poor link / offline |
| Buzzer | GPIO 18 | Pulses during a fault |
| Button INJECT | GPIO 25 | Cycle the injected fault |
| Button OUTAGE | GPIO 26 | Start or end a simulated network outage |
| Button BURST | GPIO 27 | Flood the queue to show overflow handling |

## Blynk setup (about 10 minutes)
**1. Create a template** in Blynk Console named `BMS Telemetry`, hardware ESP32, connection Wi-Fi.

**2. Add 10 datastreams.** Check your plan's limit first: older free accounts may allow
only 5 to 10 datastreams per template, newer free accounts allow 20 but cap messages at
30,000 per month.

| Pin | Name | Type | Notes |
|---|---|---|---|
| V0 | Cell 1 | Double | 2.8 to 4.4 V |
| V1 | Cell 2 | Double | 2.8 to 4.4 V |
| V2 | Cell 3 | Double | 2.8 to 4.4 V |
| V3 | Extremes | String | Weakest and strongest cell |
| V4 | Relay | Integer | 0 to 1 |
| V5 | System state | String | NORMAL, WARNING or FAULT with the cause |
| V6 | Wi-Fi health | String | Class and RSSI |
| V7 | Queue depth | Integer | 0 to 32 |
| V8 | Data mode | String | LIVE or REPLAYING |
| V9 | Event journal | String | Ordered event log |

**3. Create a device** from the template and copy its Template ID, Template Name and Auth
Token into the three `#define` lines at the top of `sketch.ino`.

**4. Build the dashboard** (web or mobile):

| Widget | Pin | Shows |
|---|---|---|
| 3 gauges | V0, V1, V2 | Real-time cell voltages |
| Labeled Value | V3 | Weakest and strongest cell |
| LED | V4 | Relay closed or open |
| Labeled Value | V5 | Fault state |
| Labeled Value | V6 | Wi-Fi health |
| Gauge or Labeled Value | V7 | Offline queue depth |
| Labeled Value | V8 | LIVE or REPLAYING |
| Terminal | V9 | Event journal |

## How to run
1. Fill in the Blynk credentials in `sketch.ino`.
2. Paste `sketch.ino`, `diagram.json` and `libraries.txt` into a Wokwi ESP32 project and press Play.
3. Open the Serial Monitor (115200 baud) next to your Blynk dashboard.

## What gets sent, and when
| Event | Triggered by |
|---|---|
| BOOT | Start-up |
| CELLS | Any cell moved 50 mV since the last report (rapid changes are merged) |
| EXTREMES | Weakest or strongest cell changed (with 20 mV hysteresis) |
| STATE | Fault state or cause changed |
| RELAY | Relay opened or closed |
| LINK | Wi-Fi quality class changed |
| NET_DOWN / NET_UP | Connection lost / restored, with the outage length |
| HEARTBEAT | Nothing else for 5 minutes |

Each event is a snapshot of all cell values and states at that moment, so a replayed
record shows what was true when it happened.

## Live vs queued on the dashboard
- **Gauges and labels (V0 to V8)** always show the latest values. They update live while online, and after a reconnect they refresh straight away.
- **Event journal (V9)** shows what happened, in order. Each line says `[LIVE]` when sent within 1.5 s of the event, or `[Q -42s]` when it was held back and is now being replayed, with its age.
- **Data mode (V8)** shows `REPLAYING` while old records are still draining, and `LIVE` once caught up. **Queue depth (V7)** counts those old records.
- During an outage nothing can be sent, so Blynk shows the device as offline and the gauges stay frozen. The LCD and red LED show the outage locally.

## Demo script
**Fault injection**
1. Turn the cell pots to about the same value: green LED, `NORMAL`.
2. Press INJECT once (CELL_DROP): within a second the journal shows `STATE FAULT CELL_UV`, then `RELAY OPEN`; the relay LED goes off and the buzzer pulses.
3. Press INJECT until NONE: the relay re-closes 3 s later and the journal shows the recovery.
4. INJECT also offers IMBALANCE and RELAY_FAULT.

**Network outage**
1. Press OUTAGE. The LCD shows `SIM OUTAGE ON`, the red LED lights and Blynk shows the device offline.
2. During the outage, move the cell pots, press INJECT (a fault and the relay opening), then press INJECT back to NONE. The LCD queue counter `Q:` climbs.
3. Press OUTAGE again. The state machine reconnects (see `NET` lines in the Serial Monitor), the mode switches to `REPLAYING`, and the journal fills with `[Q -18s]`, `[Q -14s]` records in sequence order, ending with `NET UP after 17s` and `[LIVE]` lines.
4. Press BURST during an outage to flood the queue: it stays at 32, non-critical records are dropped first, and a `GAP` marker appears in the journal.

**Signal quality:** turn Pot 4 up. The health label moves from EXCELLENT to BAD, and a poor link slows telemetry and merges more cell updates.

## Test results
Tested on a PC with simulated Wi-Fi and Blynk (not on Wokwi or real Blynk):
- 120 s with no change: 0 messages sent
- A 0.4 V cell change: 5 messages in total (3 gauge updates plus 2 journal records)
- Outage with a fault, relay change and cell changes: nothing sent while offline, then all records replayed in sequence order, tagged queued, ending back at LIVE with depth 0
- Network gone for 100 s: retry gaps of 12, 14, 18, 26 s (backoff), and automatic recovery when it returned
- Blynk server down while Wi-Fi stayed up: detected, and reconnected when the server returned
- Queue flood: stayed at 32 records, critical relay records survived, GAP markers placed
- RSSI drop to -90 dBm: classified BAD with one link event, and recovered with hysteresis
- 30 minutes of random outages, faults, floods and signal changes (4 random seeds): records always in order, every non-dropped record delivered, never more than 8 messages in any second, no writes while offline, no stalls

## Limitations
- Wokwi reports a fixed RSSI, so the sketch **simulates** RSSI from the attenuation pot (`RSSI_SIMULATED`). Set it to `false` on real hardware
- The Blynk library's `connect()` call is blocking, capped at 5 s per attempt and only tried after Wi-Fi is up, with backoff between attempts. The Wi-Fi side is fully non-blocking
- Delivery is at-least-once: a record sent just before a disconnect may be sent again after the reconnect. The sequence number lets a reader remove duplicates
- Blynk's free plans limit datastreams and messages; check your plan
- I have not run this on Wokwi or against a real Blynk account, so Wokwi's Wi-Fi and the Blynk library calls are the likeliest places for surprises


