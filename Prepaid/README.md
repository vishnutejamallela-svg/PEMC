# PREPAY_V0 Modbus energy bridge

This Keil C51/uVision firmware for the MS51PC0AE polls the energy meter on
physical RS-485 port 2 and streams the four received register values as raw
bytes on physical RS-485 port 1 for Docklight inspection.

## Modbus settings

- Energy meter: slave ID `4`, `9600 baud`, `8E1`.
- Port 1 debug output: `9600 baud`, `8E1`.
- Meter poll starts at zero-based protocol offset `0` and requests 4 registers.
- The meter values are output once per second only after a valid response.
- Port 1 emits 8 bytes per update, register 0 high byte first through register 3
  low byte. No Modbus header, CRC, or scaling is added.
- Port 1 is dedicated to debug output in this mode; PC Modbus slave service is
  disabled. No relay, LCD, or I²C features are used.
- Invalid responses/timeouts clear the forwarded values to zero.

## Wiring

1. Connect the energy meter A/B pair to `RS485_A_2` / `RS485_B_2`.
2. Connect the PC USB-RS-485 adapter A/B pair to `RS485_A_1` / `RS485_B_1`.
3. Join signal grounds when available. Do not connect the meter supply to A/B.
4. Configure the meter for ID `4`, `9600 8E1`. Configure Docklight on port 1 for
   `9600 8E1` and use Hex display.
5. Keep each bus daisy-chained and terminate only its physical ends.

## Keil live-debug watch values

The project options configure Watch 1 with the meter connection/status fields and
all four received registers. Open `Prepaid_Interface_Test.uvproj` and start Debug
mode to inspect them:

- `energy_meter_connected`: `1` after a valid meter response; `0` at startup or
  after an exception/invalid response timeout.
- `energy_meter_data_valid`: `1` when the current register array contains a
  valid response; otherwise `0`.
- `energy_meter_successful_responses`: increments for each valid meter response.
- `energy_meter_registers[0]` through `energy_meter_registers[3]`: meter's raw
  16-bit register values, also streamed on port 1.

The register array is populated by the master poll on RS-485 port 2. The values
are streamed on RS-485 port 1 for Docklight. The declarations are in
`src\\rs485_test.h`; the values are `volatile` globals for debugger inspection.

## Build

Open `Prepaid_Interface_Test.uvproj` in Keil uVision and build target
`PREPAY_V0_Modbus_Bridge`. The HEX and BIN are generated in `Output`; flash the
new HEX to the controller after building.
