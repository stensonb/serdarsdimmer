# Serdar's Dimmer

This contains designs/code for a dimmer.  The dimmer oscillates between "high" and "low" in a cycle.  If controlling lights, this might make the lights seems like they're "breathing" (as they'll always be changing their brightness).

## Hardware

| Qty | Description |
|-----|-------------|
| 1 | stm32f103c8t6 - contains logic to control the PWM output for the AC dimmer module |
| 3 | 10k potentiometers - used a analog input devices to set "high", "low" and "cycle period" values |
| 1 | 120/240v -> 3.3v transformer - use to power the stm32 |
| 1 | AC dimmer module - this modules accepts a PWM input, and the average wave-form it interprets is used to pass line-current to the output |

## Inputs

| Qty | Description |
|-----|-------------|
| 3 | dials to set "high", "low" and "cycle period" |
| 1 | 120/240v line current (supports up to 1200W) |

## Outputs
| Qty | Description |
|-----|-------------|
| 1 | 120/240v line current with voltage modulated (via PWM) based on the slice of time within the dimming cycle. |

## Dimming Cycle
On power on, a dimmer cycle loop is started with 1000 steps.


### Limits

| Limit | Value |
|-------|-------|
| Longest Dimming Cycle | 10 minutes |
| Shortest Dimming Cycle | 1 second |
| Highest "high" | line voltage (full line current) |
| Lowest "high" | logically, the "low" reading + 1 |
| Highest "low" | logically, the "high" reading - 1 |
| Lowest "low" | 0 (no line current) |