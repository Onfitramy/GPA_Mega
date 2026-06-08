# Firmware Architecture

## Hardware
MCU, board, clock tree, memory map

## Boot flow
Reset -> HAL init -> clock config -> peripheral init -> app_init -> main loop / scheduler

## Main modules
app, bsp, drivers, protocol, storage, diagnostics

## Interrupts
IRQ name, priority, owner module, shared state

## DMA
Peripheral, stream/channel, buffer location, cache maintenance rules

## Timing
Timers, control loop frequencies, deadlines

## Error handling
asserts, fault handlers, watchdog, logging

## Build variants
debug/release/bootloader/test