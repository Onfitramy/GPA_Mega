# InterBoardCom master diagnostics

The diagnostics are observational. They do not retry failed transfers, reject
bad packets, change the PA4 handshake, or recover a stalled SPI peripheral.
CRC failures are counted and the packet continues through the existing parser.

## CLI usage

Reset the counters immediately before a test:

```text
ib.diag reset
```

Read a consistent snapshot without clearing it:

```text
ib.diag
```

The command emits six lines. Important interpretations are:

- `queue ... full` is the number of outgoing packets silently rejected by the
  existing 16-packet circular buffer.
- `DMA ... busy` and `DMA ... error` are immediate return values from
  `HAL_SPI_TransmitReceive_DMA()`. With the current transport, either can leave
  the application SPI state busy.
- `stalls` increments once for a transfer that is still marked busy at least
  `INTERBOARD_DIAG_STALL_THRESHOLD_MS` after it was started.
- `SPI errors` counts asynchronous HAL SPI/DMA error callbacks. `last` is the
  raw `HAL_SPI_ERROR_*` bit mask.
- `crc-bad` counts corrupt `DataPacket_t` payloads. It does not include raw
  echo frames or unknown outer packet IDs.
- `unknown` counts nonzero outer InterBoard packet IDs that the current master
  parser does not recognize.
- `RX queue ... full` counts received packets discarded because the FreeRTOS
  receive queue was full.
- `accepted - processed` approximates the current receive backlog. The CLI
  snapshot can race one in-flight event, so a difference of one is not by
  itself an error.
- `waits/zero` exposes the current `eNoAction` plus `ulTaskNotifyTake()` usage.
  A zero return is expected with the existing code and is recorded so it can
  be compared before and after notification changes.

`app-state` is `SPI1_State`; zero is ready and one is busy. `hal-state` is the
numeric `HAL_SPI_StateTypeDef`. A persistent app-state of one with a HAL ready
state (`hal-state=1`) is evidence that the application state has become wedged.
The HAL start statuses are 0=OK, 1=ERROR, 2=BUSY, and 3=TIMEOUT.

Common `last-spi-error` mask bits are `0x01` mode fault, `0x04` overrun,
`0x08` frame error, `0x10` DMA error, `0x20` FIFO/status flag error, `0x40`
abort error, `0x80` underrun, and `0x100` timeout. Multiple bits can be set.

## Debugger usage

Add this global to the watch window:

```c
InterBoardCom_Diagnostics
```

It is declared `volatile`, so counters remain visible while the target is
running. `tx_queue_depth`, `app_spi_state`, and `hal_spi_state` are populated in
a CLI/debug snapshot by `InterBoardCom_GetDiagnostics()`; the live queue count
is also available as `txCircBuffer.count`.

## Suggested baseline test

1. Flash a build containing only the diagnostics changes.
2. Run `ib.diag reset` after both boards have completed startup.
3. Select the communication/save schedule being investigated.
4. Reproduce the problem for a fixed duration.
5. Run `ib.diag` before resetting either MCU.
6. Record the schedule, duration, both MCU states, and the complete output.
7. Repeat at least three times so intermittent errors can be separated from
   deterministic queue overload.

For debugger-only failures, halt the target before resetting it and capture
`InterBoardCom_Diagnostics`, `SPI1_State`, `hspi1.State`, `hspi1.ErrorCode`, and
`txCircBuffer`.
