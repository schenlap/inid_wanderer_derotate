# inid_wanderer_derotate
indi wanderer rotator driver enhancement with derotate function for astro photography

# WandererRotator Mini / Lite – Serial Protocol (unofficial)

This document describes the serial command protocol of the Wanderer Astro rotators
(**WandererRotator Mini V1/V2**, **Lite V1**, **Lite V2**), as far as it is known from

1. the [INDI](https://github.com/indilib/indi) driver (`drivers/rotator/wanderer_rotator_*.cpp`), and
2. a USB capture of the manufacturer's software while it was running the *derotation* function
   (Mini only).

It is **not** an official specification. Everything below was observed or inferred; open questions are
listed in [Open questions](#7-open-questions). Corrections and additions are welcome.

| Status | Meaning |
|---|---|
| **[driver]** | taken from the INDI driver source |
| **[capture]** | observed in a USB capture of the manufacturer's software |
| **[inferred]** | derived from the above, not directly observed |

## 1. Connection

- USB serial device, **19200 baud** **[driver]** (INDI uses 8N1, the default of its serial connection)
- In the USB capture the commands appear as bulk transfers on endpoint 2 (OUT) **[capture]**
- Text protocol: commands are ASCII decimal numbers (plus the literal word `Stop`)
- Line terminator: the INDI driver sends the handshake and move commands **without** `\n`, and the other
  commands **with** `\n`; the manufacturer's software sends `\n`. The firmware therefore appears to
  accept both **[driver][capture]**
- Replies consist of **sections terminated by the letter `A`** **[driver]**

## 2. Models

| Model | Handshake name (reply) | Minimum firmware | Steps per degree |
|---|---|---|---|
| WandererRotator Mini V1/V2 | `WandererRotatorMini` | 20240226 | 1142 |
| WandererRotator Lite V1 | `WandererRotatorLite` | 20240403 | 1155 |
| WandererRotator Lite V2 | `WandererRotatorLiteV2` | 20240226 | 1199 |

All values from the INDI driver **[driver]**. The derotation capture (section 4) suggests that the Mini's
real resolution is about 0.000875° per step (1 / 1142.857). The INDI driver uses the integer value 1142.

## 3. Commands (host → rotator)

| Command | Meaning | Reply |
|---|---|---|
| `1500001` | Handshake / read status | 5 sections, see 3.1 |
| `1000000 + n` | **Relative move** by `n` steps; `n` may be negative. Example: `1034260` = +34260 steps, `999990` = −10 steps | 2 sections after the move has finished, see 3.2 |
| `Stop` | Abort the current move | as after a move |
| `1500002` | Set the current position as mechanical zero | – |
| `1600000 + b` | **Set backlash**, `b` in tenths of a degree (0…20). Example: `1600005` = 0.5°, `1600010` = 1.0° | – |
| `1700001` / `1700000` | Reverse direction / normal direction | – |
| `n` + `\n` | **Derotation step:** signed number of steps as a plain decimal number, e.g. `20\n`, `-11\n` | none observed |

All rows are **[driver]**, except the last one **[capture]** and the backlash examples, which were
confirmed in the capture as `1600005\n` (0.5°) and `1600010\n` (1.0°).

### 3.1 Reply to `1500001` (handshake)

Five sections, each terminated by `A` **[driver]**:

1. Device name (see table in section 2)
2. Firmware version as a date, e.g. `20250222`
3. Angle of the *virtual mechanical position* in **thousandths of a degree**, signed. The INDI driver sets
   the position to zero with `1500002` if its magnitude exceeds 400000 (= 400°)
4. Backlash (value as reported by the device; setting it uses tenths of a degree)
5. Direction: `0` = normal, anything else = reversed

### 3.2 Reply to a move (`1000000 + n`) or `Stop`

The rotator answers only after the move has finished (the Mini rotates at up to 4.5°/s). The INDI driver
reads two sections: the first is only used as a sign of life, the second contains the angle in thousandths
of a degree (same format as item 3 in 3.1). It waits up to 5 s per section **[driver]**.

## 4. Derotation (manufacturer's software)

There is **no mode command**. The manufacturer's software periodically sends the number of steps as plain
text followed by a line feed **[capture]**. Example payload of one USB bulk-OUT transfer:

```
32 30 0a        ->  "20\n"      (+20 steps)
2d 31 30 0a     ->  "-10\n"     (−10 steps)
```

Captured values for the Mini, with the rate shown in the software:

| Displayed rate | Interval | Degrees per command | Steps (÷ 0.000875°) | Sent |
|---|---|---|---|---|
| 0.2629 °/min | 4 s | 0.01753° | 20.0 | `20\n` |
| 0.1758 °/min | 4 s | 0.01172° | 13.4 | `13\n` |
| −0.1347 °/min | 4 s | −0.00898° | −10.3 | `-10\n` |
| −0.04 °/min | ≈ 15 s | −0.0100° | −11.4 | `-11\n` |

Findings **[inferred]**:

- steps per command = rate × interval ÷ degrees-per-step, rounded to an integer;
- at small rates several 4 s ticks are combined until about **10 steps** are reached, commands below this
  threshold are skipped (not sent);
- the software exposes a *derotation correction* of **−5 % … +5 %** in 0.1 % steps and a *backlash* of
  **0 … 2°** in 0.1° steps (default 0.5°).

## 5. Movement behaviour in the INDI driver

- Absolute angles are executed as **relative** moves: `steps = (target − current) × steps_per_degree`, sent as
  `1000000 + steps` **[driver]**.
- Moves of less than one step are not sent, because the firmware would not acknowledge them **[driver]**.
- The position is a **virtual mechanical angle**: signed, without end stop, not limited to 0…360° **[driver]**.

## 6. Field-rotation formula (for derotation)

For an alt-az mount at latitude φ, azimuth A (from north) and altitude h, the field rotates at

```
dθ/dt = ω_sid · cos(φ) · cos(A) / cos(h)        ω_sid = 360° / 86164.09 s ≈ 0.0041781 °/s
```

The number of steps per command is `round(dθ/dt · interval / degrees_per_step)`; the sign has to be chosen so
that the rotator turns against the field rotation (check once on a real star field).

## 7. Open questions

- Meaning of the first reply section after a move.
- Whether the rotator replies to the plain-text derotation steps (the capture only covered host → device).
- Unit of the backlash value reported in the handshake (setting it uses tenths of a degree).
- Behaviour when a derotation step and a `1000000 + n` move overlap (the INDI-based implementation avoids
  this by pausing derotation while another move is running).
- The capture was made with a Mini; the Lite models use different step counts per degree and were not captured.

## 8. How the data was obtained

- Driver facts: read from `wanderer_rotator_base.cpp`, `wanderer_rotator_mini.cpp`,
  `wanderer_rotator_lite.cpp` and `wanderer_rotator_lite_v2.cpp` of the INDI source tree.
- Capture: USB traffic of the manufacturer's software (Linux usbmon/Wireshark). In the usbmon record the
  command text is the payload at the end of each *Submit* packet for endpoint 2 (OUT).

*Not affiliated with Wanderer Astro. Product and company names belong to their respective owners.*
