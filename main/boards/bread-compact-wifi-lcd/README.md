# bread-compact-wifi-lcd: vehicle CAN

The board listens for classic CAN standard (11-bit) data frames at 500 kbit/s.
Connect ESP32-S3 GPIO41 to the TJA105x TXD input and GPIO42 to its RXD output;
connect the grounds together and CANH/CANL to the bus. Ensure the transceiver's
power and RXD logic levels are compatible with the 3.3 V ESP32-S3, and set any
standby/slope pins for normal receive operation. Use appropriate CAN termination
at the two ends of the bus, not at every node.

The controller is in listen-only mode: it never transmits or acknowledges
frames. A separate active node must acknowledge the sender. This firmware does
not support CAN FD or extended-ID frames.

| Standard ID | Data | Interpretation |
| --- | --- | --- |
| `0x777` | Exactly 2 bytes | Unsigned big-endian integer, e.g. `01 02` is 258. No unit or scale is assumed. |
| `0x666` | Exactly 1 byte | Battery percentage, capped at 100. |
| `0x555` | Exactly 1 byte | `0`: off, `1`: left, `2`: right; other values are invalid. |

The latest values are retained in the board's CAN service and can be queried
through the AI-visible `self.vehicle.get_status` tool, for example by asking
“请告诉我车速、车的电量和转向灯状态”. Each value includes its age; absent or
malformed data is reported separately instead of being treated as zero. Values
remain the last received samples, so use their ages to judge whether they are
still current. This information is not a safety-critical driving instrument.

Build the `bread-compact-wifi-lcd` variant with ESP-IDF 6.0.1 or later:

```sh
python3 scripts/build.py bread-compact-wifi-lcd --name bread-compact-wifi-lcd
```
