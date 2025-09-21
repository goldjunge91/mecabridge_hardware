# Contract: Frame Protocol (MecaBridge)

Version: 1.0.0 (initial)  
Related FR: FR-007, FR-008, FR-011, FR-012, FR-017, FR-020

## 1. Overview
Binary protocol between Pi (host) and Pico (firmware). Reliability: integrity via CRC-16/CCITT-FALSE. Safety: watchdog depends on timely COMMAND frames.

## 2. Common Structure
```
+------------+----------+---------+----------+---------+
| START_BYTE | FRAME_ID |  LEN    | PAYLOAD  |  CRC16  |
|  (0xAA)    | 1 byte   | 1 byte  | N bytes  | 2 bytes |
+------------+----------+---------+----------+---------+
```
- LEN counts only PAYLOAD bytes.
- CRC16 computed over FRAME_ID, LEN, PAYLOAD (exclude START_BYTE & CRC bytes).
- Multi-byte numeric fields little-endian, except CRC transmitted MSB first.

## 3. Frame IDs
| ID | Name | Direction | Notes |
|----|------|-----------|-------|
| 0x01 | COMMAND | Host → Pico | Actuator targets + seq |
| 0x02 | STATE | Pico → Host | Encoder, flags, echo seq |
| 0x03 | HEARTBEAT (optional) | Host → Pico | May be omitted if COMMAND periodic |
| 0x04 | VERSION_INFO | Either | Negotiation / mismatch detection |
| 0x05 | ERROR | Pico → Host | Optional explicit error dispatch |

Initial implementation MUST support 0x01 and 0x02. Others reserved.

## 4. COMMAND Payload Layout
```
Offset  Size  Field                      Type      Description
0       4*4   wheel_vel_rad_s[4]         float32   Order: FL, FR, RL, RR
16      4     servo_pos_rad              float32   Positional servo target (rad)
20      4     servo_cont_vel_norm        float32   Continuous servo velocity (-1..1 or rad/s normalized)
24      4*2   esc_norm[2]                float32   ESC normalized [-1..1]
32      2     seq                        uint16    Incrementing sequence
34      1     reserved_flags             uint8     Future use (0)
```
LEN = 35. Total frame bytes = 1 + 1 + 1 + 35 + 2 = 40.

Validation Rules:
- All normalized values within [-1.0, 1.0]. Clamp + set limit flag if not.
- servo_pos_rad within configured [min,max]; clamp + set limit flag.

## 5. STATE Payload Layout
```
Offset  Size  Field                      Type      Description
0       4*4   encoder_counts[4]          uint32    Raw absolute counters
16      2     dt_ms                      uint16    Elapsed ms since previous state frame
18      4     servo_pos_rad              float32   Current servo position (rad)
22      4     servo_cont_vel_norm        float32   Echo last commanded (v1)
26      4*2   esc_norm[2]                float32   Echo last commanded normalized
34      2     seq_echo                   uint16    Last accepted COMMAND seq
36      2     flags                      uint16    Bitfield (see Flags)
38      1     error_code                 uint8     Recent error cause
```
LEN = 39. Total frame bytes = 1 + 1 + 1 + 39 + 2 = 44.

## 6. Flags Bitfield
See `data-model.md` section 3. Bits 0..4 currently defined.

## 7. CRC16/CCITT-FALSE Reference
- Polynomial: 0x1021
- Init: 0xFFFF
- Reflect In: false
- Reflect Out: false
- XorOut: 0x0000

Pseudo-code:
```c
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
    }
  }
  return crc;
}
```
Test Vector (ASCII "123456789") → 0x29B1.

## 8. Error Handling
- CRC fail: drop frame, set `crc_error_since_last` flag, `error_code=CRC_FAIL` (if none higher precedence).
- Malformed (LEN mismatch / truncated): drop, set `malformed_error_since_last`.
- Unknown FRAME_ID: drop, optionally raise `UNKNOWN_FRAME_ID` error code.
- Watchdog timeout: set flags.watchdog_triggered, error_code=WATCHDOG_TIMEOUT until a valid command resets it.

## 9. Sequence Reliability
Host increments `seq`; firmware stores `last_seq`. If new seq == last_seq (duplicate) → ignore commands but still treat as heartbeat. If gap >1 → may log but still accept.

## 10. Version Negotiation (Future)
VERSION_INFO frame (0x04) payload proposal (not implemented v1): major(uint8), minor(uint8), patch(uint8).

## 11. Compliance Tests (Planned)
- Encode/decode roundtrip matches binary layout sizes.
- CRC vector test passes.
- Invalid CRC frames ignored; flags set.
- Watchdog triggers upon deliberate command silence.

This contract is STABLE for v1. Changes require MINOR (additive) or MAJOR (breaking) semantic versioning updates.
