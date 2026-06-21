# B-Route reopen_delays live test

## Setup
- Branch state (uncommitted): manifest momonga → nbtk@9037771, coordinator `auto_reopen=True` → `reopen_delays=[10, 60, 300, 600]`
- Target: BP35A1 on /dev/ttyUSB0, USB port 3-2
- Action: sysfs unbind for 75s, then rebind

## Timeline (2026-05-13 JST)
| t | event |
|---|---|
| 02:19:33.728 | last successful TX (tx id 37E1) |
| 02:19:35      | unbind → /dev/ttyUSB0 gone |
| 02:19:45.729 | tx 37E1 timeout (+12s wait) |
| 02:19:45-51  | 3× SerialException "write failed [Errno 5]" |
| 02:19:54.735 | ERROR "Could not transmit. Close Momonga and open it again." |
| 02:19:54.739 | WARNING "Session needs reopen, attempting recovery." → schedules at +10s |
| 02:20:04.739 | **1st reopen fires (exactly +10.0s)** → ttyUSB0 still gone → SerialException ENOENT |
| 02:20:04.748 | WARNING "Reopen attempt failed after waiting 10.0 seconds" — `reopen_delays[0]=10` confirmed |
| 02:20:50      | rebind → /dev/ttyUSB0 returns |
| 02:21:04.749 | **2nd reopen fires (exactly +60s after 1st)** — `reopen_delays[1]=60` confirmed |
| 02:21:07.012 | Route-B credentials registered |
| 02:21:59.447 | PAN found (~52s scan) |
| 02:22:01.233 | "A Momonga session is open" |
| 02:22:06.529 | first successful TX after reopen (tx id 37E2) |

Total user-visible outage: 2m 33s (most of which is the BP35 PAN re-scan, not library overhead).

## Verdict
- ✅ Disconnect detection via SerialException + retransmit exhaustion
- ✅ `reopen_delays[0] = 10s` first retry delay verified
- ✅ `reopen_delays[1] = 60s` second retry delay verified
- ✅ Reopen on second attempt succeeded cleanly
- ✅ Coordinator polling resumed automatically, no HA restart needed
- ✅ Entities went unavailable → recovered to live values

The change (yufeikang/momonga@v0.4.0 → nbtk@9037771) and the new `reopen_delays` API are validated against a live disconnect/reconnect cycle.
