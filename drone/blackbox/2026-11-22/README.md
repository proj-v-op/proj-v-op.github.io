# Blackbox log: Nov 22, 2026

5" quad, Foxeer F722V2, Betaflight 4.3.0, 6S. Full write-up: [proj-v-op.github.io/projects/drone/blackbox-2026-11-22.html](https://proj-v-op.github.io/projects/drone/blackbox-2026-11-22.html)

- `BTFL_BLACKBOX_LOG_...BBL`: the raw log straight off the flash chip (7 logs inside; log 7 is the real flight)
- `main-flight_250Hz.csv`: log 7 decoded to CSV, downsampled from 2 kHz to 250 Hz to keep the file small
- `settings.txt`: every Betaflight setting recorded in the log header
- `graphs/`: every graph from the write-up

Main takeaways: a constant 62 Hz oscillation on roll, the quad is tail heavy, motor noise filtering is good, and the current sensor reads low.
