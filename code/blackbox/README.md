# Blackbox log analysis

Python scripts I use to read Betaflight Blackbox logs and make my own graphs, instead of just scrolling through Blackbox Explorer.

Write-up using these: [Nov 22, 2026 log](https://proj-v-op.github.io/projects/drone/blackbox-2026-11-22.html)

## The scripts

| Script | What it does |
|---|---|
| `decode.py` | Splits the `.BBL` file into its separate logs (one per arm), decodes each into a NumPy array, and saves the header settings. Prints a quick summary for each log (length, max throttle, peak current, min voltage) so you can tell real flights from bench arms. |
| `plots.py` | Makes the graphs for the main flight: overview, setpoint vs gyro, PID terms, noise spectrum, noise vs throttle, step response, motor distribution. |
| `balance.py` | Graph of average motor output by position plus I-term over time. This is how I spotted the tail-heavy balance. |
| `to_csv.py` | Exports each log to CSV, plus a settings text file per log. |

## Running it

Needs `numpy`, `scipy`, `matplotlib`, and [orangebox](https://pypi.org/project/orangebox/) (the decoder). On new pip versions, `pip install orangebox` errors out on a broken script entry in the package. The workaround: download the wheel, unzip it, and point `ORANGEBOX_PATH` at that folder.

```
pip download --no-deps orangebox -d dl
python -m zipfile -e dl/orangebox-0.5.0-py3-none-any.whl orangebox_lib
export ORANGEBOX_PATH=$PWD/orangebox_lib

python decode.py LOG.BBL work/        # decode every log
python plots.py work/ graphs/         # graphs for logs 7 and 4 (edit the list at the bottom for your log numbers)
python balance.py work/ graphs/
python to_csv.py work/ csv/
```

## Notes on what the graphs mean

- **Setpoint vs gyro:** setpoint is what the sticks asked for, gyro is what the quad actually did. Good tune = they overlap.
- **Noise spectrum:** needs debug mode set to `GYRO_SCALED` so the log has the raw gyro next to the filtered one. A peak that's the same height in both means the filters aren't catching it.
- **Noise vs throttle:** a band that climbs with throttle is motor/prop noise. A flat line that stays put is something else: frame resonance or a control loop oscillation.
- **Step response:** estimated by deconvolving gyro against setpoint (same idea as PIDtoolbox). Only trustworthy with sharp stick inputs in Acro.
- **Motor order:** Betaflight's default quad-X is M1 rear right, M2 front right, M3 rear left, M4 front left.
