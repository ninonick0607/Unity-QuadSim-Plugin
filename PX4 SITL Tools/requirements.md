# Python Requirements

The PX4 SITL tools require a small Python environment.

## Install

```bash
python -m venv .venv
source .venv/bin/activate
pip install pymavlink numpy matplotlib quadsim-sdk
```

If `quadsim-sdk` is not available from PyPI yet, install it from your local SDK checkout:

```bash
pip install -e /path/to/QuadSimLib
```

## Packages

| Package | Used by | Purpose |
|---|---|---|
| `pymavlink` | `px4_bridge.py`, `px4_confirm.py`, offboard scripts | MAVLink communication with PX4 |
| `quadsim-sdk` | `px4_bridge.py` | RPC connection into QuadSim |
| `numpy` | validation scripts | arrays and plotting data |
| `matplotlib` | validation scripts, live viewer | plots and live 3D viewer |

## Optional plain requirements.txt

You can also create this file beside the scripts:

```text
pymavlink
numpy
matplotlib
quadsim-sdk
```
