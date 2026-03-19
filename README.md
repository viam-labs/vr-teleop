# VR Teleop

Reads HTC Vive controller poses and button states via [libsurvive](https://github.com/cntools/libsurvive) and sends arm/gripper commands to a Viam robot at ~90 Hz. No SteamVR, frontend, or WebSocket required.

## Hardware

- **Watchman VR Dongles** — USB receivers for the controllers (one per controller)
- **Lighthouse Base Stations (1.0 or 2.0)** — at least two for full room-scale tracking
- **Vive 2.0 Controllers** — one per arm

### Controls

- **Trigger** — proportional gripper control. Gripper position mirrors trigger pull.
- **Grip** — deadman's switch. Arm moves only while held.
- **Menu** — returns arm to previous start pose. Each press steps further back.
- **Trackpad up** — recalibrate forward direction to current controller heading.
- **Trackpad down** — toggle absolute/relative rotation tracking.

## Software Requirements

- Go 1.21+
- Linux (libsurvive requires direct USB access to Lighthouse hardware)
- cmake, libusb-1.0-dev, zlib1g-dev (for building libsurvive)
- A Viam robot with arm and (optionally) gripper components

On Debian/Ubuntu:

```sh
sudo apt install cmake libusb-1.0-0-dev zlib1g-dev
```

### USB permissions

libsurvive accesses Vive hardware directly via USB.

**Linux** — add udev rules (one-time):

```sh
sudo cp libsurvive-src/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**macOS** — run with `sudo` (USB device access requires root).

### Pairing controllers

Each Watchman USB dongle must be paired to a controller (one-time, stored in dongle firmware). Plug in both dongles, then:

```sh
sudo make pair
```

Power on each controller one at a time and wait for it to pair. Press Ctrl-C when done.

## Setup

```sh
make setup   # go mod tidy
make build   # builds libsurvive (first time only) + binary
```

After that, `make build` only rebuilds when sources change.

## Run

```sh
cp .env.example .env   # fill in your credentials
make dev
```

Or run the binary directly:

```sh
./vr-teleop --address <machine-address> --key-id <key-id> --key <key>
```

### Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--address` | (required) | Viam machine address |
| `--key-id` | | Viam API key ID |
| `--key` | | Viam API key |
| `--hz` | `90` | Polling rate in Hz |
| `--left-arm` | `right-arm` | Arm controlled by the left controller |
| `--right-arm` | `left-arm` | Arm controlled by the right controller |
| `--left-gripper` | `right-gripper` | Gripper controlled by the left controller (empty to disable) |
| `--right-gripper` | `left-gripper` | Gripper controlled by the right controller (empty to disable) |
| `--left-controller` | (auto) | libsurvive object name for left controller (e.g. `WM0`) |
| `--right-controller` | (auto) | libsurvive object name for right controller (e.g. `WM1`) |
| `--scale` | `1.0` | Position scale factor (0.1–3.0) |
| `--rotation` | `true` | Enable orientation tracking |

### Controller assignment

libsurvive names controllers as `WM0`, `WM1`, etc. By default the first two discovered controllers are assigned left/right automatically. If they're swapped, use `--left-controller` and `--right-controller` to specify explicitly.

## Configuration

Copy `.env.example` to `.env` and fill in your Viam credentials and arm/gripper mappings. This file is gitignored.
