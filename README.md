# B-Route Smart Meter Integration for Home Assistant

[![hacs_badge](https://img.shields.io/badge/HACS-Custom-41BDF5.svg)](https://github.com/hacs/integration)

Home Assistant custom integration for reading electricity data from Japanese smart meters via the Route B protocol (ECHONET Lite over Wi-SUN).

## Features

### Sensors

| Sensor | Type | Unit |
|--------|------|------|
| Instantaneous Power | Power | W |
| Instantaneous Current (R/T phase) | Current | A |
| Instantaneous Voltage (R/T phase) | Voltage | V |
| Total Consumption (forward) | Energy | kWh |
| Total Consumption (reverse/solar) | Energy | kWh |

### Diagnostic Entities

| Entity | Description |
|--------|-------------|
| Fault Status | Smart meter fault status |
| Serial Number | Meter serial number |
| Manufacturer Code | Meter manufacturer code |
| ECHONET Lite Version | Protocol version |

### Stability

- **Automatic session recovery**: Up to 5 retries with exponential backoff (max 60s)
- **Preemptive reconnection**: Proactively reopens session after 2 consecutive failures
- **Catches all exception types**: Handles serial errors, OS errors, and protocol errors
- **USB device auto-discovery**: Automatically detects Wi-SUN adapters

## Requirements

- A Wi-SUN USB adapter (e.g., ROHM BP35A1, BP35C2)
- B-Route service credentials (ID and password) from your electricity provider
- [momonga](https://github.com/yufeikang/momonga) library (installed automatically)

## Installation

### HACS (Recommended)

1. Add this repository as a custom repository in HACS
2. Search for "B-Route Smart Meter" and install
3. Restart Home Assistant

### Manual

1. Copy `custom_components/b_route_meter` to your HA `custom_components/` directory
2. Restart Home Assistant

## Configuration

1. Go to **Settings → Devices & Services → Add Integration**
2. Search for "B-Route Smart Meter"
3. Select your USB device from the dropdown
4. Enter your B-Route ID and password
5. The integration validates the connection before completing setup

## Troubleshooting

- Check Home Assistant logs for `b_route_meter` and `momonga` entries
- Ensure the USB adapter is properly connected and accessible
- Verify your B-Route credentials are correct
- The integration automatically recovers from connection drops — check logs for recovery messages
