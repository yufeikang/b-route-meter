"""Constants for the B Route Smart Meter integration."""

from datetime import timedelta

DOMAIN = "b_route_meter"
ENTRY_TITLE = "B-Route Smart Meter"
DEFAULT_SCAN_INTERVAL = timedelta(seconds=300)

ATTR_API_INSTANTANEOUS_POWER = "instantaneous_power"
ATTR_API_TOTAL_CONSUMPTION = "total_consumption"
ATTR_API_INSTANTANEOUS_CURRENT_T_PHASE = "instantaneous_current_t_phase"
ATTR_API_INSTANTANEOUS_CURRENT_R_PHASE = "instantaneous_current_r_phase"
ATTR_API_TOTAL_CONSUMPTION_REVERSE = "total_consumption_reverse"
ATTR_API_INSTANTANEOUS_VOLTAGE_R_PHASE = "instantaneous_voltage_r_phase"
ATTR_API_INSTANTANEOUS_VOLTAGE_T_PHASE = "instantaneous_voltage_t_phase"
ATTR_API_FAULT_STATUS = "fault_status"
ATTR_API_SERIAL_NUMBER = "serial_number"
ATTR_API_MANUFACTURER_CODE = "manufacturer_code"
ATTR_API_ECHONET_VERSION = "echonet_version"
