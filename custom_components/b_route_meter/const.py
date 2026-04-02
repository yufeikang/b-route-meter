"""Constants for the B Route Smart Meter integration."""

from datetime import timedelta

DOMAIN = "b_route_meter"
ENTRY_TITLE = "B-Route Smart Meter"
CONF_SCAN_INTERVAL = "scan_interval"
DEFAULT_SCAN_INTERVAL = 300
MIN_SCAN_INTERVAL = 30
MAX_SCAN_INTERVAL = 3600

ATTR_API_INSTANTANEOUS_POWER = "instantaneous_power"
ATTR_API_TOTAL_CONSUMPTION = "total_consumption"
ATTR_API_INSTANTANEOUS_CURRENT_T_PHASE = "instantaneous_current_t_phase"
ATTR_API_INSTANTANEOUS_CURRENT_R_PHASE = "instantaneous_current_r_phase"
ATTR_API_TOTAL_CONSUMPTION_REVERSE = "total_consumption_reverse"
ATTR_API_FAULT_STATUS = "fault_status"
ATTR_API_SERIAL_NUMBER = "serial_number"
ATTR_API_MANUFACTURER_CODE = "manufacturer_code"
ATTR_API_ECHONET_VERSION = "echonet_version"
