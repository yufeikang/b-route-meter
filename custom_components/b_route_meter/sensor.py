"""Sensor platform for B-Route Smart Meter.

Defines the sensor entities that read E7/E8/E9/EA/EB data from
the B-route meter using a DataUpdateCoordinator.

Also provides a diagnostic sensor that shows device status and network information.
"""

import logging

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import (
    EntityCategory,
    UnitOfElectricCurrent,
    UnitOfElectricPotential,
    UnitOfEnergy,
    UnitOfPower,
)
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import (
    DEVICE_MANUFACTURER,
    DEVICE_MODEL,
    DEVICE_NAME,
    DEVICE_UNIQUE_ID,
    DOMAIN,
)

_LOGGER = logging.getLogger(__name__)

SENSOR_TYPES: list[SensorEntityDescription] = [
    SensorEntityDescription(
        key="diagnostic_info",
        name="B-Route Diagnostic Info",
        icon="mdi:information",
        entity_category=EntityCategory.DIAGNOSTIC,
        entity_registry_enabled_default=False,  # Disabled by default, user needs to enable manually
    ),
    SensorEntityDescription(
        key="e7_power",
        name="B-Route Instantaneous Power",
        icon="mdi:flash",
        device_class=SensorDeviceClass.POWER,
        native_unit_of_measurement=UnitOfPower.WATT,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    SensorEntityDescription(
        key="e8_current",
        name="B-Route Instantaneous Current",
        icon="mdi:current-ac",
        device_class=SensorDeviceClass.CURRENT,
        native_unit_of_measurement=UnitOfElectricCurrent.AMPERE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    SensorEntityDescription(
        key="e9_voltage",
        name="B-Route Instantaneous Voltage",
        icon="mdi:power-plug",
        device_class=SensorDeviceClass.VOLTAGE,
        native_unit_of_measurement=UnitOfElectricPotential.VOLT,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    SensorEntityDescription(
        key="ea_forward",
        name="B-Route Cumulative Forward",
        icon="mdi:gauge",
        device_class=SensorDeviceClass.ENERGY,
        native_unit_of_measurement=UnitOfEnergy.KILO_WATT_HOUR,
        state_class=SensorStateClass.TOTAL_INCREASING,
    ),
    SensorEntityDescription(
        key="eb_reverse",
        name="B-Route Cumulative Reverse",
        icon="mdi:gauge",
        device_class=SensorDeviceClass.ENERGY,
        native_unit_of_measurement=UnitOfEnergy.KILO_WATT_HOUR,
        state_class=SensorStateClass.TOTAL_INCREASING,
    ),
]


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up sensor entities from a config entry."""
    coordinator = entry.runtime_data
    _LOGGER.debug("Setting up B-Route meter sensor platform")
    await coordinator.async_config_entry_first_refresh()

    sensors = [BRouteSensorEntity(entry, description) for description in SENSOR_TYPES]

    async_add_entities(sensors)


class BRouteSensorEntity(SensorEntity):
    """B-Route sensor entity referencing a SensorEntityDescription.

    We store a reference to the DataUpdateCoordinator and a SensorEntityDescription,
    and we get the current sensor value from coordinator.data.
    """

    def __init__(
        self,
        config_entry: ConfigEntry,
        description: SensorEntityDescription,
    ) -> None:
        """Initialize the sensor."""

        self._coordinator = config_entry.runtime_data
        self.entity_description = description
        self._attr_unique_id = f"b_route_{description.key}"
        self._last_state = None
        self._last_timestamp = None
        _LOGGER.debug(
            "Setting up B-Route sensor entity for %s", self.entity_description.key
        )

    @property
    def should_poll(self) -> bool:
        """Disable polling, because DataUpdateCoordinator handles updates."""
        return False

    @property
    def available(self) -> bool:
        """Is sensor available."""
        # Always available regardless of coordinator update status
        # This allows sensors to show the last known value even when updates fail
        return True

    @property
    def extra_state_attributes(self) -> dict[str, str]:
        """Return entity specific state attributes."""
        attributes = {}
        data = self._coordinator.data
        timestamp_key = None

        if not data:
            return attributes

        # Add detailed attributes for diagnostic_info sensor
        if self.entity_description.key == "diagnostic_info":
            diagnostic_data = data.get("diagnostic_info")
            if diagnostic_data:
                # Device information
                if diagnostic_data.mac_address:
                    attributes["mac_address"] = diagnostic_data.mac_address
                if diagnostic_data.ipv6_address:
                    attributes["ipv6_address"] = diagnostic_data.ipv6_address
                if diagnostic_data.stack_version:
                    attributes["stack_version"] = diagnostic_data.stack_version
                if diagnostic_data.app_version:
                    attributes["app_version"] = diagnostic_data.app_version

                # Network configuration
                if diagnostic_data.channel:
                    attributes["channel"] = diagnostic_data.channel
                if diagnostic_data.pan_id:
                    attributes["pan_id"] = diagnostic_data.pan_id

                # Network status
                if diagnostic_data.active_tcp_connections:
                    attributes["tcp_connections_count"] = len(
                        diagnostic_data.active_tcp_connections
                    )
                    # Add details for each TCP connection
                    for i, conn in enumerate(diagnostic_data.active_tcp_connections):
                        attributes[f"tcp_connection_{i+1}"] = str(conn)

                if diagnostic_data.udp_ports:
                    attributes["udp_ports"] = ", ".join(
                        str(port) for port in diagnostic_data.udp_ports
                    )

                if diagnostic_data.tcp_ports:
                    attributes["tcp_ports"] = ", ".join(
                        str(port) for port in diagnostic_data.tcp_ports
                    )

                if diagnostic_data.neighbor_devices:
                    attributes["neighbor_devices_count"] = len(
                        diagnostic_data.neighbor_devices
                    )
                    # Add details for each neighbor device
                    for i, neighbor in enumerate(diagnostic_data.neighbor_devices):
                        attributes[f"neighbor_device_{i+1}"] = str(neighbor)

            # Add timestamp if available
            if self._last_timestamp:
                attributes["last_update"] = self._last_timestamp

            return attributes

        # Handle attributes for other sensors
        if self.entity_description.key == "e7_power":
            timestamp_key = "power_timestamp"
        elif self.entity_description.key == "ea_forward":
            timestamp_key = "forward_timestamp"
        elif self.entity_description.key == "eb_reverse":
            timestamp_key = "reverse_timestamp"

        if data and timestamp_key in data:
            self._last_timestamp = data[timestamp_key]

        if self._last_timestamp:
            attributes["last_update"] = self._last_timestamp

        return attributes

    @property
    def native_value(self) -> float | str | None:
        """Current sensor reading."""
        data = self._coordinator.data
        _LOGGER.debug(
            "Getting value for %s, data: %s", self.entity_description.key, data
        )
        if not data:
            return None  # Return None instead of "Unknown" string
        key = self.entity_description.key

        # Special handling for diagnostic info
        if key == "diagnostic_info":
            diagnostic_data = data.get(key)
            if diagnostic_data:
                # Create a concise status summary
                connection_status = (
                    "ONLINE" if diagnostic_data.ipv6_address else "OFFLINE"
                )

                # Add info about connections and neighbors if available
                connection_info = []
                if diagnostic_data.active_tcp_connections:
                    tcp_count = len(diagnostic_data.active_tcp_connections)
                    connection_info.append(f"{tcp_count} CONN")

                if diagnostic_data.neighbor_devices:
                    neighbor_count = len(diagnostic_data.neighbor_devices)
                    connection_info.append(f"{neighbor_count} NEIGH")

                # Combine status components
                if connection_info:
                    return f"{connection_status} ({', '.join(connection_info)})"
                return connection_status
            return "NO DATA"

        # Normal handling for other sensors
        value = data.get(key)

        # If the value is None, try to use the last valid value
        if value is None:
            if self._last_state is not None:
                _LOGGER.debug(
                    "Using last known value for %s: %s", key, self._last_state
                )
                return self._last_state
            # If no last valid value, return None instead of "Unknown"
            return None

        # Save current value as the last valid value for next time
        self._last_state = value

        # Format numeric-type sensors
        if key in ["e7_power", "e8_current", "e9_voltage", "ea_forward", "eb_reverse"]:
            # Ensure the value is numeric
            try:
                numeric_value = float(value)
                # Use different precision based on sensor type
                if key == "e7_power":  # Power, integer
                    return int(numeric_value)
                elif key in [
                    "e8_current",
                    "e9_voltage",
                ]:  # Current and voltage, 1 decimal place
                    return round(numeric_value, 1)
                elif key in ["ea_forward", "eb_reverse"]:  # Energy, 2 decimal places
                    return round(numeric_value, 2)
            except (ValueError, TypeError):
                _LOGGER.warning("Invalid numeric value for %s: %s", key, value)
                return None  # Return None instead of "Invalid data" string

        return value

    @property
    def device_info(self) -> DeviceInfo:
        """Return device information."""
        return {
            "identifiers": {(DOMAIN, DEVICE_UNIQUE_ID)},
            "name": DEVICE_NAME,
            "manufacturer": DEVICE_MANUFACTURER,
            "model": DEVICE_MODEL,
        }

    async def async_added_to_hass(self) -> None:
        """Register update listener when entity is added."""
        self._coordinator.async_add_listener(self.async_write_ha_state)

    async def async_remove_listener(self) -> None:
        """Remove the listener."""
        self._coordinator.async_remove_listener(self.async_write_ha_state)
