"""B Route Smart Meter sensors."""

from collections.abc import Callable
from dataclasses import dataclass

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.const import (
    EntityCategory,
    UnitOfElectricCurrent,
    UnitOfEnergy,
    UnitOfPower,
)
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback
from homeassistant.helpers.typing import StateType
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BRouteConfigEntry
from .const import (
    ATTR_API_ECHONET_VERSION,
    ATTR_API_FAULT_STATUS,
    ATTR_API_INSTANTANEOUS_CURRENT_R_PHASE,
    ATTR_API_INSTANTANEOUS_CURRENT_T_PHASE,
    ATTR_API_INSTANTANEOUS_POWER,
    ATTR_API_MANUFACTURER_CODE,
    ATTR_API_SERIAL_NUMBER,
    ATTR_API_TOTAL_CONSUMPTION,
    ATTR_API_TOTAL_CONSUMPTION_REVERSE,
    DOMAIN,
)
from .coordinator import BRouteData, BRouteUpdateCoordinator


@dataclass(frozen=True, kw_only=True)
class SensorEntityDescriptionWithValueAccessor(SensorEntityDescription):
    """Sensor entity description with data accessor."""

    value_accessor: Callable[[BRouteData], StateType]


SENSOR_DESCRIPTIONS = (
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_INSTANTANEOUS_CURRENT_R_PHASE,
        translation_key=ATTR_API_INSTANTANEOUS_CURRENT_R_PHASE,
        device_class=SensorDeviceClass.CURRENT,
        state_class=SensorStateClass.MEASUREMENT,
        native_unit_of_measurement=UnitOfElectricCurrent.AMPERE,
        value_accessor=lambda data: data.instantaneous_current_r_phase,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_INSTANTANEOUS_CURRENT_T_PHASE,
        translation_key=ATTR_API_INSTANTANEOUS_CURRENT_T_PHASE,
        device_class=SensorDeviceClass.CURRENT,
        state_class=SensorStateClass.MEASUREMENT,
        native_unit_of_measurement=UnitOfElectricCurrent.AMPERE,
        value_accessor=lambda data: data.instantaneous_current_t_phase,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_INSTANTANEOUS_POWER,
        translation_key=ATTR_API_INSTANTANEOUS_POWER,
        device_class=SensorDeviceClass.POWER,
        state_class=SensorStateClass.MEASUREMENT,
        native_unit_of_measurement=UnitOfPower.WATT,
        value_accessor=lambda data: data.instantaneous_power,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_TOTAL_CONSUMPTION,
        translation_key=ATTR_API_TOTAL_CONSUMPTION,
        device_class=SensorDeviceClass.ENERGY,
        state_class=SensorStateClass.TOTAL,
        native_unit_of_measurement=UnitOfEnergy.KILO_WATT_HOUR,
        value_accessor=lambda data: data.total_consumption,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_TOTAL_CONSUMPTION_REVERSE,
        translation_key=ATTR_API_TOTAL_CONSUMPTION_REVERSE,
        device_class=SensorDeviceClass.ENERGY,
        state_class=SensorStateClass.TOTAL,
        native_unit_of_measurement=UnitOfEnergy.KILO_WATT_HOUR,
        value_accessor=lambda data: data.total_consumption_reverse,
    ),
)

DIAGNOSTIC_SENSOR_DESCRIPTIONS = (
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_FAULT_STATUS,
        translation_key=ATTR_API_FAULT_STATUS,
        entity_category=EntityCategory.DIAGNOSTIC,
        value_accessor=lambda data: data.fault_status,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_SERIAL_NUMBER,
        translation_key=ATTR_API_SERIAL_NUMBER,
        entity_category=EntityCategory.DIAGNOSTIC,
        value_accessor=lambda _data: None,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_MANUFACTURER_CODE,
        translation_key=ATTR_API_MANUFACTURER_CODE,
        entity_category=EntityCategory.DIAGNOSTIC,
        value_accessor=lambda _data: None,
    ),
    SensorEntityDescriptionWithValueAccessor(
        key=ATTR_API_ECHONET_VERSION,
        translation_key=ATTR_API_ECHONET_VERSION,
        entity_category=EntityCategory.DIAGNOSTIC,
        value_accessor=lambda _data: None,
    ),
)


def _build_device_info(coordinator: BRouteUpdateCoordinator) -> DeviceInfo:
    return DeviceInfo(
        identifiers={(DOMAIN, coordinator.bid)},
        name=f"B-Route Smart Meter {coordinator.bid}",
        manufacturer=coordinator.device_info_data.manufacturer_code,
        serial_number=coordinator.device_info_data.serial_number,
        sw_version=coordinator.device_info_data.echonet_version,
    )


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BRouteConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Set up B Route Smart Meter entry."""
    coordinator = entry.runtime_data

    entities: list[SensorEntity] = [
        SmartMeterBRouteSensor(coordinator, description)
        for description in SENSOR_DESCRIPTIONS
    ]
    entities.extend(
        SmartMeterBRouteDiagnosticSensor(coordinator, description)
        for description in DIAGNOSTIC_SENSOR_DESCRIPTIONS
    )
    async_add_entities(entities)


class SmartMeterBRouteSensor(CoordinatorEntity[BRouteUpdateCoordinator], SensorEntity):
    """Representation of a B Route Smart Meter sensor entity."""

    _attr_has_entity_name = True

    def __init__(
        self,
        coordinator: BRouteUpdateCoordinator,
        description: SensorEntityDescriptionWithValueAccessor,
    ) -> None:
        """Initialize B Route Smart Meter sensor entity."""
        super().__init__(coordinator)
        self.entity_description: SensorEntityDescriptionWithValueAccessor = description
        self._attr_unique_id = f"{coordinator.bid}_{description.key}"
        self._attr_device_info = _build_device_info(coordinator)

    @property
    def native_value(self) -> StateType:
        """Return the state of the sensor."""
        return self.entity_description.value_accessor(self.coordinator.data)

    @property
    def extra_state_attributes(self) -> dict[str, str] | None:
        """Return meter's own measurement timestamp for cumulative energy sensors."""
        key = self.entity_description.key
        data = self.coordinator.data
        if key == ATTR_API_TOTAL_CONSUMPTION and data.meter_timestamp:
            return {"meter_timestamp": data.meter_timestamp.isoformat()}
        if key == ATTR_API_TOTAL_CONSUMPTION_REVERSE and data.meter_timestamp_reverse:
            return {"meter_timestamp": data.meter_timestamp_reverse.isoformat()}
        return None


class SmartMeterBRouteDiagnosticSensor(
    CoordinatorEntity[BRouteUpdateCoordinator], SensorEntity
):
    """Diagnostic sensor sourcing values from static device info or polled data."""

    _attr_has_entity_name = True

    _STATIC_KEY_MAP = {
        ATTR_API_SERIAL_NUMBER: "serial_number",
        ATTR_API_MANUFACTURER_CODE: "manufacturer_code",
        ATTR_API_ECHONET_VERSION: "echonet_version",
    }

    def __init__(
        self,
        coordinator: BRouteUpdateCoordinator,
        description: SensorEntityDescriptionWithValueAccessor,
    ) -> None:
        super().__init__(coordinator)
        self.entity_description: SensorEntityDescriptionWithValueAccessor = description
        self._attr_unique_id = f"{coordinator.bid}_{description.key}"
        self._attr_device_info = _build_device_info(coordinator)

    @property
    def native_value(self) -> StateType:
        """Return static device info or polled diagnostic value."""
        key = self.entity_description.key
        if key == ATTR_API_FAULT_STATUS:
            return self.coordinator.data.fault_status
        attr = self._STATIC_KEY_MAP.get(key)
        if attr:
            return getattr(self.coordinator.device_info_data, attr, None)
        return None
