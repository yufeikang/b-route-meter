"""The B Route Smart Meter integration."""

import logging

from homeassistant.const import Platform
from homeassistant.core import HomeAssistant

from .coordinator import BRouteConfigEntry, BRouteUpdateCoordinator

_LOGGER = logging.getLogger(__name__)
PLATFORMS: list[Platform] = [Platform.SENSOR]


async def async_setup_entry(hass: HomeAssistant, entry: BRouteConfigEntry) -> bool:
    """Set up B Route Smart Meter from a config entry."""

    coordinator = BRouteUpdateCoordinator(hass, entry)
    await coordinator.async_config_entry_first_refresh()
    entry.runtime_data = coordinator
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)

    entry.async_on_unload(entry.add_update_listener(_async_update_listener))

    return True


async def _async_update_listener(hass: HomeAssistant, entry: BRouteConfigEntry) -> None:
    """Reload integration when options change."""
    await hass.config_entries.async_reload(entry.entry_id)


async def async_unload_entry(hass: HomeAssistant, entry: BRouteConfigEntry) -> bool:
    """Unload a config entry."""
    await hass.async_add_executor_job(entry.runtime_data.api.close)
    return await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
