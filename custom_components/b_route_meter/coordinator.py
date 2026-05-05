"""DataUpdateCoordinator for the B Route Smart Meter integration."""

from dataclasses import dataclass
from datetime import datetime, timedelta
import logging
import time

from momonga import Momonga, MomongaError, MomongaNeedToReopen

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_DEVICE, CONF_ID, CONF_PASSWORD
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL, DOMAIN

_LOGGER = logging.getLogger(__name__)


@dataclass
class BRouteDeviceInfo:
    """Static device information fetched once at setup."""

    serial_number: str | None = None
    manufacturer_code: str | None = None
    echonet_version: str | None = None


@dataclass
class BRouteData:
    """Class for data of the B Route."""

    instantaneous_current_r_phase: float
    instantaneous_current_t_phase: float
    instantaneous_power: float
    total_consumption: float
    total_consumption_reverse: float | None = None
    meter_timestamp: datetime | None = None
    meter_timestamp_reverse: datetime | None = None
    fault_status: bool | None = None


type BRouteConfigEntry = ConfigEntry[BRouteUpdateCoordinator]


class BRouteUpdateCoordinator(DataUpdateCoordinator[BRouteData]):
    """The B Route update coordinator."""

    device_info_data: BRouteDeviceInfo

    def __init__(
        self,
        hass: HomeAssistant,
        entry: BRouteConfigEntry,
    ) -> None:
        """Initialize."""

        self.device = entry.data[CONF_DEVICE]
        self.bid = entry.data[CONF_ID]
        self._password = entry.data[CONF_PASSWORD]

        self.api = Momonga(
            dev=self.device,
            rbid=self.bid,
            pwd=self._password,
            auto_reopen=True,
        )
        self.device_info_data = BRouteDeviceInfo()

        scan_interval = entry.options.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL)
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            config_entry=entry,
            update_interval=timedelta(seconds=scan_interval),
        )

    def _fetch_device_info(self) -> BRouteDeviceInfo:
        """Fetch static device info (called once at setup)."""
        info = BRouteDeviceInfo()
        try:
            sn = self.api.get_serial_number()
            # B-route protocol returns a fixed-width field NUL-padded; Postgres
            # rejects NUL in TEXT, so strip before exposing as state.
            info.serial_number = sn.replace("\x00", "").strip() if isinstance(sn, str) else sn
        except MomongaError:
            _LOGGER.debug("Could not fetch serial number", exc_info=True)
        time.sleep(self.api.internal_xmit_interval)

        try:
            raw = self.api.get_manufacturer_code()
            info.manufacturer_code = raw.hex().upper()
        except MomongaError:
            _LOGGER.debug("Could not fetch manufacturer code", exc_info=True)
        time.sleep(self.api.internal_xmit_interval)

        try:
            info.echonet_version = self.api.get_standard_version()
        except MomongaError:
            _LOGGER.debug("Could not fetch ECHONET version", exc_info=True)

        return info

    async def _async_setup(self) -> None:
        await self.hass.async_add_executor_job(self.api.open)
        self.device_info_data = await self.hass.async_add_executor_job(
            self._fetch_device_info
        )

    def _get_data(self) -> BRouteData:
        """Get the data from API."""
        current = self.api.get_instantaneous_current()
        data = BRouteData(
            instantaneous_current_r_phase=current["r phase current"],
            instantaneous_current_t_phase=current["t phase current"],
            instantaneous_power=self.api.get_instantaneous_power(),
            total_consumption=self.api.get_measured_cumulative_energy(),
        )

        # Reverse cumulative energy (solar sell-back)
        try:
            time.sleep(self.api.internal_xmit_interval)
            data.total_consumption_reverse = self.api.get_measured_cumulative_energy(
                reverse=True
            )
        except MomongaError:
            _LOGGER.debug("Could not fetch reverse cumulative energy", exc_info=True)

        try:
            time.sleep(self.api.internal_xmit_interval)
            fixed = self.api.get_cumulative_energy_measured_at_fixed_time()
            data.meter_timestamp = fixed["timestamp"]
        except MomongaError:
            _LOGGER.debug("Could not fetch fixed-time timestamp", exc_info=True)

        try:
            time.sleep(self.api.internal_xmit_interval)
            fixed_rev = self.api.get_cumulative_energy_measured_at_fixed_time(
                reverse=True
            )
            data.meter_timestamp_reverse = fixed_rev["timestamp"]
        except MomongaError:
            _LOGGER.debug("Could not fetch fixed-time timestamp (reverse)", exc_info=True)

        try:
            time.sleep(self.api.internal_xmit_interval)
            data.fault_status = self.api.get_fault_status()
        except MomongaError:
            _LOGGER.debug("Could not fetch fault status", exc_info=True)

        return data

    async def _async_update_data(self) -> BRouteData:
        """Update data."""
        try:
            return await self.hass.async_add_executor_job(self._get_data)
        except (MomongaError, MomongaNeedToReopen) as error:
            raise UpdateFailed(error) from error
