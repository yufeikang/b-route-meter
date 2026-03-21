"""DataUpdateCoordinator for the B Route Smart Meter integration.

Adds automatic recovery when momonga session dies.
Handles ALL exception types (not just MomongaNeedToReopen) to ensure
recovery is attempted for serial errors, OS errors, and other low-level
failures from the Wi-SUN adapter.
"""

from dataclasses import dataclass
from datetime import datetime
import logging
import time

from momonga import Momonga, MomongaError  # noqa: F401

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_DEVICE, CONF_ID, CONF_PASSWORD
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import DEFAULT_SCAN_INTERVAL, DOMAIN

_LOGGER = logging.getLogger(__name__)

MAX_REOPEN_ATTEMPTS = 5
REOPEN_BACKOFF_BASE = 5
REOPEN_BACKOFF_MAX = 60
CONSECUTIVE_FAILURES_BEFORE_PREEMPTIVE_REOPEN = 2


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
    instantaneous_voltage_r_phase: float | None = None
    instantaneous_voltage_t_phase: float | None = None
    meter_timestamp: datetime | None = None
    meter_timestamp_reverse: datetime | None = None
    fault_status: bool | None = None


type BRouteConfigEntry = ConfigEntry[BRouteUpdateCoordinator]


class BRouteUpdateCoordinator(DataUpdateCoordinator[BRouteData]):
    """The B Route update coordinator with automatic session recovery."""

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

        self.api = Momonga(dev=self.device, rbid=self.bid, pwd=self._password)
        self.device_info_data = BRouteDeviceInfo()
        self._consecutive_failures = 0

        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            config_entry=entry,
            update_interval=DEFAULT_SCAN_INTERVAL,
        )

    def _fetch_device_info(self) -> BRouteDeviceInfo:
        """Fetch static device info (called once at setup)."""
        info = BRouteDeviceInfo()
        try:
            info.serial_number = self.api.get_serial_number()
        except Exception:
            _LOGGER.debug("Could not fetch serial number", exc_info=True)
        time.sleep(self.api.internal_xmit_interval)

        try:
            raw = self.api.get_manufacturer_code()
            info.manufacturer_code = raw.hex().upper()
        except Exception:
            _LOGGER.debug("Could not fetch manufacturer code", exc_info=True)
        time.sleep(self.api.internal_xmit_interval)

        try:
            info.echonet_version = self.api.get_standard_version()
        except Exception:
            _LOGGER.debug("Could not fetch ECHONET version", exc_info=True)

        return info

    async def _async_setup(self) -> None:
        await self.hass.async_add_executor_job(self.api.open)
        self.device_info_data = await self.hass.async_add_executor_job(
            self._fetch_device_info
        )

    def _reopen(self) -> None:
        """Close and reopen the momonga session."""
        _LOGGER.warning("Attempting to reopen momonga session")
        try:
            self.api.close()
        except Exception:
            _LOGGER.debug("Error closing momonga (ignored)", exc_info=True)

        # Recreate the Momonga instance to ensure clean state
        self.api = Momonga(dev=self.device, rbid=self.bid, pwd=self._password)
        self.api.open()
        _LOGGER.info("Momonga session reopened successfully")

    def _get_data(self) -> BRouteData:
        """Get the data from API."""
        current = self.api.get_instantaneous_current()
        data = BRouteData(
            instantaneous_current_r_phase=current["r phase current"],
            instantaneous_current_t_phase=current["t phase current"],
            instantaneous_power=self.api.get_instantaneous_power(),
            total_consumption=self.api.get_measured_cumulative_energy(),
        )

        # Voltage (R-phase and T-phase)
        try:
            time.sleep(self.api.internal_xmit_interval)
            voltage = self.api.get_instantaneous_voltage()
            data.instantaneous_voltage_r_phase = voltage["r phase voltage"]
            data.instantaneous_voltage_t_phase = voltage["t phase voltage"]
        except MomongaError:
            _LOGGER.debug("Could not fetch instantaneous voltage", exc_info=True)

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

    def _get_data_with_recovery(self) -> BRouteData:
        """Get data, automatically recovering from session failures.

        Catches ALL exceptions (not just MomongaNeedToReopen) because the
        underlying Wi-SUN adapter can throw serial.SerialException, OSError,
        or other non-MomongaError exceptions when the connection drops.
        """
        # If we've been failing repeatedly, preemptively reopen before even trying
        if self._consecutive_failures >= CONSECUTIVE_FAILURES_BEFORE_PREEMPTIVE_REOPEN:
            _LOGGER.info(
                "Preemptive reopen after %d consecutive failures",
                self._consecutive_failures,
            )
            try:
                self._reopen()
            except Exception:
                _LOGGER.warning("Preemptive reopen failed, will try data fetch anyway", exc_info=True)

        try:
            data = self._get_data()
            self._consecutive_failures = 0
            return data
        except Exception as initial_err:
            _LOGGER.warning(
                "Data fetch failed (%s: %s). Will attempt up to %d reopens.",
                type(initial_err).__name__,
                initial_err,
                MAX_REOPEN_ATTEMPTS,
            )

            last_error: Exception = initial_err
            for attempt in range(1, MAX_REOPEN_ATTEMPTS + 1):
                backoff = min(REOPEN_BACKOFF_BASE * (2 ** (attempt - 1)), REOPEN_BACKOFF_MAX)
                time.sleep(backoff)
                try:
                    self._reopen()
                    data = self._get_data()
                    _LOGGER.info(
                        "Recovery successful on attempt %d/%d",
                        attempt,
                        MAX_REOPEN_ATTEMPTS,
                    )
                    self._consecutive_failures = 0
                    return data
                except Exception as err:
                    last_error = err
                    _LOGGER.warning(
                        "Reopen attempt %d/%d failed (%s: %s)",
                        attempt,
                        MAX_REOPEN_ATTEMPTS,
                        type(err).__name__,
                        err,
                    )

            self._consecutive_failures += 1
            raise UpdateFailed(
                f"Failed to recover after {MAX_REOPEN_ATTEMPTS} attempts"
                + f" ({self._consecutive_failures} consecutive poll failures)"
            ) from last_error

    async def _async_update_data(self) -> BRouteData:
        """Update data with automatic session recovery."""
        try:
            return await self.hass.async_add_executor_job(
                self._get_data_with_recovery
            )
        except UpdateFailed:
            raise
        except Exception as error:
            self._consecutive_failures += 1
            raise UpdateFailed(
                f"Unexpected error ({type(error).__name__}): {error}"
            ) from error
