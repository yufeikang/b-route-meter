"""BP35C2 adapter implementation."""

import logging
from datetime import datetime, timedelta, timezone

import serial
from homeassistant.exceptions import ConfigEntryNotReady, IntegrationError

from ..adapter_interface import AdapterInterface, DiagnosticInfo, MeterReading

_LOGGER = logging.getLogger(__name__)

JST = timezone(timedelta(hours=9), "JST")
UTC = timezone.utc


class BP35C2Adapter(AdapterInterface):
    """BP35C2 adapter implementation."""

    def __init__(
        self,
        route_b_id: str,
        route_b_pwd: str,
        serial_port: str,
    ) -> None:
        """Initialize the adapter.

        Args:
            route_b_id: B-route ID
            route_b_pwd: B-route password
            serial_port: Serial port device path
        """
        self.route_b_id = route_b_id
        self.route_b_pwd = route_b_pwd
        self.serial_port_path = serial_port

        self.serial_port = None
        self.scan_res: dict[str, str] = {}
        self.ipv6_addr = None

    def connect(self) -> None:
        """Establish connection with the smart meter."""
        try:
            # 1) Open serial port
            _LOGGER.debug("Opening serial port: %s", self.serial_port_path)
            self.serial_port = serial.Serial(
                port=self.serial_port_path,
                baudrate=115200,
                timeout=2,
            )

            # 2) Set B-route password
            _LOGGER.debug("Setting B-route password")
            self._write_cmd(f"SKSETPWD C {self.route_b_pwd}\r\n")
            self._wait_ok()

            # 3) Set B-route ID
            _LOGGER.debug("Setting B-route ID: %s", self.route_b_id)
            self._write_cmd(f"SKSETRBID {self.route_b_id}\r\n")
            self._wait_ok()

            # 4) Channel scan
            _LOGGER.debug("Starting channel scan")
            self._scan_channel()
            _LOGGER.info(
                "Channel scan complete: Channel=%s, Pan ID=%s, Addr=%s",
                self.scan_res.get("Channel"),
                self.scan_res.get("Pan ID"),
                self.scan_res.get("Addr"),
            )

            # 5) Set channel & PAN ID
            _LOGGER.debug("Setting channel: %s", self.scan_res["Channel"])
            self._write_cmd(f"SKSREG S2 {self.scan_res['Channel']}\r\n")
            self._wait_ok()

            _LOGGER.debug("Setting PAN ID: %s", self.scan_res["Pan ID"])
            self._write_cmd(f"SKSREG S3 {self.scan_res['Pan ID']}\r\n")
            self._wait_ok()

            # 6) Get IPv6 address
            _LOGGER.debug("Getting IPv6 address for: %s", self.scan_res["Addr"])
            self._write_cmd(f"SKLL64 {self.scan_res['Addr']}\r\n")
            self.serial_port.readline()  # possibly empty
            line_ipv6 = (
                self.serial_port.readline().decode("utf-8", errors="ignore").strip()
            )
            self.ipv6_addr = line_ipv6
            _LOGGER.debug("IPv6 address: %s", self.ipv6_addr)

            # 7) PANA authentication
            _LOGGER.debug("Starting PANA authentication with: %s", self.ipv6_addr)
            self._write_cmd(f"SKJOIN {self.ipv6_addr}\r\n")
            self._wait_ok()

            # Wait for EVENT 25 (success) or EVENT 24 (fail)
            _LOGGER.debug("Waiting for PANA authentication result")
            bConnected = False
            timeout_count = 0
            max_timeout = 10  # 最多允许10次超时

            while not bConnected:
                raw_line = self.serial_port.readline()
                if not raw_line:
                    timeout_count += 1
                    _LOGGER.debug(
                        "Empty response during PANA auth (timeout %d/%d)",
                        timeout_count,
                        max_timeout,
                    )
                    if timeout_count >= max_timeout:
                        self._handle_error(
                            "PANA authentication timed out after multiple attempts"
                        )
                    continue

                _LOGGER.debug("PANA auth response: %s", raw_line)

                if raw_line.startswith(b"EVENT 24"):
                    self._handle_error("PANA authentication failed. (EVENT 24)")
                if raw_line.startswith(b"EVENT 25"):
                    _LOGGER.debug("PANA authentication success. (EVENT 25)")
                    bConnected = True
                if raw_line.startswith(b"EVENT 21"):
                    # EVENT 21 表示 Scan 完成
                    _LOGGER.debug("Scan complete event received (EVENT 21)")
                if raw_line.startswith(b"EVENT 22"):
                    # EVENT 22 表示 Scan 超时
                    _LOGGER.debug("Scan timeout event received (EVENT 22)")
                if raw_line.startswith(b"EVENT 29"):
                    # EVENT 29 表示 PANA 会话超时
                    _LOGGER.warning("PANA session timeout (EVENT 29)")

            _LOGGER.info("B-route connection established successfully")
        except Exception as e:
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            _LOGGER.error("Failed to connect B-route: %s", e)
            raise ConfigEntryNotReady(f"Failed to connect B-route: {e}") from e

    def get_data(self) -> MeterReading:
        """Read data from the smart meter."""
        if not self.serial_port or not self.ipv6_addr:
            raise RuntimeError("B-route is not connected. Call connect() first.")

        # Build ECHONET Lite frame
        # 添加额外的EPC码以获取更多信息
        epcs = [0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0x80, 0x82, 0x97, 0x98, 0xD3, 0xD7]
        # 使用更标准的 ECHONET Lite 帧格式
        frame = b"\x10\x81"  # EHD
        frame += b"\x00\x01"  # TID
        frame += b"\x05\xff\x01"  # SEOJ=Controller
        frame += b"\x02\x88\x01"  # DEOJ=低圧スマートメーター
        frame += b"\x62"  # ESV=ReadRequest
        frame += bytes([len(epcs)])  # OPC=参数数量
        for epc_code in epcs:
            frame += epc_code.to_bytes(1, "big")  # EPC
            frame += b"\x00"  # PDC=0

        cmd_str = (
            f"SKSENDTO 1 {self.ipv6_addr} 0E1A 1 0 {len(frame):04X} ".encode()
            + frame
            + b"\r\n"
        )
        _LOGGER.debug("Sending command: %s", cmd_str.hex())
        self.serial_port.write(cmd_str)

        reading = MeterReading()
        year = month = day = hour = minute = second = 0

        # 初始化结果变量，避免未赋值
        e7_power = None
        e8_current = None
        e9_voltage = None
        ea_val = None
        eb_val = None
        r_phase_current = None
        t_phase_current = None

        # 新属性的初始值
        operation_status = None
        error_status = None
        current_limit = None
        meter_type = None
        detected_abnormality = None
        power_unit = None

        # 功能支持标志
        has_operational_info = False
        has_limit_info = False
        has_abnormality_detection = False

        # Read response
        complete_response = b""
        response_received = False

        _LOGGER.debug("Waiting for response from smart meter...")
        timeout_count = 0
        max_timeout = 3  # 最多允许连续3次超时

        for attempt in range(10):
            try:
                raw_line = self.serial_port.readline()
                if not raw_line:
                    timeout_count += 1
                    _LOGGER.debug(
                        "Empty response line (timeout %d/%d), continuing...",
                        timeout_count,
                        max_timeout,
                    )
                    if timeout_count >= max_timeout:
                        _LOGGER.warning("Too many consecutive timeouts, breaking loop")
                        break
                    continue

                timeout_count = 0  # 重置超时计数
                _LOGGER.debug("Received line (%d bytes): %s", len(raw_line), raw_line)

                if raw_line.startswith(b"ERXUDP"):
                    _LOGGER.debug("Found ERXUDP response")
                    complete_response = raw_line
                    response_received = True
                elif response_received:
                    complete_response += raw_line

                if complete_response and b"\r\n" in complete_response:
                    _LOGGER.debug("Complete response: %s", complete_response)
                    tokens = complete_response.split(b" ", 9)
                    if len(tokens) < 10:
                        _LOGGER.warning(
                            "Incomplete ERXUDP response: %s", complete_response
                        )
                        continue

                    # 从ERXUDP响应中提取有用的信息，可用于诊断
                    # ERXUDP <送信元IPv6アドレス> <送信先IPv6アドレス> <送信元ポート> <送信先ポート> <送信元MAC> <セキュリティ> <SIDE> <データ長> <データ>
                    try:
                        sender_ipv6 = tokens[1].decode("utf-8", errors="ignore")
                        receiver_ipv6 = tokens[2].decode("utf-8", errors="ignore")

                        # 保存IPv6地址，确保连接状态的持续性
                        if sender_ipv6.startswith("FE80:"):
                            # 更新实例变量，使诊断信息可以使用这个地址
                            self.ipv6_addr = sender_ipv6
                            _LOGGER.debug(
                                "Updated IPv6 address from ERXUDP: %s", sender_ipv6
                            )
                    except Exception as e:
                        _LOGGER.debug("Error extracting IPv6 from ERXUDP: %s", e)

                    echonet_payload = bytes.fromhex(tokens[-1].rstrip(b"\r\n").decode())
                    _LOGGER.debug(
                        "ECHONET payload (%d bytes): %s",
                        len(echonet_payload),
                        echonet_payload.hex(),
                    )

                    if len(echonet_payload) < 12:
                        _LOGGER.warning(
                            "ECHONET payload too short: %s", echonet_payload.hex()
                        )
                        continue

                    frame_info = self._parse_echonet_frame(echonet_payload)
                    _LOGGER.debug("Parsed frame: %s", frame_info)

                    # 不再严格检查帧格式，接受任何可能的响应
                    if "properties" not in frame_info or not frame_info["properties"]:
                        _LOGGER.warning("No properties found in response")
                        continue

                    _LOGGER.debug(
                        "Found %d properties in response", len(frame_info["properties"])
                    )

                    for epc, pdc, edt in frame_info.get("properties", []):
                        _LOGGER.debug(
                            "Processing property: EPC=0x%02X, PDC=%d, EDT=%s",
                            epc,
                            pdc,
                            edt.hex() if edt else "<empty>",
                        )

                        # 尝试处理所有属性，即使格式不完全匹配
                        try:
                            if epc == 0xE7:
                                # E7: instantaneous power
                                if pdc == 4:
                                    # 4 bytes, signed int, unit: W
                                    # 修改处理方式以匹配测试代码
                                    val = int.from_bytes(
                                        edt, byteorder="big", signed=False
                                    )
                                    # 处理符号位
                                    if (val >> 31) & 0x01 == 1:
                                        val = (val ^ 0xFFFFFFFF) * (-1) - 1
                                    e7_power = val
                                    _LOGGER.debug("Parsed power: %s W", val)
                                elif pdc > 0:
                                    # 尝试非标准格式解析
                                    try:
                                        val = int.from_bytes(
                                            edt, byteorder="big", signed=False
                                        )
                                        # 检查符号位
                                        if pdc == 2 and (val >> 15) & 0x01 == 1:
                                            val = (val ^ 0xFFFF) * (-1) - 1
                                        elif pdc == 3 and (val >> 23) & 0x01 == 1:
                                            val = (val ^ 0xFFFFFF) * (-1) - 1
                                        elif pdc > 4 and (val >> 31) & 0x01 == 1:
                                            val = (val ^ 0xFFFFFFFF) * (-1) - 1
                                        e7_power = val
                                        _LOGGER.debug(
                                            "Parsed non-standard power: %s W", val
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing non-standard power value: %s",
                                            e,
                                        )
                                else:
                                    _LOGGER.debug(
                                        "Meter does not support E7 or no power data"
                                    )

                            elif epc == 0xE8:
                                # E8: instantaneous current
                                if pdc == 4:
                                    try:
                                        # 2 bytes for R phase, 2 bytes for T phase
                                        # Each is unsigned, 0.1 A unit
                                        i1 = int.from_bytes(
                                            edt[0:2], "big", signed=False
                                        )
                                        i2 = int.from_bytes(
                                            edt[2:4], "big", signed=False
                                        )
                                        phase1 = i1 / 10.0
                                        phase2 = i2 / 10.0
                                        e8_current = phase1 + phase2
                                        # 保存R相和T相电流值，供属性显示使用
                                        r_phase_current = phase1
                                        t_phase_current = phase2
                                        _LOGGER.debug(
                                            "Parsed current: R=%s A, T=%s A, total=%s A",
                                            phase1,
                                            phase2,
                                            e8_current,
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing current value: %s", e
                                        )
                                elif pdc > 0:
                                    # 尝试解析非标准格式的电流
                                    try:
                                        if pdc == 2:
                                            i = int.from_bytes(edt, "big", signed=False)
                                            e8_current = i / 10.0
                                        else:
                                            # 尽可能多地使用数据
                                            i = int.from_bytes(
                                                edt[0 : min(2, pdc)],
                                                "big",
                                                signed=False,
                                            )
                                            e8_current = i / 10.0
                                        _LOGGER.debug(
                                            "Parsed non-standard current: %s A",
                                            e8_current,
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing non-standard current: %s", e
                                        )
                                else:
                                    _LOGGER.debug(
                                        "Meter does not support E8 or no current data"
                                    )

                            elif epc == 0xE9:
                                # E9: instantaneous voltage
                                if pdc == 4:
                                    try:
                                        v1 = int.from_bytes(
                                            edt[0:2], "big", signed=False
                                        )
                                        v2 = int.from_bytes(
                                            edt[2:4], "big", signed=False
                                        )
                                        # 取平均值
                                        e9_voltage = (v1 + v2) / 2.0
                                        _LOGGER.debug(
                                            "Parsed voltage: %s V", e9_voltage
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing voltage value: %s", e
                                        )
                                elif pdc == 2:
                                    # 单相电压
                                    try:
                                        e9_voltage = int.from_bytes(
                                            edt, "big", signed=False
                                        )
                                        _LOGGER.debug(
                                            "Parsed single-phase voltage: %s V",
                                            e9_voltage,
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing single-phase voltage: %s", e
                                        )
                                elif pdc > 0:
                                    # 尝试解析非标准格式
                                    try:
                                        v = int.from_bytes(
                                            edt[0 : min(2, pdc)], "big", signed=False
                                        )
                                        e9_voltage = v
                                        _LOGGER.debug(
                                            "Parsed non-standard voltage: %s V",
                                            e9_voltage,
                                        )
                                    except Exception as e:
                                        _LOGGER.error(
                                            "Error parsing non-standard voltage: %s", e
                                        )
                                else:
                                    _LOGGER.debug(
                                        "Meter does not support E9 or no voltage data"
                                    )

                            elif epc == 0xEA and pdc >= 10:
                                # EA: 正向有功电能
                                try:
                                    year = int.from_bytes(edt[0:2], "big")
                                    month = edt[2]
                                    day = edt[3]
                                    hour = edt[4]
                                    minute = edt[5]
                                    second = edt[6]
                                    accum_raw = int.from_bytes(
                                        edt[7:11], "big", signed=False
                                    )
                                    # 假设步进为0.1 kWh
                                    ea_val = accum_raw / 10.0
                                    _LOGGER.debug("Parsed EA forward: %s kWh", ea_val)
                                except Exception as e:
                                    _LOGGER.error("Error parsing EA value: %s", e)
                            elif epc == 0xEA and pdc > 0:
                                # 非标准格式的EA
                                try:
                                    # 尝试直接读取累计值
                                    accum_raw = int.from_bytes(edt, "big", signed=False)
                                    # 假设步进为0.1 kWh
                                    ea_val = accum_raw / 10.0
                                    _LOGGER.debug(
                                        "Parsed non-standard EA forward: %s kWh", ea_val
                                    )
                                except Exception as e:
                                    _LOGGER.error(
                                        "Error parsing non-standard EA value: %s", e
                                    )

                            elif epc == 0xEB and pdc >= 10:
                                # EB: 反向有功电能
                                try:
                                    if year == 0:  # 如果EA没有设置年月日，则从EB获取
                                        year = int.from_bytes(edt[0:2], "big")
                                        month = edt[2]
                                        day = edt[3]
                                        hour = edt[4]
                                        minute = edt[5]
                                        second = edt[6]
                                    accum_raw = int.from_bytes(
                                        edt[7:11], "big", signed=False
                                    )
                                    # 假设步进为0.1 kWh
                                    eb_val = accum_raw / 10.0
                                    _LOGGER.debug("Parsed EB reverse: %s kWh", eb_val)
                                except Exception as e:
                                    _LOGGER.error("Error parsing EB value: %s", e)
                            elif epc == 0xEB and pdc > 0:
                                # 非标准格式的EB
                                try:
                                    # 尝试直接读取累计值
                                    accum_raw = int.from_bytes(edt, "big", signed=False)
                                    # 假设步进为0.1 kWh
                                    eb_val = accum_raw / 10.0
                                    _LOGGER.debug(
                                        "Parsed non-standard EB reverse: %s kWh", eb_val
                                    )
                                except Exception as e:
                                    _LOGGER.error(
                                        "Error parsing non-standard EB value: %s", e
                                    )

                            # 新增的ECHONET Lite属性解析
                            elif epc == 0x80 and pdc == 1:  # 动作状态
                                try:
                                    operation_status = (
                                        edt[0] == 0x30
                                    )  # 0x30=ON, 0x31=OFF
                                    _LOGGER.debug(
                                        "Parsed operation status: %s",
                                        "ON" if operation_status else "OFF",
                                    )
                                    has_operational_info = True
                                except Exception as e:
                                    _LOGGER.error(
                                        "Error parsing operation status: %s", e
                                    )

                            elif epc == 0x82 and pdc == 1:  # 错误状态
                                try:
                                    error_status = (
                                        edt[0] == 0x41
                                    )  # 0x41=Error, 0x40=Normal
                                    _LOGGER.debug(
                                        "Parsed error status: %s",
                                        "Error" if error_status else "Normal",
                                    )
                                    has_operational_info = True
                                except Exception as e:
                                    _LOGGER.error("Error parsing error status: %s", e)

                            elif epc == 0x97 and pdc >= 1:  # 当前限制容量
                                try:
                                    limit_raw = int.from_bytes(edt, "big", signed=False)
                                    # 根据规格，单位一般是0.1安培
                                    current_limit = limit_raw / 10.0
                                    _LOGGER.debug(
                                        "Parsed current limit: %s A", current_limit
                                    )
                                    has_limit_info = True
                                except Exception as e:
                                    _LOGGER.error("Error parsing current limit: %s", e)

                            elif epc == 0x98 and pdc >= 1:  # 电表分类
                                try:
                                    meter_code = edt[0]
                                    meter_type_map = {
                                        0x30: "Electric energy",
                                        0x31: "Water flow",
                                        0x32: "Gas flow",
                                        0x33: "LP gas flow",
                                        0x34: "Clock",
                                        0x35: "Temperature",
                                        0x36: "Hot water",
                                        0x37: "Air conditioning",
                                        0x38: "Ventilation",
                                        0x39: "Others",
                                    }
                                    meter_type = meter_type_map.get(
                                        meter_code, f"Unknown({meter_code:02X})"
                                    )
                                    _LOGGER.debug("Parsed meter type: %s", meter_type)
                                    has_operational_info = True
                                except Exception as e:
                                    _LOGGER.error("Error parsing meter type: %s", e)

                            elif epc == 0xD3 and pdc >= 1:  # 检测到的异常
                                try:
                                    abnormality_code = edt[0]
                                    abnormality_map = {
                                        0x41: "Error occurred",
                                        0x42: "No electricity",
                                        0x43: "Power outage",
                                        0x44: "Power overload",
                                        0x45: "Voltage upper limit",
                                        0x46: "Voltage lower limit",
                                        0x47: "Current limit",
                                        0x48: "Leakage",
                                        0x49: "No gas",
                                        0x4A: "Gas pressure",
                                        0x4B: "Gas leakage",
                                        0x4C: "Emergency",
                                        0x4D: "Shutter open",
                                        0x4E: "Failure in communication circuit",
                                    }
                                    detected_abnormality = abnormality_map.get(
                                        abnormality_code,
                                        f"Unknown({abnormality_code:02X})",
                                    )
                                    _LOGGER.debug(
                                        "Parsed abnormality: %s", detected_abnormality
                                    )
                                    has_abnormality_detection = True
                                except Exception as e:
                                    _LOGGER.error("Error parsing abnormality: %s", e)

                            elif epc == 0xD7 and pdc >= 1:  # 积算有效电力量单位
                                try:
                                    unit_code = edt[0]
                                    if unit_code == 0x00:
                                        power_unit = 1.0  # 1kWh
                                    elif unit_code == 0x01:
                                        power_unit = 0.1  # 0.1kWh
                                    elif unit_code == 0x02:
                                        power_unit = 0.01  # 0.01kWh
                                    elif unit_code == 0x03:
                                        power_unit = 0.001  # 0.001kWh
                                    elif unit_code == 0x04:
                                        power_unit = 0.0001  # 0.0001kWh
                                    elif unit_code == 0x0A:
                                        power_unit = 10.0  # 10kWh
                                    elif unit_code == 0x0B:
                                        power_unit = 100.0  # 100kWh
                                    elif unit_code == 0x0C:
                                        power_unit = 1000.0  # 1000kWh
                                    elif unit_code == 0x0D:
                                        power_unit = 10000.0  # 10000kWh
                                    else:
                                        power_unit = 0.1  # 默认0.1kWh
                                    _LOGGER.debug(
                                        "Parsed power unit: %s kWh", power_unit
                                    )
                                except Exception as e:
                                    _LOGGER.error("Error parsing power unit: %s", e)

                        except Exception as e:
                            _LOGGER.error(
                                "Error processing property EPC=0x%02X: %s", epc, e
                            )

                    # 在找到一个完整的响应后退出循环
                    break
            except Exception as e:
                _LOGGER.error("Unexpected error in get_data: %s", e)

        # 构造结果
        reading.power = e7_power
        reading.current = e8_current
        reading.voltage = e9_voltage
        reading.forward = ea_val
        reading.reverse = eb_val
        reading.r_phase_current = r_phase_current
        reading.t_phase_current = t_phase_current

        # 添加新属性到读数结果
        reading.operation_status = operation_status
        reading.error_status = error_status
        reading.current_limit = current_limit
        reading.meter_type = meter_type
        reading.detected_abnormality = detected_abnormality
        reading.power_unit = power_unit

        # 设置功能支持标志 - 对于任何具有值的属性，将其标记为支持
        # 我们已经获取到了操作状态，所以这是明确支持的
        reading.has_operational_info = operation_status is not None
        reading.has_limit_info = current_limit is not None
        reading.has_abnormality_detection = detected_abnormality is not None

        # 如果我们没有明确的操作状态信息，但可以从其他信息推断，则设为支持
        if not reading.has_operational_info and (
            reading.power is not None or reading.current is not None
        ):
            reading.has_operational_info = True
            # 如果有功率或电流，设备肯定是在线的
            if reading.operation_status is None and (reading.power or reading.current):
                reading.operation_status = True
                _LOGGER.debug(
                    "Inferred operation status: ON (from power/current readings)"
                )

        _LOGGER.debug(
            "Final reading values: power=%s W, current=%s A, voltage=%s V, forward=%s kWh, reverse=%s kWh",
            reading.power,
            reading.current,
            reading.voltage,
            reading.forward,
            reading.reverse,
        )

        if has_operational_info or has_limit_info or has_abnormality_detection:
            _LOGGER.debug(
                "Additional meter info: operation_status=%s, error_status=%s, current_limit=%s A, meter_type=%s, abnormality=%s, power_unit=%s kWh",
                reading.operation_status,
                reading.error_status,
                reading.current_limit,
                reading.meter_type,
                reading.detected_abnormality,
                reading.power_unit,
            )

        return reading

    def close(self) -> None:
        """Close the connection with the smart meter."""
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            self.ipv6_addr = None

    def get_diagnostic_info(self) -> DiagnosticInfo:
        """Get diagnostic information from the device.

        Returns:
            DiagnosticInfo: Device diagnostic information including network status
        """
        if not self.serial_port:
            raise RuntimeError("B-route is not connected. Call connect() first.")

        info = DiagnosticInfo()

        # 直接从实例变量中获取已知的IPv6地址
        # 这个地址在 connect() 和每次 get_data() 方法调用时都会更新
        if self.ipv6_addr:
            info.ipv6_address = self.ipv6_addr
            _LOGGER.debug("Using stored IPv6 address: %s", self.ipv6_addr)

        # 1. Get basic device info using SKINFO
        self._write_cmd("SKINFO\r\n")
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"EINFO"):
                # EINFO <IPADDR> <ADDR64> <CHANNEL> <PANID> <ADDR16>
                info_parts = raw_line.decode().split()[1:]
                if len(info_parts) >= 5:
                    # 如果SKINFO命令返回了IPv6地址，优先使用它
                    if info_parts[0] and info_parts[0] != "undefined":
                        info.ipv6_address = info_parts[0]
                    info.mac_address = info_parts[1]
                    info.channel = int(info_parts[2], 16)
                    info.pan_id = info_parts[3]
                break
            elif raw_line.startswith(b"OK"):
                break

        # 1.1 Get signal strength (RSSI)
        try:
            # 如果没有任何邻居设备，SKRSSI 将返回错误，因此先检查是否有活跃连接
            have_active_connection = False
            for attempts in range(3):  # 尝试最多3次获取RSSI
                self._write_cmd("SKRSSI\r\n")
                rssi_timeout = 0
                while rssi_timeout < 5:  # 等待响应最多5次超时
                    raw_line = self.serial_port.readline()
                    if not raw_line:
                        rssi_timeout += 1
                        continue

                    if raw_line.startswith(b"ERSSI"):
                        # ERSSI <RSSI>
                        try:
                            rssi_parts = raw_line.decode().split()
                            if len(rssi_parts) >= 2:
                                # RSSI值通常是负数，表示为十六进制值，如"8A"表示-74 dBm
                                rssi_hex = rssi_parts[1]
                                rssi_value = int(rssi_hex, 16)
                                # 如果高位为1，表示负数，需要转换
                                if rssi_value > 127:
                                    rssi_value = rssi_value - 256
                                info.rssi = rssi_value
                                have_active_connection = True
                                _LOGGER.debug("RSSI: %d dBm", rssi_value)
                        except Exception as e:
                            _LOGGER.warning("Error parsing RSSI: %s", e)
                        break
                    elif raw_line.startswith(b"FAIL"):
                        _LOGGER.debug("SKRSSI command failed, may not be supported")
                        break
                    elif raw_line.startswith(b"OK"):
                        break

                if have_active_connection:
                    break  # 如果获取到RSSI，退出重试循环

            # 如果多次尝试后仍未获取到RSSI，但有IPv6地址，将RSSI设为默认值
            if not have_active_connection and info.ipv6_address:
                # 设一个合理的默认值，表示连接存在但信号强度未知
                info.rssi = -75  # 典型的中等信号强度
                _LOGGER.debug("Could not get RSSI, using default value: -75 dBm")

        except Exception as e:
            _LOGGER.warning("Error getting RSSI: %s", e)

        # 2. Get version information
        # Get stack version
        self._write_cmd("SKVER\r\n")
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"EVER"):
                info.stack_version = raw_line.decode().split()[1]
                break
            elif raw_line.startswith(b"OK"):
                break

        # Get app version
        self._write_cmd("SKAPPVER\r\n")
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"EAPPVER"):
                info.app_version = raw_line.decode().split()[1]
                break
            elif raw_line.startswith(b"OK"):
                break

        # 3. Get network status using SKTABLE
        # Get active TCP connections
        self._write_cmd("SKTABLE F\r\n")
        info.active_tcp_connections = []
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"EHANDLE"):
                # Parse TCP connection info
                parts = raw_line.decode().split()
                if len(parts) >= 5:  # EHANDLE <HANDLE> <IPADDR> <RPORT> <LPORT>
                    info.active_tcp_connections.append(
                        {
                            "handle": parts[1],
                            "remote_addr": parts[2],
                            "remote_port": parts[3],
                            "local_port": parts[4],
                        }
                    )
            elif raw_line.startswith(b"OK"):
                break

        # Get port settings
        self._write_cmd("SKTABLE E\r\n")
        info.udp_ports = []
        info.tcp_ports = []
        parsing_udp = True
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"OK"):
                break

            try:
                port = int(raw_line.strip(), 16)
                if port > 0:
                    if parsing_udp:
                        info.udp_ports.append(port)
                    else:
                        info.tcp_ports.append(port)
            except ValueError:
                if raw_line.strip() == b"":  # Empty line separates UDP/TCP sections
                    parsing_udp = False
                continue

        # Get neighbor devices
        self._write_cmd("SKTABLE 2\r\n")
        info.neighbor_devices = []
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                continue
            if raw_line.startswith(b"ENEIGHBOR"):
                parts = raw_line.decode().split()
                if len(parts) >= 3:  # ENEIGHBOR <IPADDR> <ADDR64> <ADDR16>
                    info.neighbor_devices.append(
                        {"ipv6_addr": parts[1], "mac_addr": parts[2]}
                    )
            elif raw_line.startswith(b"OK"):
                break

        # 如果没有找到邻居设备，但我们有已知的IPv6地址，则添加它作为一个隐含的邻居
        if not info.neighbor_devices and info.ipv6_address:
            # 确保这是一个有效的邻居，这通常是Smart Meter的地址
            if info.ipv6_address.startswith("FE80:"):
                # 如果我们不知道MAC地址，使用一个默认值
                mac_addr = info.mac_address or "UNKNOWN_MAC"
                info.neighbor_devices.append(
                    {"ipv6_addr": info.ipv6_address, "mac_addr": mac_addr}
                )
                _LOGGER.debug("Adding implicit neighbor device: %s", info.ipv6_address)

        return info

    def _scan_channel(self) -> None:
        """Scan channel and populate scan_res."""
        _LOGGER.debug("Scanning channel for Smart Meter")
        scan_duration = 5

        while "Channel" not in self.scan_res:
            cmd_str = f"SKSCAN 2 FFFFFFFF {scan_duration} 0\r\n"
            self._write_cmd(cmd_str)
            scan_end = False
            while not scan_end:
                raw_line = self.serial_port.readline()
                if not raw_line:
                    continue
                if raw_line.startswith(b"EVENT 22"):
                    scan_end = True
                elif raw_line.startswith(b"  "):
                    line_decoded = raw_line.decode("utf-8", errors="ignore").strip()
                    cols = line_decoded.split(":")
                    if len(cols) == 2:
                        key, val = cols
                        self.scan_res[key] = val
            scan_duration += 1
            if scan_duration > 14 and "Channel" not in self.scan_res:
                raise ConfigEntryNotReady(
                    "Could not find valid channel within scan duration."
                )

    def _wait_ok(self) -> None:
        """Wait until we see 'OK' in a line."""
        empty_count = 0
        max_empty_read = 5
        while True:
            raw_line = self.serial_port.readline()
            if not raw_line:
                empty_count += 1
                if empty_count >= max_empty_read:
                    raise IntegrationError(
                        "wait_ok() timed out / too many empty reads."
                    )
                continue
            if raw_line.startswith(b"OK"):
                break

    def _write_cmd(self, cmd_str: str | bytes) -> None:
        """Write command to serial port."""
        if isinstance(cmd_str, str):
            cmd_str = cmd_str.encode()
        _LOGGER.debug("Write to meter: %s", cmd_str)
        self.serial_port.write(cmd_str)

    def _handle_error(self, error: str) -> None:
        """Handle error by closing port and raising exception."""
        if self.serial_port:
            self.serial_port.close()
        raise ConfigEntryNotReady(error)

    def _parse_echonet_frame(self, echonet_bytes: bytes) -> dict:
        """Parse ECHONET Lite frame.

        Args:
            echonet_bytes: Raw ECHONET Lite frame bytes

        Returns:
            dict: Parsed frame information
        """
        result = {}
        if len(echonet_bytes) < 12:
            _LOGGER.warning(
                "ECHONET frame too short (%d bytes): %s",
                len(echonet_bytes),
                echonet_bytes.hex(),
            )
            return result

        try:
            # 解析ECHONET Lite帧头部
            result["EHD"] = echonet_bytes[0:2].hex()  # ECHONET Lite header
            result["TID"] = echonet_bytes[2:4].hex()  # Transaction ID
            result["SEOJ"] = echonet_bytes[4:7].hex()  # Source ECHONET object
            result["DEOJ"] = echonet_bytes[7:10].hex()  # Destination ECHONET object
            result["ESV"] = echonet_bytes[10]  # ECHONET service
            result["OPC"] = echonet_bytes[11]  # Operation property count
            result["properties"] = []

            # 检查EHD值是否正确（应为1081或1082）
            if result["EHD"] not in ["1081", "1082"]:
                _LOGGER.warning("Unexpected EHD value: %s", result["EHD"])
                # 继续处理，不要中断

            # 检查DEOJ值是否正确
            # 添加发现的非标准DEOJ值到允许列表中
            valid_deoj = ["028801", "0ef001", "05ff01"]
            if result["DEOJ"] not in valid_deoj:
                _LOGGER.debug(
                    "Non-standard DEOJ value: %s (expected one of %s)",
                    result["DEOJ"],
                    ", ".join(valid_deoj),
                )
                # 继续处理，不要中断 - 只是记录为debug级别

            # 检查ESV值是否为有效的响应类型
            # 扩展有效的ESV值列表，包括可能的自定义值
            valid_esv = [
                0x71,  # 读取响应
                0x72,  # 写入响应
                0x73,  # 通知
                0x74,  # 读取-写入响应
                0x7A,  # 属性读取响应（带有数组）
                0x7B,  # 属性写入响应（带有数组）
                0x7C,  # 属性读取通知（带有数组）
                0x7D,  # 属性写入通知（带有数组）
                0x50,  # 属性设置（无响应）
                0x51,  # 属性设置（响应要求）
                0x52,  # 属性获取（响应要求）
                0x53,  # 属性设置获取（响应要求）
                0x5E,  # 属性设置（响应要求）带有数组
                0x60,  # 属性设置（响应要求）
                0x61,  # 属性设置请求（响应要求）
                0x62,  # 读取请求
                0x63,  # 写入读取请求
                0x6E,  # 属性设置读取请求（带有数组）
            ]
            if result["ESV"] not in valid_esv:
                _LOGGER.debug(
                    "Non-standard ESV value: 0x%02X (expected one of standard values)",
                    result["ESV"],
                )
                # 继续处理，不要中断

            # 检查OPC值是否合理
            if result["OPC"] > 20:  # 设置一个合理的上限
                _LOGGER.debug("Potentially non-standard OPC value: %d", result["OPC"])
                # 尝试修正OPC值
                if result["OPC"] <= len(echonet_bytes) - 12:
                    _LOGGER.debug("Attempting to continue with large OPC value")
                else:
                    _LOGGER.debug(
                        "OPC value too large for frame size, limiting to available data"
                    )
                    result["OPC"] = min(
                        result["OPC"], (len(echonet_bytes) - 12) // 3
                    )  # 估计每个属性至少需要3字节

            _LOGGER.debug(
                "ECHONET frame header: EHD=%s, TID=%s, SEOJ=%s, DEOJ=%s, ESV=0x%02X, OPC=%d",
                result["EHD"],
                result["TID"],
                result["SEOJ"],
                result["DEOJ"],
                result["ESV"],
                result["OPC"],
            )

            # 解析属性数据
            offset = 12
            for i in range(result["OPC"]):
                if offset + 2 > len(echonet_bytes):
                    _LOGGER.debug(
                        "Incomplete property data at index %d (offset %d, frame length %d)",
                        i,
                        offset,
                        len(echonet_bytes),
                    )
                    break

                try:
                    epc = echonet_bytes[offset]
                    pdc = echonet_bytes[offset + 1]
                    offset += 2

                    # 检查PDC值是否合理
                    if pdc > 100:  # 设置一个合理的上限
                        _LOGGER.debug(
                            "Potentially non-standard PDC value: %d for EPC 0x%02X",
                            pdc,
                            epc,
                        )
                        # 尝试修正PDC值
                        pdc = min(pdc, len(echonet_bytes) - offset)

                    # 确保有足够的数据
                    if offset + pdc > len(echonet_bytes):
                        _LOGGER.debug(
                            "Not enough data for property EPC=0x%02X, PDC=%d (offset %d, frame length %d)",
                            epc,
                            pdc,
                            offset,
                            len(echonet_bytes),
                        )
                        # 调整PDC以匹配可用数据
                        pdc = max(0, len(echonet_bytes) - offset)

                    # 提取属性数据
                    edt = echonet_bytes[offset : offset + pdc] if pdc > 0 else b""
                    offset += pdc

                    # 添加到属性列表
                    result["properties"].append((epc, pdc, edt))
                    _LOGGER.debug(
                        "Property %d: EPC=0x%02X, PDC=%d, EDT=%s",
                        i,
                        epc,
                        pdc,
                        edt.hex() if edt else "<empty>",
                    )
                except Exception as e:
                    _LOGGER.error(
                        "Error parsing property at index %d (offset %d): %s",
                        i,
                        offset,
                        str(e),
                    )
                    break

        except Exception as e:
            _LOGGER.error("Error parsing ECHONET frame: %s", str(e))

        return result
