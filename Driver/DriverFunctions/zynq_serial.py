"""Zybo Z7 FT2232 serial vs JTAG.

Digilent enumerates two USB serial endpoints. Interface 0 is JTAG (often
/dev/ttyUSB0); interface 1 is the PS UART used by the cartpole firmware.
ttyUSB numbering is not stable, so SERIAL_PORT_NUMBER indexing the combined
list can open JTAG and the driver hangs reconnecting.
"""
from serial.tools import list_ports

_ZYNQ_DESCRIPTIONS = (
    "Digilent Adept USB Device - Digilent Adept USB Device",
    "Digilent Adept USB Device",
)


def _ftdi_interface_index(port):
    loc = getattr(port, "location", None) or ""
    if "." not in loc:
        return None
    try:
        return int(loc.rsplit(".", 1)[-1])
    except ValueError:
        return None


def prefer_zynq_uart_port(current_device=None):
    """Return the Digilent FTDI UART device, or current_device if none found."""
    uart_ports = [
        p for p in list_ports.comports()
        if p.description in _ZYNQ_DESCRIPTIONS and _ftdi_interface_index(p) == 1
    ]
    if not uart_ports:
        return current_device
    device = uart_ports[0].device
    if current_device and current_device != device:
        print(
            f"Zybo serial: {current_device} is not FTDI UART (interface 1); "
            f"using {device}"
        )
    return device
