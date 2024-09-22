
from pymavlink import mavutil

from logging import basicConfig as logging_basicConfig
from logging import getLevelName as logging_getLevelName

from logging import debug as logging_debug
from logging import info as logging_info
from logging import error as logging_error



def auto_detect_serial():
    preferred_ports = [
        '*FTDI*',
        "*3D*",
        "*USB_to_UART*",
        '*Ardu*',
        '*PX4*',
        '*Hex_*',
        '*Holybro_*',
        '*mRo*',
        '*FMU*',
        '*Swift-Flyer*',
        '*Serial*',
        '*CubePilot*',
        '*Qiotek*',
    ]
    serial_list = mavutil.auto_detect_serial(preferred_list=preferred_ports)
    serial_list.sort(key=lambda x: x.device)

    # remove OTG2 ports for dual CDC
    if len(serial_list) == 2 and serial_list[0].device.startswith("/dev/serial/by-id"):
        if serial_list[0].device[:-1] == serial_list[1].device[0:-1]:
            serial_list.pop(1)

    return serial_list


def auto_connect(device):
    comport = None
    if device:
        comport = mavutil.SerialPort(device=device, description=device)
    else:
        autodetect_serial = auto_detect_serial()
        if autodetect_serial:
            # Resolve the soft link if it's a Linux system
            if os.name == 'posix':
                try:
                    dev = autodetect_serial[0].device
                    logging_debug("Auto-detected device %s", dev)
                    # Get the directory part of the soft link
                    softlink_dir = os.path.dirname(dev)
                    # Resolve the soft link and join it with the directory part
                    resolved_path = os.path.abspath(os.path.join(softlink_dir, os.readlink(dev)))
                    autodetect_serial[0].device = resolved_path
                    logging_debug("Resolved soft link %s to %s", dev, resolved_path)
                except OSError:
                    pass # Not a soft link, proceed with the original device path
            comport = autodetect_serial[0]
        else:
            logging_error("No serial ports found. Please connect a flight controller and try again.")
            sys.exit(1)
    return comport


def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    logging_info("Waiting for flight controller heartbeat")
    m.wait_heartbeat()
    logging_info("Got heartbeat from system %u, component %u", m.target_system, m.target_system)
# pylint: enable=duplicate-code