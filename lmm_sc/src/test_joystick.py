#!/usr/bin/env python

"""
Standalone joystick test script for Raspberry Pi.

Reads raw joystick events from /dev/input/js0 and prints axis/button
values in real time. No ROS installation required — only Python and a
connected joystick.

Usage (from the Raspberry Pi terminal):
    python test_joystick.py

Press Ctrl+C to stop.
"""

import struct
import sys
import os
import glob
import array
import fcntl

# --- Linux joystick ioctl constants ---
# The size field in an _IOR ioctl code is encoded at bit-shift 16.
IOCTL_SIZE_SHIFT = 0x10000
MAX_NAME_LENGTH = 128

JSIOCGNAME = 0x80006A13      # Get device name (up to MAX_NAME_LENGTH chars)
JSIOCGAXES = 0x80016A11      # Get number of axes
JSIOCGBUTTONS = 0x80016A12   # Get number of buttons

# Joystick event struct: (timestamp_ms, value, type, number)
JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)

# Event types
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def find_joystick_devices():
    """Return a sorted list of /dev/input/js* paths."""
    return sorted(glob.glob('/dev/input/js*'))


def get_device_name(fd):
    """Read the human-readable name of a joystick device."""
    buf = array.array('B', [0] * MAX_NAME_LENGTH)
    try:
        fcntl.ioctl(fd, JSIOCGNAME + (IOCTL_SIZE_SHIFT * len(buf)), buf)
        return buf.tobytes().split(b'\x00', 1)[0].decode('utf-8', errors='replace')
    except (IOError, OSError):
        return '(unknown)'


def get_axis_count(fd):
    """Return the number of axes reported by the device."""
    buf = array.array('B', [0])
    try:
        fcntl.ioctl(fd, JSIOCGAXES, buf)
        return buf[0]
    except (IOError, OSError):
        return 0


def get_button_count(fd):
    """Return the number of buttons reported by the device."""
    buf = array.array('B', [0])
    try:
        fcntl.ioctl(fd, JSIOCGBUTTONS, buf)
        return buf[0]
    except (IOError, OSError):
        return 0


def print_device_info(path, fd):
    """Print a summary of the joystick device."""
    name = get_device_name(fd)
    axes = get_axis_count(fd)
    buttons = get_button_count(fd)
    print('')
    print('=== Joystick Detected ===')
    print('  Device : {}'.format(path))
    print('  Name   : {}'.format(name))
    print('  Axes   : {}'.format(axes))
    print('  Buttons: {}'.format(buttons))
    print('=========================')
    print('')
    return axes, buttons


def run(device_path=None):
    # --- Discover device ---
    if device_path is None:
        devices = find_joystick_devices()
        if not devices:
            print('ERROR: No joystick found at /dev/input/js*')
            print('')
            print('Troubleshooting:')
            print('  1. Plug in the joystick (USB) or pair via Bluetooth.')
            print('  2. Run: ls /dev/input/js*')
            print('  3. If nothing shows, try: sudo apt-get install joystick')
            print('  4. Load the driver:  sudo modprobe joydev')
            print('  5. Check dmesg:      dmesg | tail')
            sys.exit(1)
        device_path = devices[0]
        if len(devices) > 1:
            print('Multiple joystick devices found: {}'.format(devices))
            print('Using first device: {}'.format(device_path))

    # --- Open device ---
    try:
        fd = os.open(device_path, os.O_RDONLY)
    except OSError as exc:
        print('ERROR: Cannot open {}: {}'.format(device_path, exc))
        print('')
        print('If you see "Permission denied", run:')
        print('  sudo chmod a+r {}'.format(device_path))
        print('or run this script with sudo.')
        sys.exit(1)

    axes_count, buttons_count = print_device_info(device_path, fd)

    # State arrays
    axes = [0] * axes_count
    buttons = [0] * buttons_count

    print('Reading joystick events — move sticks / press buttons ...')
    print('Press Ctrl+C to stop.')
    print('')

    try:
        while True:
            event = os.read(fd, JS_EVENT_SIZE)
            if len(event) < JS_EVENT_SIZE:
                continue

            timestamp, value, ev_type, number = struct.unpack(JS_EVENT_FMT, event)

            # Strip the INIT flag for state tracking
            base_type = ev_type & ~JS_EVENT_INIT

            if base_type == JS_EVENT_AXIS and number < axes_count:
                axes[number] = value
            elif base_type == JS_EVENT_BUTTON and number < buttons_count:
                buttons[number] = value

            # Skip printing pure init events to reduce noise
            if ev_type & JS_EVENT_INIT:
                continue

            if base_type == JS_EVENT_AXIS:
                print('AXIS  {:2d}  value: {:+6d}   (all axes: {})'.format(
                    number, value, axes))
            elif base_type == JS_EVENT_BUTTON:
                state = 'PRESSED' if value else 'released'
                print('BTN   {:2d}  {:8s}        (all btns: {})'.format(
                    number, state, buttons))
    except KeyboardInterrupt:
        print('')
        print('Stopped.')
    finally:
        os.close(fd)

    # Print the axis mapping used by joy_incremental.py for reference.
    # NOTE: These indices must stay in sync with joy_incremental.py (lines 29-31).
    print('')
    print('--- SiMMER Axis Mapping Reference ---')
    print('  joy_incremental.py reads:')
    print('    axes[3] -> right stick X  (manipulator X)')
    print('    axes[4] -> right stick Y  (manipulator Y)')
    print('    axes[1] -> left stick Y   (manipulator Z)')
    print('')
    print('If your joystick axes differ, update the indices in')
    print('  lmm_sc/src/joy_incremental.py  (lines 29-31)')


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else None
    run(path)
