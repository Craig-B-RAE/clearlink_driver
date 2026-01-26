#!/usr/bin/env python3
"""
ClearLink EtherNet/IP Deep Discovery Script
Focused on finding voltage, current, and power data.
"""

import struct
from pycomm3 import CIPDriver, Services

CLEARLINK_IP = "192.168.20.240"

def read_attribute(driver, class_id, instance, attribute):
    """Read a single attribute, return raw bytes."""
    try:
        result = driver.generic_message(
            service=Services.get_attribute_single,
            class_code=class_id,
            instance=instance,
            attribute=attribute
        )
        if result and result.value:
            return result.value
        return None
    except Exception as e:
        return None


def decode_data(data, label=""):
    """Try to decode data as various types."""
    if data is None:
        return "None"

    results = [f"raw={data.hex()}"]

    if len(data) == 1:
        results.append(f"BOOL={bool(data[0])}")
        results.append(f"USINT={data[0]}")
    if len(data) >= 2:
        results.append(f"UINT={struct.unpack('<H', data[:2])[0]}")
        results.append(f"INT={struct.unpack('<h', data[:2])[0]}")
    if len(data) >= 4:
        results.append(f"UDINT={struct.unpack('<I', data[:4])[0]}")
        results.append(f"DINT={struct.unpack('<i', data[:4])[0]}")
        results.append(f"REAL={struct.unpack('<f', data[:4])[0]:.6f}")

    return " | ".join(results)


def main():
    print(f"Connecting to ClearLink at {CLEARLINK_IP}...")

    with CIPDriver(CLEARLINK_IP) as driver:
        driver.open()
        print("Connected!\n")

        # Motor Input Objects - Deep scan
        print("=" * 70)
        print("MOTOR INPUT OBJECTS - FULL ATTRIBUTE SCAN")
        print("=" * 70)

        MOTOR_INPUT_BASE = 0x6A
        for axis in range(1, 5):
            class_id = MOTOR_INPUT_BASE + (axis - 1)
            print(f"\nMotor {axis} Input (Class 0x{class_id:02X}):")

            for attr in range(1, 50):
                data = read_attribute(driver, class_id, 1, attr)
                if data:
                    print(f"  Attr {attr:2d}: {decode_data(data)}")

        # ClearLink System Object (0x64) - might have voltage
        print("\n" + "=" * 70)
        print("CLEARLINK SYSTEM OBJECT (Class 0x64) - DEEP SCAN")
        print("=" * 70)

        for instance in range(0, 5):
            found = False
            for attr in range(1, 50):
                data = read_attribute(driver, 0x64, instance, attr)
                if data:
                    if not found:
                        print(f"\n  Instance {instance}:")
                        found = True
                    print(f"    Attr {attr:2d}: {decode_data(data)}")

        # ClearLink I/O Object (0x65)
        print("\n" + "=" * 70)
        print("CLEARLINK I/O OBJECT (Class 0x65)")
        print("=" * 70)

        for instance in range(0, 5):
            found = False
            for attr in range(1, 30):
                data = read_attribute(driver, 0x65, instance, attr)
                if data:
                    if not found:
                        print(f"\n  Instance {instance}:")
                        found = True
                    print(f"    Attr {attr:2d}: {decode_data(data)}")

        # Unknown Class 0x70 - might be power/status
        print("\n" + "=" * 70)
        print("UNKNOWN CLASS 0x70 - POSSIBLE POWER/STATUS")
        print("=" * 70)

        for instance in range(0, 5):
            found = False
            for attr in range(1, 30):
                data = read_attribute(driver, 0x70, instance, attr)
                if data:
                    if not found:
                        print(f"\n  Instance {instance}:")
                        found = True
                    print(f"    Attr {attr:2d}: {decode_data(data)}")

        # Check higher class IDs that might have power data
        print("\n" + "=" * 70)
        print("SCANNING HIGHER CLASS IDS (0x71-0x80)")
        print("=" * 70)

        for class_id in range(0x71, 0x81):
            for instance in range(0, 3):
                found = False
                for attr in range(1, 20):
                    data = read_attribute(driver, class_id, instance, attr)
                    if data:
                        if not found:
                            print(f"\n  Class 0x{class_id:02X}, Instance {instance}:")
                            found = True
                        print(f"    Attr {attr:2d}: {decode_data(data)}")

        # Analog Input Class - check for voltage/current
        print("\n" + "=" * 70)
        print("ANALOG INPUT OBJECT (Class 0x0A) - ALL ATTRIBUTES")
        print("=" * 70)

        for instance in range(1, 10):
            found = False
            for attr in range(1, 20):
                data = read_attribute(driver, 0x0A, instance, attr)
                if data:
                    if not found:
                        print(f"\n  Instance {instance}:")
                        found = True
                    print(f"    Attr {attr:2d}: {decode_data(data)}")

        # Check Vendor-specific classes (often 0x64+ or 0xA0+)
        print("\n" + "=" * 70)
        print("VENDOR SPECIFIC CLASSES (0xA0-0xB0)")
        print("=" * 70)

        for class_id in range(0xA0, 0xB1):
            for instance in range(0, 3):
                found = False
                for attr in range(1, 15):
                    data = read_attribute(driver, class_id, instance, attr)
                    if data:
                        if not found:
                            print(f"\n  Class 0x{class_id:02X}, Instance {instance}:")
                            found = True
                        print(f"    Attr {attr:2d}: {decode_data(data)}")

        print("\n" + "=" * 70)
        print("DISCOVERY COMPLETE")
        print("=" * 70)


if __name__ == '__main__':
    main()
