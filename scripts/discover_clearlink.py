#!/usr/bin/env python3
"""
ClearLink EtherNet/IP Discovery Script
Queries the ClearLink to discover available objects and attributes.
"""

import struct
from pycomm3 import CIPDriver, Services

CLEARLINK_IP = "192.168.20.240"

# Known ClearLink class IDs
MOTOR_OUTPUT_BASE = 0x66  # Motor 1-4 output: 0x66-0x69
MOTOR_INPUT_BASE = 0x6A   # Motor 1-4 input: 0x6A-0x6D

# Standard CIP class IDs
IDENTITY_CLASS = 0x01
ASSEMBLY_CLASS = 0x04
CONNECTION_MANAGER = 0x06
ANALOG_INPUT_CLASS = 0x0A
ANALOG_OUTPUT_CLASS = 0x0B
DISCRETE_INPUT_CLASS = 0x08
DISCRETE_OUTPUT_CLASS = 0x09

# Possible ClearLink-specific classes
CLEARLINK_SYSTEM_CLASS = 0x64  # Guess - system info
CLEARLINK_IO_CLASS = 0x65      # Guess - digital I/O


def read_attribute(driver, class_id, instance, attribute, data_type='raw'):
    """Read a single attribute."""
    try:
        result = driver.generic_message(
            service=Services.get_attribute_single,
            class_code=class_id,
            instance=instance,
            attribute=attribute
        )
        if result and result.value:
            data = result.value
            if data_type == 'BOOL' and len(data) >= 1:
                return struct.unpack('?', data[:1])[0]
            elif data_type == 'USINT' and len(data) >= 1:
                return struct.unpack('B', data[:1])[0]
            elif data_type == 'UINT' and len(data) >= 2:
                return struct.unpack('<H', data[:2])[0]
            elif data_type == 'INT' and len(data) >= 2:
                return struct.unpack('<h', data[:2])[0]
            elif data_type == 'UDINT' and len(data) >= 4:
                return struct.unpack('<I', data[:4])[0]
            elif data_type == 'DINT' and len(data) >= 4:
                return struct.unpack('<i', data[:4])[0]
            elif data_type == 'REAL' and len(data) >= 4:
                return struct.unpack('<f', data[:4])[0]
            elif data_type == 'STRING':
                return data.decode('utf-8', errors='ignore').rstrip('\x00')
            else:
                return data.hex() if data else None
        return None
    except Exception as e:
        return None


def get_all_attributes(driver, class_id, instance, max_attr=20):
    """Try to read all attributes for a class/instance."""
    attrs = {}
    for attr in range(1, max_attr + 1):
        result = read_attribute(driver, class_id, instance, attr)
        if result is not None:
            attrs[attr] = result
    return attrs


def main():
    print(f"Connecting to ClearLink at {CLEARLINK_IP}...")

    with CIPDriver(CLEARLINK_IP) as driver:
        driver.open()
        print("Connected!\n")

        # 1. Identity Object (Class 0x01)
        print("=" * 60)
        print("IDENTITY OBJECT (Class 0x01, Instance 1)")
        print("=" * 60)
        identity_attrs = {
            1: ('Vendor ID', 'UINT'),
            2: ('Device Type', 'UINT'),
            3: ('Product Code', 'UINT'),
            4: ('Revision', 'raw'),
            5: ('Status', 'UINT'),
            6: ('Serial Number', 'UDINT'),
            7: ('Product Name', 'STRING'),
        }
        for attr, (name, dtype) in identity_attrs.items():
            val = read_attribute(driver, IDENTITY_CLASS, 1, attr, dtype)
            if val is not None:
                print(f"  Attr {attr} ({name}): {val}")

        # 2. Scan for additional classes
        print("\n" + "=" * 60)
        print("SCANNING FOR AVAILABLE CLASSES")
        print("=" * 60)

        classes_to_check = [
            (0x04, "Assembly"),
            (0x64, "ClearLink System?"),
            (0x65, "ClearLink I/O?"),
            (0x66, "Motor 1 Output"),
            (0x67, "Motor 2 Output"),
            (0x68, "Motor 3 Output"),
            (0x69, "Motor 4 Output"),
            (0x6A, "Motor 1 Input"),
            (0x6B, "Motor 2 Input"),
            (0x6C, "Motor 3 Input"),
            (0x6D, "Motor 4 Input"),
            (0x6E, "Unknown 0x6E"),
            (0x6F, "Unknown 0x6F"),
            (0x70, "Unknown 0x70"),
            (0x71, "Unknown 0x71"),
            (0x72, "Unknown 0x72"),
            (0x73, "Unknown 0x73"),
            (0x74, "Unknown 0x74"),
            (0x75, "Unknown 0x75"),
            (0x0A, "Analog Input"),
            (0x0B, "Analog Output"),
            (0x08, "Discrete Input"),
            (0x09, "Discrete Output"),
        ]

        for class_id, name in classes_to_check:
            attrs = get_all_attributes(driver, class_id, 1, 20)
            if attrs:
                print(f"\n  Class 0x{class_id:02X} ({name}) - {len(attrs)} attributes found:")
                for attr, val in attrs.items():
                    print(f"    Attr {attr}: {val}")

        # 3. Deep dive on Motor Input objects
        print("\n" + "=" * 60)
        print("MOTOR INPUT OBJECTS - EXTENDED SCAN")
        print("=" * 60)

        for axis in range(1, 5):
            class_id = MOTOR_INPUT_BASE + (axis - 1)
            print(f"\n  Motor {axis} Input (Class 0x{class_id:02X}):")

            # Try more attributes than documented
            for attr in range(1, 30):
                val = read_attribute(driver, class_id, 1, attr)
                if val is not None:
                    # Try to decode as different types
                    raw = bytes.fromhex(val) if isinstance(val, str) else val
                    interpretations = []

                    if isinstance(val, str) and len(val) >= 2:
                        try:
                            raw_bytes = bytes.fromhex(val)
                            if len(raw_bytes) == 1:
                                interpretations.append(f"BOOL={bool(raw_bytes[0])}")
                                interpretations.append(f"USINT={raw_bytes[0]}")
                            if len(raw_bytes) >= 2:
                                interpretations.append(f"UINT={struct.unpack('<H', raw_bytes[:2])[0]}")
                                interpretations.append(f"INT={struct.unpack('<h', raw_bytes[:2])[0]}")
                            if len(raw_bytes) >= 4:
                                interpretations.append(f"UDINT={struct.unpack('<I', raw_bytes[:4])[0]}")
                                interpretations.append(f"DINT={struct.unpack('<i', raw_bytes[:4])[0]}")
                                interpretations.append(f"REAL={struct.unpack('<f', raw_bytes[:4])[0]:.4f}")
                        except:
                            pass

                    interp_str = " | ".join(interpretations) if interpretations else ""
                    print(f"    Attr {attr}: {val} {interp_str}")

        # 4. Check for analog inputs (current sensors, voltage, etc.)
        print("\n" + "=" * 60)
        print("ANALOG INPUT SCAN (Class 0x0A)")
        print("=" * 60)

        for instance in range(1, 10):
            attrs = get_all_attributes(driver, ANALOG_INPUT_CLASS, instance, 10)
            if attrs:
                print(f"\n  Instance {instance}:")
                for attr, val in attrs.items():
                    print(f"    Attr {attr}: {val}")

        # 5. Check Assembly objects for I/O data
        print("\n" + "=" * 60)
        print("ASSEMBLY OBJECTS (Class 0x04)")
        print("=" * 60)

        for instance in [100, 101, 102, 103, 104, 105, 150, 151, 152]:
            attrs = get_all_attributes(driver, ASSEMBLY_CLASS, instance, 5)
            if attrs:
                print(f"\n  Assembly Instance {instance}:")
                for attr, val in attrs.items():
                    print(f"    Attr {attr}: {val}")

        print("\n" + "=" * 60)
        print("DISCOVERY COMPLETE")
        print("=" * 60)


if __name__ == '__main__':
    main()
