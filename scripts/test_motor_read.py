#!/usr/bin/env python3
"""
Test reading Motor Input objects from ClearLink.
"""

import struct
from pycomm3 import CIPDriver, Services

CLEARLINK_IP = "192.168.20.240"

# Motor Input Object class IDs
MOTOR_INPUT_CLASSES = {
    1: 0x6A,
    2: 0x6B,
    3: 0x6C,
    4: 0x6D,
}

# Known attributes
ATTRS = {
    1: ('Enabled', 'BOOL'),
    2: ('Moving', 'BOOL'),
    3: ('In Fault', 'BOOL'),
    4: ('Homed', 'BOOL'),
    5: ('Position', 'DINT'),
    6: ('Velocity', 'DINT'),
}


def read_attr(driver, class_id, instance, attr, dtype='raw'):
    """Read attribute with type conversion."""
    try:
        result = driver.generic_message(
            service=Services.get_attribute_single,
            class_code=class_id,
            instance=instance,
            attribute=attr
        )
        if result is None:
            return None, f"No result"
        if not result.value:
            return None, f"Empty value, error={result.error}"

        data = result.value
        if dtype == 'BOOL':
            return bool(data[0]), None
        elif dtype == 'DINT' and len(data) >= 4:
            return struct.unpack('<i', data[:4])[0], None
        elif dtype == 'UDINT' and len(data) >= 4:
            return struct.unpack('<I', data[:4])[0], None
        else:
            return data.hex(), None
    except Exception as e:
        return None, str(e)


def main():
    print(f"Connecting to ClearLink at {CLEARLINK_IP}...")

    with CIPDriver(CLEARLINK_IP) as driver:
        driver.open()
        print("Connected!\n")

        # Try different instances (0 and 1)
        for instance in [0, 1]:
            print(f"=" * 60)
            print(f"INSTANCE {instance}")
            print(f"=" * 60)

            for axis, class_id in MOTOR_INPUT_CLASSES.items():
                print(f"\nMotor {axis} Input (Class 0x{class_id:02X}):")

                for attr_id, (attr_name, attr_type) in ATTRS.items():
                    val, err = read_attr(driver, class_id, instance, attr_id, attr_type)
                    if val is not None:
                        print(f"  {attr_name}: {val}")
                    else:
                        print(f"  {attr_name}: ERROR - {err}")

        # Also try Get Attribute All
        print("\n" + "=" * 60)
        print("TRYING GET_ATTRIBUTE_ALL")
        print("=" * 60)

        for axis, class_id in MOTOR_INPUT_CLASSES.items():
            print(f"\nMotor {axis} (Class 0x{class_id:02X}):")
            try:
                result = driver.generic_message(
                    service=Services.get_attributes_all,
                    class_code=class_id,
                    instance=1
                )
                if result and result.value:
                    print(f"  All attrs: {result.value.hex()}")
                else:
                    print(f"  No data, error={result.error if result else 'None'}")
            except Exception as e:
                print(f"  Error: {e}")

        # Check if motors need to be enabled first
        print("\n" + "=" * 60)
        print("CHECKING MOTOR OUTPUT OBJECTS")
        print("=" * 60)

        MOTOR_OUTPUT_CLASSES = {1: 0x66, 2: 0x67, 3: 0x68, 4: 0x69}

        for axis, class_id in MOTOR_OUTPUT_CLASSES.items():
            print(f"\nMotor {axis} Output (Class 0x{class_id:02X}):")
            for attr in range(1, 15):
                val, err = read_attr(driver, class_id, 1, attr)
                if val is not None:
                    print(f"  Attr {attr}: {val}")


if __name__ == '__main__':
    main()
