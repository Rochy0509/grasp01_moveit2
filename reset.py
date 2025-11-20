#!/bin/python3

import myactuator_rmd_py as rmd
from time import sleep

# Init the CAN driver
try:
    driver = rmd.CanDriver("can0")
except Exception as e:
    print(f"Failed to initialize CAN driver: {e}")
    exit(1)

# Create motor instances with dictionary for easier ID tracking
motors = {
    1: rmd.ActuatorInterface(driver, 1),
    2: rmd.ActuatorInterface(driver, 2),
    3: rmd.ActuatorInterface(driver, 3),
    4: rmd.ActuatorInterface(driver, 4),
    5: rmd.ActuatorInterface(driver, 5),
    6: rmd.ActuatorInterface(driver, 6),
}

def set_and_verify_zero(motors, tolerance=0.1, max_attempts=10):
    """
    Sets motor encoder positions to zero and verifies they stay within tolerance.
    Will retry if any motor drifts beyond the specified tolerance.
    
    Args:
        motors (dict): Dictionary mapping motor_id to ActuatorInterface
        tolerance (float): Max allowed deviation from zero (default: 0.05)
        max_attempts (int): How many times to try before giving up (default: 10)
    
    Returns:
        bool: True if successful, False if we couldn't get motors to stay at zero
    """
    for attempt in range(1, max_attempts + 1):
        print(f"\n=== Attempt {attempt} to set zero positions ===")
        all_within_tolerance = True

        # First, set current positions as zero
        print("Setting initial encoder positions as zero:")
        for motor_id, motor in motors.items():
            try:
                initial_pos = motor.getMultiTurnAngle()
                print(f"Motor {motor_id} Initial Position: {initial_pos}")
                sleep(0.1)  # Small delay for stability
                motor.setCurrentPositionAsEncoderZero()
                print(f"Motor {motor_id} current position set as zero.")
                zero_offset = motor.getMultiTurnEncoderZeroOffset()
                print(f"Motor {motor_id} Zero Offset (Post Set): {zero_offset}")
            except rmd.ProtocolException as e:
                print(f"Error setting zero for Motor {motor_id}: {e}")
                all_within_tolerance = False
            except Exception as e:
                print(f"Unexpected error with Motor {motor_id}: {e}")
                all_within_tolerance = False

        # Now reset the motors
        print("\nResetting motors:")
        for motor_id, motor in motors.items():
            try:
                motor.reset()
                print(f"Motor {motor_id} has been reset.")
            except rmd.ProtocolException as e:
                print(f"Error resetting Motor {motor_id}: {e}")
            except Exception as e:
                print(f"Unexpected error with Motor {motor_id}: {e}")
        sleep(5)  # Wait for motors to reboot - might need to adjust this time

        # Check if the motors stayed at zero
        print("\nVerifying zero positions:")
        for motor_id, motor in motors.items():
            try:
                current_pos = motor.getMultiTurnAngle()
                print(f"Motor {motor_id} Current Position: {current_pos}")
                if abs(current_pos) > tolerance:
                    print(f"Motor {motor_id} position {current_pos} exceeds tolerance ±{tolerance}")
                    all_within_tolerance = False
            except rmd.ProtocolException as e:
                print(f"Error reading position for Motor {motor_id}: {e}")
                all_within_tolerance = False
            except Exception as e:
                print(f"Unexpected error with Motor {motor_id}: {e}")
                all_within_tolerance = False

        # If everything looks good, we're done
        if all_within_tolerance:
            print("All motors are within tolerance.")
            return True
        else:
            print("Some motors not within tolerance. Retrying...")

    print(f"\nFailed to set motors within tolerance ±{tolerance} after {max_attempts} attempts.")
    return False

# Run the zero-setting process
if set_and_verify_zero(motors):
    print("\nSuccessfully set all motors to zero pose.")
else:
    print("\nFailed to set motors to zero pose within tolerance.")