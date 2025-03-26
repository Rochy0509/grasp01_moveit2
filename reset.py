#!/bin/python3

import myactuator_rmd_py as rmd
from time import sleep

# Initialize the CAN driver
try:
    driver = rmd.CanDriver("can0")
except Exception as e:
    print(f"Failed to initialize CAN driver: {e}")
    exit(1)

# Create motor instances with explicit ID tracking using a dictionary
motors = {
    1: rmd.ActuatorInterface(driver, 1),
    2: rmd.ActuatorInterface(driver, 2),
    3: rmd.ActuatorInterface(driver, 3),
    4: rmd.ActuatorInterface(driver, 4),
    5: rmd.ActuatorInterface(driver, 5),
    6: rmd.ActuatorInterface(driver, 6),
}

def set_and_verify_zero(motors, tolerance=0.05, max_attempts=10):
    """
    Set motor encoder positions to zero and verify they stay within tolerance.
    Repeats the process if any motor deviates beyond ±tolerance.
    
    Args:
        motors (dict): Dictionary of motor_id: ActuatorInterface pairs
        tolerance (float): Maximum allowed deviation from zero (default: 0.05)
        max_attempts (int): Maximum number of retries (default: 5)
    
    Returns:
        bool: True if all motors are within tolerance, False otherwise
    """
    for attempt in range(1, max_attempts + 1):
        print(f"\n=== Attempt {attempt} to set zero positions ===")
        all_within_tolerance = True

        # Step 1: Set initial encoder positions as zero
        print("Setting initial encoder positions as zero:")
        for motor_id, motor in motors.items():
            try:
                initial_pos = motor.getMultiTurnAngle()
                print(f"Motor {motor_id} Initial Position: {initial_pos}")
                sleep(0.1)  # Short delay for stability
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

        # Step 2: Reset motors
        print("\nResetting motors:")
        for motor_id, motor in motors.items():
            try:
                motor.reset()
                print(f"Motor {motor_id} has been reset.")
            except rmd.ProtocolException as e:
                print(f"Error resetting Motor {motor_id}: {e}")
            except Exception as e:
                print(f"Unexpected error with Motor {motor_id}: {e}")
        sleep(2)  # Wait for motors to reboot/reset, adjust time if needed

        # Step 3: Verify zero positions
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

        # Check if all motors are within tolerance
        if all_within_tolerance:
            print("All motors are within tolerance.")
            return True
        else:
            print("Some motors are not within tolerance. Retrying...")

    print(f"\nFailed to set motors within tolerance ±{tolerance} after {max_attempts} attempts.")
    return False

# Execute the zero-setting and verification process
if set_and_verify_zero(motors):
    print("\nSuccessfully set all motors to zero pose.")
else:
    print("\nFailed to set motors to zero pose within tolerance.")