#!/usr/bin/env python3
"""
Example usage of the Motor Control API

This demonstrates how to use the motor_api module to control motors.
"""

from motor_api import connect, disconnect, enable_motor, move_motor, stop_motor
import time

def main():
    print("Motor Control API Example")
    print("=" * 60)
    
    # Step 1: Connect to gateway
    print("\n1. Connecting to gateway...")
    success, error = connect()
    if not success:
        print(f"   ✗ Failed: {error}")
        return
    print("   ✓ Connected")
    
    node_id = 5  # Motor CAN node ID
    
    try:
        # Step 2: Enable motor
        print(f"\n2. Enabling motor (node ID {node_id})...")
        success, error = enable_motor(node_id, True)
        if not success:
            print(f"   ✗ Failed: {error}")
            return
        print("   ✓ Motor enabled")
        
        # Example 1: Continuous forward motion
        print("\n3. Starting continuous forward motion (jog speed 1000)...")
        success, error = move_motor(node_id, jog_speed=1000)
        if success:
            print("   ✓ Motion started")
            time.sleep(2)  # Run for 2 seconds
            stop_motor(node_id)
            print("   ✓ Stopped")
        else:
            print(f"   ✗ Failed: {error}")
        
        # Example 2: Move to absolute position
        print("\n4. Moving to absolute position 5000 (speed 2000)...")
        success, error = move_motor(node_id, position=5000, relative=False, speed=2000)
        if success:
            print("   ✓ Motion started")
            time.sleep(3)  # Wait for motion to complete
            print("   ✓ Motion complete")
        else:
            print(f"   ✗ Failed: {error}")
        
        # Example 3: Relative motion
        print("\n5. Moving relative +1000 steps (speed 1500)...")
        success, error = move_motor(node_id, position=1000, relative=True, speed=1500)
        if success:
            print("   ✓ Motion started")
            time.sleep(2)
            print("   ✓ Motion complete")
        else:
            print(f"   ✗ Failed: {error}")
        
        # Step 6: Disable motor
        print(f"\n6. Disabling motor...")
        enable_motor(node_id, False)
        print("   ✓ Motor disabled")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        stop_motor(node_id)
    finally:
        # Always disconnect
        disconnect()
        print("\n✓ Disconnected")


if __name__ == '__main__':
    main()

