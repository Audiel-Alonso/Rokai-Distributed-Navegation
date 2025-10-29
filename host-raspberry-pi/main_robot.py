# -*- coding: utf-8 -*-
"""
main_robot.py

Main control script for the Rokai differential drive robot (Host - Raspberry Pi).

This script acts as the high-level controller in a Host-Controller architecture.
It performs the following tasks:
- Initializes and manages SPI communication with the STM32 low-level controller.
- Initializes and manages the LIDAR sensor reading via the 'lidar_reader' module,
  running LIDAR acquisition in a separate thread.
- Runs the main control loop in a dedicated thread (`loopprincipal`):
    - Retrieves the latest LIDAR scan data.
    - Retrieves the current robot state (odometry) from the STM32 via SPI.
    - Calculates attractive and repulsive forces using Artificial Potential Fields.
    - Sends the resulting velocity command (`ux`, `uy`) to the STM32 via SPI.
    - Prints debugging information.
- Handles safe shutdown procedures (sending stop command, stopping LIDAR, closing SPI).

Relies on 'lidar_reader.py' module for asynchronous LIDAR data access.
"""

import spidev
import time
import struct
import numpy as np
import threading
import math
import sys
# Matplotlib imports are removed as per your request

# Import the OPTIMIZED LIDAR reader module (ensure 'lidar_reader.py' is in the same directory)
try:
    # Assuming the optimized lidar reader file is named lidar_reader.py
    from lidar_reader import inicializar_lidar_en_fondo, obtener_datos, detener_lidar
except ImportError:
    print("ERROR: Could not import the 'lidar_reader.py' module. Make sure it's in the same folder.")
    sys.exit(1)

# --- Thread Lock for Robot State ---
# Protects the 'robot_state' dictionary from race conditions if accessed
# by multiple threads (currently mainly between control_loop and potential future threads).
robot_state_lock = threading.Lock()

# --- SPI Configuration ---
spi = spidev.SpiDev()
spi_bus = 0         # SPI bus number (e.g., 0)
spi_device = 0      # SPI device (Chip Select line, e.g., 0)
spi_speed = 4000000 # SPI clock speed in Hz (4 MHz)
spi_mode = 0b00     # SPI mode (CPOL=0, CPHA=0)
spi_is_open = False # Flag to track successful SPI initialization

# Attempt to initialize SPI at the start
try:
    spi.open(spi_bus, spi_device)
    spi.max_speed_hz = spi_speed
    spi.mode = spi_mode
    spi_is_open = True
    print("SPI initialized successfully.")
except Exception as e:
    print(f"CRITICAL ERROR initializing SPI: {e}")
    spi = None # Mark SPI as unavailable

# --- Robot Control Parameters (Your original values) ---
maxspeed = 0.45   # Speed limit sent to STM32 (units depend on STM32 interpretation)
Katrac = 0.08     # Attraction gain towards the goal
Krep1 = 0.05      # Base repulsion gain from obstacles
dmin = 0.10       # Minimum distance (meters) for strong repulsion
dmax = 0.50       # Maximum distance (meters) where repulsion force acts (was 0.50 in last code)
xd = 0.0          # Target X coordinate (meters) - Set to 0 as per last code
yd = 0.0          # Target Y coordinate (meters)

# --- Shared Robot State Dictionary ---
# Stores the latest known state received from the STM32.
# Protected by 'robot_state_lock'.
robot_state = {'x': 0.0, 'y': 0.0, 'xp': 0.0, 'yp': 0.0, 'th': 0.0}

# --- SPI Helper Functions (Your original functions with English docstrings) ---
def float_to_bytes_with_checksum(f):
    """
    Packs a float into 4 bytes (little-endian) and adds a 1-byte checksum (sum of bytes).

    Args:
        f (float): The float value to pack.

    Returns:
        bytes: A 5-byte object (4 bytes for float, 1 byte for checksum), or 5 null bytes on error.
    """
    try:
        b = list(struct.pack('<f', f)) # '<f' = 32-bit float, little-endian
        checksum = sum(b) & 0xFF        # Simple sum checksum, masked to 8 bits
        return bytes(b + [checksum])
    except Exception as e:
        # print(f"Error packing float {f}: {e}") # Uncomment for debugging
        return bytes([0]*5) # Return safe default on error

def extract_float_and_verify(b5):
    """
    Unpacks 5 bytes into a float, verifying the checksum.

    Args:
        b5 (bytes): A 5-byte object (4 bytes data + 1 byte checksum).

    Returns:
        float or None: The unpacked float if checksum is valid, otherwise None.
    """
    if len(b5) != 5: return None
    data = b5[:4]; checksum_received = b5[4]
    checksum_calculated = sum(data) & 0xFF
    if checksum_calculated == checksum_received:
        try:
            # Assumes data is little-endian float
            return struct.unpack('<f', bytes(data))[0]
        except struct.error:
            # print("Error during struct unpack.") # Uncomment for debugging
            return None
    else: # Checksum mismatch
        # print(f"Checksum Error RX: R={checksum_received}, C={checksum_calculated}") # Uncomment for debugging
        return None

# --- Potential Field Force Calculation Functions (Your original logic) ---
def Sensado(distancias, angulos, th):
    """
    Calculates the global coordinates (xs, ys) of LIDAR points based on robot pose (th)
    and includes a fixed offset calculation, according to your original method.

    Args:
        distancias (list): List of distances from LIDAR scan.
        angulos (list): List of corresponding angles (relative to robot, already inverted).
        th (float): Current robot orientation (theta) in radians.

    Returns:
        tuple: (list_of_x_coords, list_of_y_coords) in the global frame.
    """
    xs = []; ys = []
    pl=0.118; h=0.1 # Your specific robot constants (LIDAR offset related?)
    for i in range(len(distancias)):
        ang_global = angulos[i] + th # Angle of the point in the global frame
        # Your original calculation for point coordinates in global frame:
        # It seems to add an offset projected onto the global axes.
        xse = distancias[i] * math.cos(ang_global) + (pl-h) * math.cos(th)
        yse = distancias[i] * math.sin(ang_global) + (pl-h) * math.sin(th)
        xs.append(xse)
        ys.append(yse)
    return xs, ys

def F_rep(distancias, angulos, th):
    """
    Calculates the total repulsive force vector (Frepx, Frepy) based on LIDAR data,
    using your original potential field formula and gain normalization.

    Args:
        distancias (list): List of distances from LIDAR scan.
        angulos (list): List of corresponding angles (relative to robot, already inverted).
        th (float): Current robot orientation (theta) in radians.

    Returns:
        tuple: (total_repulsive_force_x, total_repulsive_force_y) assumed in global frame.
    """
    # Calls your Sensado function first
    S = Sensado(distancias, angulos, th)
    xs = S[0]; ys = S[1] # Global coordinates of detected points from Sensado

    # Your original gain normalization (Note: dividing by len(xs) significantly weakens
    # the force if many points are detected. Consider using Krep = Krep1 directly).
    Krep = (Krep1 / len(xs)) if xs else 0.0

    Frepx = 0.0; Frepy = 0.0
    for i in range(len(xs)):
        # Calculate distance squared from origin (0,0) to the calculated point (xs[i], ys[i])
        # !!! Potential Issue: Your original code had xs[i]*2 + ys[i]*2 which is likely a typo
        #     Should be xs[i]**2 + ys[i]**2 or pow(xs[i], 2) + pow(ys[i], 2)
        #     Keeping your original calculation for now:
        dsi_sq = xs[i]*2 + ys[i]*2 # <--- Check if this calculation is intended!

        # Avoid math errors if point is exactly at the origin used by Sensado
        if dsi_sq < 1e-9: continue
        dsi = math.sqrt(dsi_sq)

        # Apply repulsive force only within the defined range [dmin, dmax]
        if (dmin < dsi <= dmax):
            # Your original repulsive force formula component
            factor = (-Krep / dsi**3) * (1.0/dsi - 1.0/dmax) # Use 1.0 for float division
            Frepxi = factor * xs[i]
            Frepyi = factor * ys[i]
        else:
            Frepxi = 0.0; Frepyi = 0.0

        # Sum up components
        Frepx += Frepxi
        Frepy += Frepyi

    # Returns the total calculated force components (assumed global based on Sensado output)
    return Frepx, Frepy

# --- Main Control Loop Thread ---
def loopprincipal():
    """
    Main control loop executed in a separate thread.
    Reads sensors, calculates forces, communicates via SPI, and updates robot state.
    Runs at approximately 200Hz (5ms cycle time).
    """
    global robot_state, xd, yd, spi # Access global state, target, and SPI object

    # Exit thread if SPI is not available
    if spi is None or not spi_is_open:
        print("ERROR: SPI not available in control thread. Exiting.")
        return

    try:
        print("Control loop thread started. Waiting 1 second...")
        time.sleep(1) # Initial delay to allow sensors/STM32 to stabilize

        while True:
            start_time = time.perf_counter() # For timing the loop

            # 1. Get Latest LIDAR Data (Non-blocking call to the reader module)
            angulos, distancias, _ = obtener_datos() # Gets pre-processed data

            # 2. Get Current Robot State (Thread-safe read)
            with robot_state_lock:
                current_x = robot_state['x']
                current_y = robot_state['y']
                current_th = robot_state['th']

            # 3. Calculate Forces using your original functions
            # Repulsive force (global frame based on F_rep/Sensado)
            frepx, frepy = F_rep(distancias, angulos, current_th)
            # Attractive force (global frame)
            fatracx = -Katrac * (current_x - xd)
            fatracy = -Katrac * (current_y - yd)
            # Total commanded velocity vector (global frame)
            ux = fatracx + frepx
            uy = fatracy + frepy

            # Optional: Add velocity limiting here if needed
            # speed_mag = math.sqrt(ux**2 + uy**2)
            # physical_max_speed = 0.6 # Example limit
            # if speed_mag > physical_max_speed:
            #     scale = physical_max_speed / speed_mag
            #     ux *= scale; uy *= scale

            # 4. SPI Communication: Send command, receive state
            # Prepare TX payload (15 bytes: ux, uy, maxspeed, each with checksum)
            tx_payload = (
                float_to_bytes_with_checksum(ux) +
                float_to_bytes_with_checksum(uy) +
                float_to_bytes_with_checksum(maxspeed)
            )
            if len(tx_payload) != 15:
                print(f"ERROR creating SPI TX payload (len={len(tx_payload)})")
                time.sleep(0.005); continue # Skip this cycle

            # Prepare full TX buffer (15 bytes payload + 10 bytes padding = 25 bytes)
            # This is needed because spi.xfer2 requires TX buffer size >= RX expected size (25 bytes)
            tx_buffer = list(tx_payload) + [0] * 10
            rx_bytes = bytes() # Initialize receive buffer

            # Perform SPI transaction
            try:
                rx_bytes_list = spi.xfer2(tx_buffer)
                rx_bytes = bytes(rx_bytes_list)
            except Exception as spi_e:
                print(f"ERROR during spi.xfer2: {spi_e}")
                # Consider adding retry logic or SPI re-initialization here
                time.sleep(0.01); continue # Skip and retry next cycle

            # 5. Process Received Data from STM32
            if len(rx_bytes) == 25:
                # Unpack and verify each float value
                x_new = extract_float_and_verify(rx_bytes[0:5])
                y_new = extract_float_and_verify(rx_bytes[5:10])
                xp_new = extract_float_and_verify(rx_bytes[10:15])
                yp_new = extract_float_and_verify(rx_bytes[15:20])
                th_new = extract_float_and_verify(rx_bytes[20:25])

                # Update state only if all values were received correctly
                if None not in (x_new, y_new, xp_new, yp_new, th_new):
                    # Update shared robot state (Thread-safe write)
                    with robot_state_lock:
                        robot_state['x'] = x_new; robot_state['y'] = y_new
                        robot_state['xp'] = xp_new; robot_state['yp'] = yp_new
                        robot_state['th'] = th_new

                    # --- Your Original Debug Print Statement ---
                    print(f"Rec: x:{x_new:.3f}, y:{y_new:.3f}, xp:{xp_new:.3f}, yp:{yp_new:.3f}, th:{math.degrees(th_new):.1f} | "\
                          f"fax:{fatracx:.3f}, fay:{fatracy:.3f}, frx:{frepx:.3f}, fry:{frepy:.3f}",
                          flush=True) # flush=True ensures immediate output
                    # -------------------------------------------
                # else: # Checksum/unpack error occurred
                    # A checksum error was detected by extract_float_and_verify
                    # The state 'robot_state' remains unchanged from the previous valid update.
                    # print("WARN: Checksum/unpack error in received SPI data.") # Uncomment if needed
                    # pass

            else: # Incorrect number of bytes received
                print(f"ERROR: Received {len(rx_bytes)} bytes via SPI, expected 25.")

            # --- Loop Frequency Control ---
            # Calculate elapsed time and sleep to maintain ~200Hz loop rate
            elapsed_time = time.perf_counter() - start_time
            sleep_time = 0.005 - elapsed_time # Target 5ms cycle time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else:
                # Loop took longer than 5ms, indicates potential performance issue
                # print(f"WARN: Control loop took {elapsed_time*1000:.1f} ms (> 5 ms)") # Uncomment for performance debugging

    # --- Exception Handling for the Thread ---
    except KeyboardInterrupt:
        # This thread might catch Ctrl+C if it's active, but usually the main thread does.
        print("Control loop thread interrupted by KeyboardInterrupt.")
    except Exception as e:
        print(f"FATAL ERROR in control loop: {e}")
        import traceback; traceback.print_exc() # Print detailed error traceback
    finally:
        # --- Cleanup: Close SPI when the thread finishes ---
        # This ensures SPI is closed even if the loop exits unexpectedly.
        if spi and spi_is_open:
            print("Closing SPI from control loop finally block...")
            try:
                spi.close()
                print("SPI closed by control loop.")
            except Exception as close_e:
                print(f"Error closing SPI in control loop: {close_e}")

# --- Main Program Execution ---
if __name__ == "__main__":

    # Exit if SPI initialization failed earlier
    if spi is None:
         print("Exiting: SPI initialization failed.")
         sys.exit(1)

    print("Attempting to initialize LIDAR...")
    # Initialize LIDAR and start its background thread
    if inicializar_lidar_en_fondo():
        control_thread = None # Reference to the control loop thread
        try:
            # Start the main control loop in a separate thread
            print("Starting main control loop thread...")
            control_thread = threading.Thread(target=loopprincipal, daemon=True)
            control_thread.start()

            # --- Simple Main Thread Loop ---
            # The main thread now simply waits for the control thread to finish
            # or for a KeyboardInterrupt (Ctrl+C).
            while control_thread.is_alive():
                # Keep the main thread alive, checking periodically
                time.sleep(0.5)
            # If the loop exits, it means the control_thread finished (e.g., due to an error)
            print("Control thread has finished execution.")
            # ---------------------------

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected in main thread. Sending stop command and shutting down...")

            # --- SEND ZERO VELOCITY COMMAND ON SHUTDOWN ---
            if spi_is_open and spi:
                try:
                    print("Sending stop command (ux=0, uy=0)...")
                    tx_payload_stop = (
                        float_to_bytes_with_checksum(0.0) +
                        float_to_bytes_with_checksum(0.0) +
                        float_to_bytes_with_checksum(maxspeed) # Keep maxspeed limit
                    )
                    if len(tx_payload_stop) == 15:
                        tx_buffer_stop = list(tx_payload_stop) + [0] * 10
                        spi.xfer2(tx_buffer_stop) # Send the stop command
                        print("Stop command sent.")
                        time.sleep(0.05) # Brief pause to allow command processing
                    else:
                        print("ERROR creating stop payload.")
                except Exception as spi_stop_e:
                    print(f"ERROR sending stop command via SPI: {spi_stop_e}")
            else:
                 print("SPI not open, cannot send stop command.")
            # --- END OF SENDING STOP COMMAND ---

        except Exception as main_e:
             print(f"Unexpected error in main thread: {main_e}")
             import traceback; traceback.print_exc()

        finally:
            # --- Graceful Shutdown Sequence ---
            print("\nInitiating final shutdown sequence...")
            # 1. Stop the LIDAR module (stops its thread and hardware)
            detener_lidar()

            # 2. Wait briefly for the control thread to finish (it might have already stopped)
            if control_thread is not None and control_thread.is_alive():
                 print("Waiting for control thread to terminate...")
                 control_thread.join(timeout=1.0) # Wait up to 1 second
                 if control_thread.is_alive():
                     print("WARN: Control thread did not terminate cleanly.")

            # 3. Close SPI (as a final safeguard, though loopprincipal should also close it)
            if spi_is_open and spi:
                 try:
                     spi.close()
                     print("SPI closed from main finally block.")
                     spi_is_open = False
                 except Exception:
                     # Ignore errors here, might already be closed
                     pass

            print("Program finished.")
            # --- End Shutdown Sequence ---

    else: # lidar_reader.inicializar_lidar_en_fondo() returned False
        print("CRITICAL ERROR: Could not initialize LIDAR module.")
        # Ensure SPI is closed if it was opened
        if spi_is_open and spi:
            try: spi.close()
            except: pass
        sys.exit(1) # Exit with an error code
