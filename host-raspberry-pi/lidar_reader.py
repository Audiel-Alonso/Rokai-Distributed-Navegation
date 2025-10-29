# -*- coding: utf-8 -*-
"""
lidar_reader.py (or osensores.py)

Module for handling YDLIDAR sensor readings asynchronously using a background thread.

This module initializes the YDLIDAR sensor, reads scan data in a separate thread
to avoid blocking the main application, pre-processes the data (e.g., inverts angles),
and provides a thread-safe way to access the latest available scan data.

Key Features:
- Initializes and configures the YDLIDAR.
- Runs LIDAR data acquisition in a dedicated background thread.
- Pre-processes scan data (inverts angles).
- Provides a thread-safe function (`obtener_datos`) to get the latest scan.
- Handles clean shutdown of the LIDAR and the background thread.
- Optimized for minimal lock contention between the reader thread and data consumers.

Usage:
1. Call `inicializar_lidar_en_fondo()` to start the sensor and background thread.
2. In your main loop, call `obtener_datos()` frequently to get the latest scan data.
3. Call `detener_lidar()` before your application exits to ensure clean shutdown.
"""

import threading
import time
import sys
import os # Added for path joining

# --- YDLIDAR Library Path Configuration ---
# Ensure this path points correctly to your ydlidar .egg file or installation directory
# Example using relative path (adjust if needed)
# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# EGG_PATH = os.path.join(SCRIPT_DIR, "../path/to/your/ydlidar-1.2.4-py3.12-linux-aarch64.egg")
EGG_PATH = "/usr/local/lib/python3.12/dist-packages/ydlidar-1.2.4-py3.12-linux-aarch64.egg/" # Your specific path
if EGG_PATH not in sys.path:
    sys.path.append(EGG_PATH)

try:
    import ydlidar
except ImportError:
    print(f"FATAL ERROR: Could not import ydlidar. Check the path in sys.path: {EGG_PATH}")
    sys.exit(1)

# --- Module Global Variables ---

# Stores the latest processed LIDAR scan data.
# Format: {"angle": [float,...], "range": [float,...], "intensity": [float,...]}
# Initialized as empty dict to indicate no data received yet.
_latest_lidar_data = {}
# Thread lock to ensure thread-safe access to _latest_lidar_data.
_lock = threading.Lock()

# Background thread management
_lidar_thread = None
_running = False
_laser = None
_scan = None # LaserScan object, initialized later for thread safety if needed

def _read_lidar_loop():
    """
    Internal function executed by the background thread.
    Continuously reads from the LIDAR, processes data, and updates the shared variable.
    """
    global _latest_lidar_data, _running, _laser, _scan

    # Best practice: Initialize objects used only by this thread here.
    if _scan is None:
        _scan = ydlidar.LaserScan()
        if _scan is None:
             print("FATAL ERROR: Failed to create ydlidar.LaserScan object in thread.")
             _running = False # Stop the thread if object creation fails
             return

    print("LIDAR reading thread started.")
    while _running:
        try:
            # doProcessSimple is a blocking call, waits for a full scan (~100ms at 10Hz)
            if _laser and hasattr(_laser, 'doProcessSimple') and _laser.doProcessSimple(_scan):
                # 1. PROCESS DATA OUTSIDE THE LOCK for efficiency
                angles_rad = []
                ranges_m = []
                intensities = []

                # Check if _scan has 'points' attribute before iterating
                points_data = _scan.points if _scan and hasattr(_scan, 'points') else []

                for point in points_data:
                    # Pre-process: Invert angle (adjust sign if needed for your coordinate system)
                    angles_rad.append(-point.angle) # Angle in radians
                    ranges_m.append(point.range)     # Range in meters
                    intensities.append(point.intensity) # Intensity value

                # Prepare the new data dictionary
                new_data = {
                    "angle": angles_rad,
                    "range": ranges_m,
                    "intensity": intensities
                }

                # 2. MINIMAL CRITICAL SECTION (VERY FAST)
                # Update the shared variable with the new data reference.
                with _lock:
                    _latest_lidar_data = new_data

            # else:
                # Optional: Handle cases where doProcessSimple returns false (e.g., timeout)
                # print("Warning: doProcessSimple did not return a valid scan.")
                # time.sleep(0.001) # Small delay if no scan to prevent busy-wait

        except AttributeError as ae:
             # Catch cases where _laser might become None unexpectedly during shutdown
             print(f"AttributeError in _read_lidar_loop (likely during shutdown): {ae}")
             time.sleep(0.1)
        except Exception as e:
            print(f"ERROR in _read_lidar_loop: {e}")
            # Consider adding more robust error handling, e.g., attempt reconnect?
            time.sleep(0.5) # Pause before retrying after an error

    print("LIDAR reading thread stopped.")


def inicializar_lidar_en_fondo():
    """
    Initializes the YDLIDAR sensor and starts the background reading thread.

    Configures the LIDAR parameters (port, baudrate, scan frequency, etc.),
    turns on the laser, and launches the `_read_lidar_loop` in a daemon thread.

    Returns:
        bool: True if initialization was successful, False otherwise.
    """
    global _laser, _lidar_thread, _running, _scan
    # Prevent re-initialization if already running
    if _lidar_thread is not None and _lidar_thread.is_alive():
        print("INFO: LIDAR already initialized and running.")
        return True

    print("Initializing YDLIDAR...")
    try:
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar" # Default port
        detected_port = None
        # Simple detection: use the first available port
        for key, value in ports.items():
            detected_port = value
            break # Use the first one found

        if detected_port:
            port = detected_port
            print(f"Detected LIDAR port: {port}")
        else:
            print(f"WARN: Could not auto-detect LIDAR port. Using default: {port}")

        _laser = ydlidar.CYdLidar()

        # --- LIDAR Configuration (Your Original Settings) ---
        _laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        _laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400) # Common for YDLIDAR
        _laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE) # Check if correct for your model
        _laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        _laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0) # Target scan rate (Hz)
        _laser.setlidaropt(ydlidar.LidarPropSampleRate, 4) # Check datasheet for optimal value (e.g., 4, 5, 9)
        _laser.setlidaropt(ydlidar.LidarPropSingleChannel, False) # Check if your LIDAR is single/dual channel
        _laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0) # Max angle (degrees)
        _laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0) # Min angle (degrees)
        _laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0) # Max range (meters)
        _laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02) # Min range (meters) - adjust based on specs
        _laser.setlidaropt(ydlidar.LidarPropIntenstiy, True) # Enable intensity readings
        # --- End Configuration ---

        # Initialize LaserScan object here if required by library specifics
        _scan = ydlidar.LaserScan()

        # Attempt to initialize the laser driver
        ret = _laser.initialize()
        if ret:
            print("YDLIDAR Driver Initialized.")
            # Attempt to turn on the laser scanning
            ret = _laser.turnOn()
            if ret:
                print("YDLIDAR Laser Turned On.")
                _running = True
                # Start the background thread as a daemon thread
                _lidar_thread = threading.Thread(target=_read_lidar_loop, daemon=True)
                _lidar_thread.start()
                print("LIDAR background reading thread started.")
                return True
            else:
                print("ERROR: Failed to turn on LIDAR laser.")
                error_msg = _laser.getLastError() if hasattr(_laser, 'getLastError') else "N/A"
                print(f"Laser Error: {error_msg}")
                _laser.disconnecting() # Clean up driver
                return False
        else:
            print("ERROR: Failed to initialize YDLIDAR driver.")
            error_msg = _laser.getLastError() if hasattr(_laser, 'getLastError') else "N/A"
            print(f"Initialization Error: {error_msg}")
            return False

    except Exception as e:
        print(f"FATAL ERROR during LIDAR initialization: {e}")
        # Clean up in case of partial initialization
        if _laser:
            try: _laser.disconnecting()
            except: pass
        _laser = None
        return False

def obtener_datos():
    """
    Retrieves a thread-safe copy of the latest available LIDAR scan data.

    This function is optimized for speed by minimizing the time the internal
    lock is held. It performs data copying outside the critical section.

    Returns:
        tuple: A tuple containing three lists:
               (angles [radians], ranges [meters], intensities [unitless]).
               Returns empty lists ([], [], []) if no data is available yet.
               Angles are relative to the LIDAR's forward direction and are
               pre-inverted (negative sign applied) as processed by the reader thread.
    """
    global _latest_lidar_data

    # 1. MINIMAL CRITICAL SECTION (VERY FAST)
    # Get the reference to the latest data dictionary.
    with _lock:
        current_data = _latest_lidar_data

    # 2. COPY DATA OUTSIDE THE LOCK
    # Check if the dictionary is not empty (i.e., first scan received)
    if current_data:
        # Use slicing [:] to create shallow copies of the lists.
        # This is efficient for lists of numbers and ensures the caller
        # gets a snapshot that won't be modified by the reader thread.
        ang = current_data.get("angle", [])[:] # Use .get with default and copy
        dist = current_data.get("range", [])[:]
        inten = current_data.get("intensity", [])[:]
        return ang, dist, inten
    else:
        # Return empty lists if no data has been received yet
        return [], [], []

def detener_lidar():
    """
    Safely stops the background LIDAR thread and turns off the laser.

    Should be called before the main application exits.
    """
    global _running, _lidar_thread, _laser
    if not _running:
        print("INFO: LIDAR already stopped or not initialized.")
        return

    print("Stopping LIDAR...")
    _running = False # Signal the thread to stop

    # Wait briefly for the thread to exit its loop
    if _lidar_thread is not None and _lidar_thread.is_alive():
        print("Waiting for LIDAR thread to join...")
        _lidar_thread.join(timeout=1.0) # Wait up to 1 second
        if _lidar_thread.is_alive():
            print("WARN: LIDAR thread did not exit cleanly after 1 second.")

    # Turn off laser and disconnect driver
    if _laser:
        print("Turning off laser and disconnecting driver...")
        try:
            _laser.turnOff()
            _laser.disconnecting()
            print("LIDAR laser off and driver disconnected.")
        except Exception as e:
            print(f"ERROR during LIDAR shutdown: {e}")

    # Clear references
    _lidar_thread = None
    _laser = None
    print("LIDAR shutdown complete.")

# --- Optional: Self-test block ---
if __name__ == "__main__":
    """ Quick test to run if the module is executed directly """
    print("Running LIDAR reader module self-test...")
    if inicializar_lidar_en_fondo():
        try:
            print("LIDAR initialized. Reading scans for 5 seconds...")
            for i in range(10): # Try getting data 10 times
                time.sleep(0.5)
                angles, ranges, intensities = obtener_datos()
                if ranges:
                    print(f"Scan {i+1}: Received {len(ranges)} points. First point: range={ranges[0]:.2f}m, angle={math.degrees(angles[0]):.1f}deg")
                else:
                    print(f"Scan {i+1}: Waiting for data...")
            print("Test complete.")
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            detener_lidar()
    else:
        print("Self-test failed: Could not initialize LIDAR.")
