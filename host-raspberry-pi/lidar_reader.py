
import threading
import time
import sys

# Asegúrate de que esta ruta sea correcta para tu sistema
sys.path.append("/usr/local/lib/python3.12/dist-packages/ydlidar-1.2.4-py3.12-linux-aarch64.egg/")
try:
    import ydlidar
except ImportError:
    print("Error: No se pudo importar ydlidar. Verifica la ruta en sys.path.append.")
    sys.exit(1)

# --- Variables Globales del Módulo ---
# Usamos un diccionario vacío como "puntero" inicial.
_datos_lidar = {}
_lock = threading.Lock() # Lock interno del módulo

# Control del hilo
_thread = None
_running = False
_laser = None
_scan = None # Inicializar como None

def _leer_lidar():
    """
    Función que corre en el hilo de fondo. Lee y pre-procesa datos LIDAR.
    """
    global _datos_lidar, _running, _laser, _scan

    if _scan is None:
        _scan = ydlidar.LaserScan()

    while _running:
        try:
            if _laser.doProcessSimple(_scan):
                # 1. PRE-PROCESAR DATOS FUERA DEL LOCK
                ang = []
                dist = []
                inten = []
                points_data = _scan.points if hasattr(_scan, 'points') else []
                for p in points_data:
                    ang.append(-p.angle) # Invertir ángulo AQUÍ
                    dist.append(p.range)
                    inten.append(p.intensity)

                new_data = {
                    "angle": ang,
                    "range": dist,
                    "intensity": inten
                }

                # 2. SECCIÓN CRÍTICA MÍNIMA (MUY RÁPIDA)
                # Solo actualiza la referencia al diccionario.
                with _lock:
                    _datos_lidar = new_data

            # else: # Si no hubo scan, podrías añadir una pausa mínima
                # time.sleep(0.001)

        except Exception as e:
            print(f"Error en hilo _leer_lidar: {e}")
            time.sleep(0.1) # Pausa antes de reintentar

        # 3. ELIMINADO time.sleep(0.01) - innecesario

def inicializar_lidar_en_fondo():
    """
    Configura, inicializa el LIDAR e inicia el hilo de lectura.
    """
    global _laser, _thread, _running, _scan
    if _thread is not None and _thread.is_alive(): return True # Ya inicializado

    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    detected_port = None
    for key, value in ports.items(): detected_port = value; break
    if detected_port: port = detected_port
    else: print(f"WARN: Puerto LIDAR no detectado, usando default: {port}")
    print(f"Usando puerto LIDAR: {port}")

    _laser = ydlidar.CYdLidar()
    # === TU CONFIGURACIÓN ORIGINAL ===
    _laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    _laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    _laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    _laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    _laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    _laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
    _laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    _laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    _laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    _laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    _laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
    _laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
    # ==================================
    _scan = ydlidar.LaserScan() # Crear instancia Scan

    ret = _laser.initialize()
    if ret:
        ret = _laser.turnOn()
        if ret:
            _running = True
            _thread = threading.Thread(target=_leer_lidar, daemon=True)
            _thread.start()
            print("LIDAR inicializado y leyendo en fondo...")
            return True
        else:
            print("Error: No se pudo encender el LIDAR"); _laser.disconnecting(); return False
    else:
        print("Error: No se pudo inicializar el LIDAR"); return False

def obtener_datos():
    """
    Obtiene una copia segura y pre-procesada de los últimos datos LIDAR.
    Optimizado para mínima contención del lock.
    """
    global _datos_lidar

    # 1. SECCIÓN CRÍTICA MÍNIMA (MUY RÁPIDA)
    # Obtiene la referencia al último diccionario completo.
    with _lock:
        current_data = _datos_lidar

    # 2. COPIA LOS DATOS FUERA DEL LOCK
    if current_data:
        # El [:] hace una copia superficial (eficiente para listas de números)
        ang = current_data["angle"][:]
        dist = current_data["range"][:]
        inten = current_data["intensity"][:]
        # El ángulo ya viene invertido desde _leer_lidar
        return ang, dist, inten
    else:
        return [], [], [] # Retorna vacío si no hay datos aún

def detener_lidar():
    """ Detiene el hilo y apaga el LIDAR de forma segura. """
    global _running, _thread, _laser
    if not _running: return
    _running = False
    print("Deteniendo el LIDAR...")
    if _thread is not None and _thread.is_alive():
        _thread.join(timeout=0.5)
    if _laser:
        try: _laser.turnOff(); _laser.disconnecting()
        except Exception as e: print(f"Error al detener LIDAR: {e}")
    print("LIDAR detenido.")
    _thread = None; _laser = None

# (Bloque if _name_ == "_main_": opcional para pruebas rápidas aquí)