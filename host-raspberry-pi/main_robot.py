import spidev
import time
import struct
import numpy as np
import threading
import math
# Se eliminan las importaciones de matplotlib
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# Importa el módulo OPTIMIZADO que se llama 'osensores.py'
try:
    from lidar_reader import inicializar_lidar_en_fondo, obtener_datos, detener_lidar
except ImportError:
    print("Error: No se pudo importar el módulo 'osensores.py'. Asegúrate de que esté en la misma carpeta.")
    sys.exit(1)

# --- Lock para proteger estado del robot ---
# Útil si accedes a robot_state desde otros hilos
robot_state_lock = threading.Lock()

# --- Configuración SPI ---
spi = spidev.SpiDev()
spi_bus = 0
spi_device = 0
spi_speed = 4000000 # 4 MHz
spi_mode = 0b00
spi_is_open = False # Flag para saber si SPI se abrió correctamente

try:
    spi.open(spi_bus, spi_device)
    spi.max_speed_hz = spi_speed
    spi.mode = spi_mode
    spi_is_open = True
    print("SPI inicializado correctamente.")
except Exception as e:
    print(f"Error CRÍTICO al inicializar SPI: {e}")
    spi = None # SPI no está disponible

# --- Parámetros Robot/Control (Tus valores originales) ---
maxspeed = 0.45; Katrac = 0.08; Krep1 = 0.05
dmin=0.10; dmax=0.50 # dmax estaba en 0.50 en tu último código
xd = 0; yd = 0.0 # Objetivo (usar directamente)

# --- Estado Robot (Compartido, protegido por robot_state_lock) ---
robot_state = {'x': 0.0, 'y': 0.0, 'xp': 0.0, 'yp': 0.0, 'th': 0.0}

# --- Funciones SPI (Tus funciones originales) ---
def float_to_bytes_with_checksum(f):
    """ Empaqueta un float y añade checksum. Devuelve 5 bytes o bytes nulos en error."""
    try:
        b = list(struct.pack('<f', f))
        checksum = sum(b) & 0xFF
        return bytes(b + [checksum])
    except Exception as e:
        # print(f"Error empaquetando float {f}: {e}") # Descomentar para depurar
        return bytes([0]*5)

def extract_float_and_verify(b5):
    """ Desempaqueta 5 bytes, verifica checksum y devuelve float o None si falla. """
    if len(b5) != 5: return None
    data = b5[:4]; checksum = b5[4]
    if sum(data) & 0xFF == checksum:
        try: return struct.unpack('<f', bytes(data))[0]
        except struct.error: return None
    else: # Error checksum
        # print(f"Error checksum RX: R={checksum}, C={sum(data) & 0xFF}") # Descomentar para depurar
        return None

# --- Tus Funciones de Cálculo de Fuerzas (SIN CAMBIOS) ---
def Sensado(distancias, angulos, th):
    """ Calcula coordenadas globales de puntos LIDAR según tu método original. """
    xs = []; ys = []
    pl=0.118; h=0.1 # Tus constantes
    for i in range(len(distancias)):
        ang_global = angulos[i] + th
        # TU CÁLCULO ORIGINAL:
        xse = distancias[i] * math.cos(ang_global) + (pl-h) * math.cos(th)
        yse = distancias[i] * math.sin(ang_global) + (pl-h) * math.sin(th)
        xs.append(xse)
        ys.append(yse)
    return xs, ys

def F_rep(distancias, angulos, th):
    """ Calcula fuerza de repulsión según tu método original. """
    S = Sensado(distancias, angulos, th)
    xs = S[0]; ys = S[1]
    # TU CÁLCULO ORIGINAL (con posible dilución de Krep):
    Krep = (Krep1 / len(xs)) if xs else 0
    Frepx = 0; Frepy = 0
    for i in range(len(xs)):
        dsi_sq = xs[i]*2 + ys[i]*2
        # Evitar división por cero si el punto está en el origen
        if dsi_sq < 1e-9: continue # Saltar este punto si está muy cerca del origen
        dsi = math.sqrt(dsi_sq)

        if (dmin < dsi <= dmax):
            # Tu fórmula original
            factor = (-Krep / dsi**3) * (1/dsi - 1/dmax)
            Frepxi = factor * xs[i]
            Frepyi = factor * ys[i]
        else:
            Frepxi = 0; Frepyi = 0
        Frepx += Frepxi
        Frepy += Frepyi
    return Frepx, Frepy

# --- Hilo Principal de Control (Tu lógica original adaptada) ---
def loopprincipal():
    """ Bucle que calcula fuerzas, comunica por SPI y actualiza estado. """
    global robot_state, xd, yd, spi # Acceder a globales

    if spi is None or not spi_is_open:
        print("Error: SPI no está disponible en loopprincipal.")
        return # Salir del hilo si SPI no funcionó

    try:
        time.sleep(1) # Espera inicial

        while True:
            start_time = time.perf_counter()

            # 1. Obtener datos LIDAR (optimizado)
            angulos, distancias, _ = obtener_datos()

            # 2. Obtener estado robot (protegido)
            with robot_state_lock:
                current_x = robot_state['x']
                current_y = robot_state['y']
                current_th = robot_state['th']

            # 3. Calcular fuerzas (USANDO TUS FUNCIONES ORIGINALES)
            frepx, frepy = F_rep(distancias, angulos, current_th)
            fatracx = -Katrac * (current_x - xd)
            fatracy = -Katrac * (current_y - yd)
            ux = fatracx + frepx # Comando X global
            uy = fatracy + frepy # Comando Y global

            # (Opcional: Limitar velocidad total ux, uy si es necesario aquí)

            # 4. Enviar por SPI
            tx_payload = (
                float_to_bytes_with_checksum(ux) +
                float_to_bytes_with_checksum(uy) +
                float_to_bytes_with_checksum(maxspeed)
            )
            if len(tx_payload) != 15:
                print(f"Error creando payload SPI TX (len={len(tx_payload)})")
                time.sleep(0.005); continue

            tx_buffer = list(tx_payload) + [0] * 10 # Relleno para recibir 25 bytes
            rx_bytes = bytes()
            try:
                rx_bytes_list = spi.xfer2(tx_buffer)
                rx_bytes = bytes(rx_bytes_list)
            except Exception as spi_e:
                print(f"Error durante spi.xfer2: {spi_e}")
                # Considera reintentar o cerrar SPI si el error persiste
                time.sleep(0.01); continue

            # 5. Procesar recepción
            if len(rx_bytes) == 25:
                x_new = extract_float_and_verify(rx_bytes[0:5])
                y_new = extract_float_and_verify(rx_bytes[5:10])
                xp_new = extract_float_and_verify(rx_bytes[10:15])
                yp_new = extract_float_and_verify(rx_bytes[15:20])
                th_new = extract_float_and_verify(rx_bytes[20:25])

                if None not in (x_new, y_new, xp_new, yp_new, th_new):
                    # Actualizar estado (protegido)
                    with robot_state_lock:
                        robot_state['x'] = x_new; robot_state['y'] = y_new
                        robot_state['xp'] = xp_new; robot_state['yp'] = yp_new
                        robot_state['th'] = th_new

                    # --- TU IMPRESIÓN DE DEPURACIÓN ORIGINAL ---
                    print(f"Rec: x:{x_new:.3f}, y:{y_new:.3f}, xp:{xp_new:.3f}, yp:{yp_new:.3f}, th:{math.degrees(th_new):.1f} | "\
                          f"fax:{fatracx:.3f}, fay:{fatracy:.3f}, frx:{frepx:.3f}, fry:{frepy:.3f}",
                          flush=True)
                    # ---------------------------------------------
                # else: # Checksum error ya manejado en extract_...
                    # print("Error checksum/recepción") # Descomentar si es necesario
                    # pass

            else: print(f"Error SPI RX: {len(rx_bytes)} bytes recibidos, se esperaban 25.")

            # Control frecuencia bucle (~200Hz)
            elapsed_time = time.perf_counter() - start_time
            sleep_time = 0.005 - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else:
                # print(f"WARN: Ciclo tardó {elapsed_time*1000:.1f} ms") # Descomentar si sospechas lentitud

    # Este bloque se ejecutará si el bucle while True termina (ej. por un error no capturado)
    except KeyboardInterrupt:
        # Esto normalmente se captura en el hilo principal, pero por si acaso
        print("Hilo control interrumpido por KeyboardInterrupt.")
    except Exception as e:
        print(f"Error fatal en loopprincipal: {e}")
        import traceback; traceback.print_exc()
    finally:
        # Asegurarse de cerrar SPI al terminar el hilo
        if spi and spi_is_open:
            print("Cerrando SPI desde loopprincipal..."); spi.close(); print("SPI cerrado.")

# --- Programa Principal (Inicia hilos, sin gráfica) ---
if _name_ == "_main_":
    if spi is None:
         print("Saliendo: SPI no pudo inicializarse.")
         sys.exit(1)

    # Usar funciones del módulo osensores
    if inicializar_lidar_en_fondo():
        control_thread = None
        try:
            # Iniciar hilo de control
            control_thread = threading.Thread(target=loopprincipal, daemon=True)
            control_thread.start()

            # --- Bucle Principal Simple ---
            # Espera a Ctrl+C o a que el hilo termine
            while control_thread.is_alive():
                time.sleep(0.5)
            # Si salimos del bucle while, significa que el hilo terminó (quizás por error)
            print("El hilo de control ha terminado.")
            # ---------------------------

        except KeyboardInterrupt:
            print("\nInterrupción en main. Enviando comando de parada y apagando...")

            # --- ENVIAR COMANDO DE VELOCIDAD CERO ---
            if spi_is_open and spi:
                try:
                    print("Enviando comando de parada (ux=0, uy=0)...")
                    tx_payload_stop = (
                        float_to_bytes_with_checksum(0.0) +
                        float_to_bytes_with_checksum(0.0) +
                        float_to_bytes_with_checksum(maxspeed) # Mantener límite
                    )
                    if len(tx_payload_stop) == 15:
                        tx_buffer_stop = list(tx_payload_stop) + [0] * 10
                        spi.xfer2(tx_buffer_stop) # Enviar parada
                        print("Comando de parada enviado.")
                        time.sleep(0.01) # Pequeña pausa para asegurar envío
                    else:
                        print("Error creando payload de parada.")
                except Exception as spi_stop_e:
                    print(f"Error al enviar comando de parada por SPI: {spi_stop_e}")
            else:
                 print("SPI no estaba abierto, no se puede enviar comando de parada.")
            # --- FIN DEL ENVÍO DE PARADA ---

        except Exception as main_e:
             print(f"Error inesperado en main: {main_e}")
             import traceback; traceback.print_exc()

        finally:
            # Secuencia de apagado ordenada
            print("\nIniciando secuencia de apagado final...")
            detener_lidar() # Detener LIDAR y su hilo

            # Esperar (un poco) a que el hilo de control termine si aún no lo ha hecho
            if control_thread is not None and control_thread.is_alive():
                 print("Esperando que hilo de control termine..."); control_thread.join(timeout=1.0)
                 if control_thread.is_alive(): print("WARN: Hilo control no terminó.")

            # Cerrar SPI si sigue abierto (doble chequeo por seguridad)
            if spi_is_open and spi:
                 try:
                     # Verificar si el descriptor de archivo aún es válido antes de cerrar
                     # Esto depende de la implementación de spidev, puede no ser necesario
                     # if spi.fileno() >= 0:
                     spi.close()
                     print("SPI cerrado desde main finally.")
                     spi_is_open = False # Marcar como cerrado
                 except Exception as e:
                     # Puede dar error si ya estaba cerrado por el hilo
                     # print(f"Error menor cerrando SPI en main finally (puede ser normal): {e}")
                     pass

            print("Programa finalizado.")
    else:
        print("Error crítico: No se pudo iniciar el LIDAR.")
        sys.exit(1)