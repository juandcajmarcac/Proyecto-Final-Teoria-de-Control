from djitellopy import Tello
import time
import matplotlib.pyplot as plt

# ==========================================
# 1. PARÁMETROS DE CONTROL
# ==========================================
Kp = 160.0       # Ganancia proporcional (PRUEBA SOLO P)

Z_REF = 1.10     # Referencia de altura (m)
T_MAX_VUELO = 25.0
T_LOOP_OBJ = 0.1    # 10 Hz

VZ_MAX = 40      # Saturación (cm/s)
Z_TECHO = 1.90
Z_SUELO = 0.40

# ==========================================
# 2. INICIALIZACIÓN DEL DRON
# ==========================================
tello = Tello()
tello.connect()
tello.streamoff()

print(f"\n>>> BATERÍA: {tello.get_battery()}% <<<\n")

print(">>> DESPEGANDO...")
tello.takeoff()
time.sleep(2)

# ==========================================
# 3. VARIABLES DE CONTROL
# ==========================================
dist_ini = tello.get_distance_tof()
z_medida = dist_ini / 100.0 if 0 < dist_ini < 300 else 0.0

u_anterior = 0.0

log_t = []
log_z = []
log_ref = []
log_u = []

t0 = time.time()
t_prev = time.time()

# ==========================================
# 4. BUCLE PRINCIPAL (CONTROL P)
# ==========================================
try:
    while True:
        t = time.time()
        dt = t - t_prev
        if dt <= 0.01:
            continue
        t_prev = t

        tiempo = t - t0
        if tiempo >= T_MAX_VUELO:
            break

        # --------- SENSOR ToF ----------
        dist = tello.get_distance_tof()
        if 0 < dist < 300:
            z_nueva = dist / 100.0
            if abs(z_nueva - z_medida) < 0.4:
                z_medida = z_nueva

        # --------- ERROR ----------
        error = Z_REF - z_medida

        # --------- CONTROL PROPORCIONAL ----------
        u = Kp * error

        # --------- SATURACIÓN ----------
        u = max(min(u, VZ_MAX), -VZ_MAX)

        # --------- SEGURIDAD ----------
        if z_medida > Z_TECHO and u > 0:
            u = 0
        if z_medida < Z_SUELO and u < 0:
            u = 0

        cmd_vz = int(round(u))
        tello.send_rc_control(0, 0, cmd_vz, 0)

        # --------- LOG ----------
        log_t.append(tiempo)
        log_z.append(z_medida)
        log_ref.append(Z_REF)
        log_u.append(cmd_vz)

        print(f"T={tiempo:.1f}s | Z={z_medida:.2f} | Err={error:.2f} | Cmd={cmd_vz}")

        # --------- FRECUENCIA ----------
        elapsed = time.time() - t
        if T_LOOP_OBJ > elapsed:
            time.sleep(T_LOOP_OBJ - elapsed)

except KeyboardInterrupt:
    print("\n>>> INTERRUPCIÓN <<<")

finally:
    # ==========================================
    # 5. ATERRIZAJE Y GRÁFICAS
    # ==========================================
    print(">>> ATERRIZANDO...")
    for _ in range(5):
        tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.05)
    tello.land()

    if len(log_t) > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        ax1.plot(log_t, log_z, 'b.-', label='Altura (m)')
        ax1.plot(log_t, log_ref, 'r--', label='Referencia')
        ax1.set_ylabel('Altura (m)')
        ax1.set_title(f'Control P de Altura\nKp={Kp}')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(log_t, log_u, 'g', label='Velocidad vertical (cm/s)')
        ax2.axhline(VZ_MAX, linestyle=':', color='orange')
        ax2.axhline(-VZ_MAX, linestyle=':', color='orange')
        ax2.set_ylabel('Velocidad (cm/s)')
        ax2.set_xlabel('Tiempo (s)')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.show()