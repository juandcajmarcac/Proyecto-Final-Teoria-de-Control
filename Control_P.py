from djitellopy import Tello
import time
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk

# ==========================================
# VARIABLES GLOBALES
# ==========================================
Z_REF = 1.10
T_LOOP_OBJ = 0.1

VZ_MAX = 40
Z_TECHO = 1.90
Z_SUELO = 0.40


# ==========================================
# FUNCIÓN PRINCIPAL DEL CONTROL
# ==========================================
def iniciar_control():

    # Leer valores de la interfaz
    Kp = float(entry_kp.get())
    T_MAX_VUELO = float(entry_time.get())

    # ---------------- DRON ----------------
    tello = Tello()
    tello.connect()
    tello.streamoff()

    print(f"\n>>> BATERÍA: {tello.get_battery()}% <<<\n")

    print(">>> DESPEGANDO...")
    tello.takeoff()
    time.sleep(2)

    # ---------------- VARIABLES ----------------
    dist_ini = tello.get_distance_tof()
    z_medida = dist_ini / 100.0 if 0 < dist_ini < 300 else 0.0

    log_t = []
    log_z = []
    log_ref = []
    log_u = []

    t0 = time.time()
    t_prev = time.time()

    # ---------------- CONTROL ----------------
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

            # -------- SENSOR --------
            dist = tello.get_distance_tof()

            if 0 < dist < 300:

                z_nueva = dist / 100.0

                if abs(z_nueva - z_medida) < 0.4:
                    z_medida = z_nueva


            # -------- ERROR --------
            error = Z_REF - z_medida


            # -------- CONTROL P --------
            u = Kp * error


            # -------- SATURACIÓN --------
            u = max(min(u, VZ_MAX), -VZ_MAX)


            # -------- SEGURIDAD --------
            if z_medida > Z_TECHO and u > 0:
                u = 0

            if z_medida < Z_SUELO and u < 0:
                u = 0


            cmd_vz = int(round(u))

            tello.send_rc_control(0, 0, cmd_vz, 0)


            # -------- LOG --------
            log_t.append(tiempo)
            log_z.append(z_medida)
            log_ref.append(Z_REF)
            log_u.append(cmd_vz)


            print(f"T={tiempo:.1f}s | Z={z_medida:.2f} | Err={error:.2f} | Cmd={cmd_vz}")


            # -------- FRECUENCIA --------
            elapsed = time.time() - t

            if T_LOOP_OBJ > elapsed:
                time.sleep(T_LOOP_OBJ - elapsed)


    except KeyboardInterrupt:

        print("\n>>> INTERRUPCIÓN <<<")


    finally:

        # ---------------- ATERRIZAJE ----------------
        print(">>> ATERRIZANDO...")

        for _ in range(5):
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.05)

        tello.land()


        # ---------------- GRÁFICAS ----------------
        if len(log_t) > 0:

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

            ax1.plot(log_t, log_z, 'b.-', label='Altura')
            ax1.plot(log_t, log_ref, 'r--', label='Referencia')

            ax1.set_ylabel('Altura (m)')
            ax1.set_title(f'Control P - Kp = {Kp}')
            ax1.grid(True)
            ax1.legend()


            ax2.plot(log_t, log_u, 'g', label='Velocidad')

            ax2.axhline(VZ_MAX, linestyle=':', color='orange')
            ax2.axhline(-VZ_MAX, linestyle=':', color='orange')

            ax2.set_ylabel('Velocidad (cm/s)')
            ax2.set_xlabel('Tiempo (s)')
            ax2.grid(True)
            ax2.legend()


            plt.tight_layout()
            plt.show()



# ==========================================
# INTERFAZ GRÁFICA
# ==========================================

root = tk.Tk()
root.title("Control P - Dron Tello")
root.geometry("400x250")


# -------- TÍTULO --------
label_title = ttk.Label(
    root,
    text="Control Proporcional - DJI Tello",
    font=("Arial", 14, "bold")
)

label_title.pack(pady=10)


# -------- KP --------
frame_kp = ttk.Frame(root)
frame_kp.pack(pady=5)

ttk.Label(frame_kp, text="Kp: ").pack(side="left")

entry_kp = ttk.Entry(frame_kp, width=10)
entry_kp.pack(side="left")

entry_kp.insert(0, "160")


# -------- TIEMPO --------
frame_time = ttk.Frame(root)
frame_time.pack(pady=5)

ttk.Label(frame_time, text="Tiempo de vuelo (s): ").pack(side="left")

entry_time = ttk.Entry(frame_time, width=10)
entry_time.pack(side="left")

entry_time.insert(0, "25")


# -------- BOTÓN --------
btn_start = ttk.Button(
    root,
    text="Iniciar Vuelo",
    command=iniciar_control
)

btn_start.pack(pady=20)


# -------- MENSAJE --------
label_info = ttk.Label(
    root,
    text="Asegúrate de estar conectado al dron",
    foreground="blue"
)

label_info.pack()


root.mainloop()