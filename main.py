import matplotlib.pyplot as plt
from simulator import getCDO, h_at_waypoints

Aircrafts = ["B767-300ER", "B777-300", "B737", "A320-212", "A319-131"]
Colors = ["sandybrown", "mediumseagreen", "darkcyan", "darkorchid", "hotpink", 
          "coral", "mediumspringgreen", "turquoise", "plum", "mediumpurple"]

# =============================================================================
# GRÁFICO 1: Perfil de Vuelo Espacial (Altitud h vs Distancia horizontal x)
# =============================================================================
plt.figure(1, figsize=(10, 5))
i = 0
while i < len(Aircrafts):
    # Simulación al 100% MLW empezando desde los 6000 ft regulados del IAF
    lectura = getCDO(Aircrafts[i], 100, initial_h_ft=6000.0)
    CoordenadaX = lectura[0]
    CoordenadaY = lectura[1]
    plt.plot(CoordenadaX, CoordenadaY, color=Colors[i], label=Aircrafts[i] + " MLW: 100%")
    
    # Simulación al 80% MLW
    lectura = getCDO(Aircrafts[i], 80, initial_h_ft=6000.0)
    CoordenadaX = lectura[0]
    CoordenadaY = lectura[1]
    plt.plot(CoordenadaX, CoordenadaY, color=Colors[i+5], label=Aircrafts[i] + " MLW: 80%")
    i += 1

plt.legend()
plt.title("Simulated descent trajectories in a Continuous Descent Operation (CDO)")
plt.xlabel("Horizontal distance to the IAF, x (m)")
plt.ylabel("Pressure altitude, h (m)")
plt.grid(True, linestyle='--')

# =============================================================================
# GRÁFICO 2: Perfil Temporal de Vuelo (Altitud h vs Tiempo t)
# =============================================================================
plt.figure(2, figsize=(10, 5))
u = 0
while u < len(Aircrafts):
    # Simulación al 100% MLW
    lectura = getCDO(Aircrafts[u], 100, initial_h_ft=6000.0)
    CoordenadaY = lectura[1]
    Temps = lectura[2]
    plt.plot(Temps, CoordenadaY, color=Colors[u], label=Aircrafts[u] + " MLW: 100%")
    
    # Simulación al 80% MLW
    lectura = getCDO(Aircrafts[u], 80, initial_h_ft=6000.0)
    CoordenadaY = lectura[1]
    Temps = lectura[2]
    plt.plot(Temps, CoordenadaY, color=Colors[u+5], label=Aircrafts[u] + " MLW: 80%")
    u += 1

plt.legend()
plt.title("Simulated descent trajectories in a Continuous Descent Operation (CDO)")
plt.xlabel("Time relative to the IAF, t (s)")
plt.ylabel("Pressure altitude, h (m)")
plt.grid(True, linestyle='--')

# Mostramos ambos gráficos de forma independiente
plt.show()

h_at_waypoints()