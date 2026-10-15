import matplotlib.pyplot as plt
from simulator import getCDO, h_at_waypoints, generate_scenario_1_csv, generate_scenario_2_csv

# Identificadores de aeronaves y colores unificados para toda la simulación
Aircrafts = ["B767-300ER", "B777-300", "B737", "A320-212", "A319-131"]
Colors = ["sandybrown", "mediumseagreen", "darkcyan", "darkorchid", "hotpink", 
          "coral", "mediumspringgreen", "turquoise", "plum", "mediumpurple"]

# =============================================================================
# GRÁFICO 1: Perfil de Vuelo Espacial Global (Altitud h vs Distancia horizontal x)
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
# GRÁFICO 2: Perfil Temporal de Vuelo Global (Altitud h vs Tiempo t)
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


# =============================================================================
# ACTIVIDAD 4: CDO COMPLIANCE TO STAR ALTITUDE CONSTRAINTS (ANÁLISIS SECUENCIAL)
# =============================================================================

# Diccionario de restricciones de altitud oficiales calculadas por vuestro equipo
STAR_CONSTRAINTS = {
    'ALBER1Z': [
        {'name': 'ALBER', 'x': -129084.40, 'min': 3352.8, 'max': 7620.0},
        {'name': 'CUTXE', 'x': -92970.40,  'min': None,   'max': None},
        {'name': 'UTHAN', 'x': -51485.60,  'min': None,   'max': None},
        {'name': 'ENJUC', 'x': -32039.60,  'min': 2438.4, 'max': None},
        {'name': 'UCREQ', 'x': -19260.80,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ],
    'CASPE2W': [
        {'name': 'CASPE', 'x': -164087.20, 'min': None,   'max': 8534.4},
        {'name': 'MECUH', 'x': -88155.20,  'min': 4876.8, 'max': 6096.0},
        {'name': 'VIBOK', 'x': -50930.00,  'min': None,   'max': 4572.0},
        {'name': 'BL461', 'x': -18520.00,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ],
    'LOBAR2W': [
        {'name': 'LOBAR', 'x': -151308.40, 'min': None,   'max': 8534.4},
        {'name': 'PEKIS', 'x': -83340.00,  'min': 4876.8, 'max': 6096.0},
        {'name': 'BL461', 'x': -18520.00,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ],
    'MARTA3Z': [
        {'name': 'MARTA', 'x': -178347.60, 'min': None,   'max': 7315.2},
        {'name': 'EBROX', 'x': -138900.00, 'min': 3048.0, 'max': None},
        {'name': 'RES',   'x': -90192.40,  'min': None,   'max': None},
        {'name': 'VLA',   'x': -50930.00,  'min': None,   'max': None},
        {'name': 'BL463', 'x': -18520.00,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ],
    'MATEX3Z': [
        {'name': 'MATEX', 'x': -190015.20, 'min': None,   'max': 8534.4},
        {'name': 'SENIA', 'x': -137048.00, 'min': 3048.0, 'max': None},
        {'name': 'RES',   'x': -90192.40,  'min': None,   'max': None},
        {'name': 'VLA',   'x': -50930.00,  'min': None,   'max': None},
        {'name': 'BL463', 'x': -18520.00,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ],
    'PUMAL1Z': [
        {'name': 'PUMAL', 'x': -94452.00,  'min': 3962.4, 'max': 7620.0},
        {'name': 'BERGA', 'x': -72598.40,  'min': 3657.6, 'max': None},
        {'name': 'KOSIT', 'x': -46300.00,  'min': None,   'max': None},
        {'name': 'MAMUK', 'x': -35373.20,  'min': 2438.4, 'max': None},
        {'name': 'UCREQ', 'x': -19260.80,  'min': None,   'max': 3048.0},
        {'name': 'SLL',   'x': 0.00,       'min': 1828.8, 'max': None}
    ]
}

# Inicializamos el contador de figuras a partir de la ventana 3
fig_num = 3
for star_name, wps in STAR_CONSTRAINTS.items():
    # Creamos una ventana gráfica flotante limpia para la STAR actual
    plt.figure(fig_num, figsize=(10, 6))
    
    # 1. Dibujamos las 10 trayectorias teóricas calculadas de fondo como línea base
    ac_idx = 0
    while ac_idx < len(Aircrafts):
        # Configuración 100% MLW
        lectura_100 = getCDO(Aircrafts[ac_idx], 100, initial_h_ft=6000.0)
        plt.plot(lectura_100[0], lectura_100[1], color=Colors[ac_idx], alpha=0.5, linewidth=1.2,
                 label=f"{Aircrafts[ac_idx]} (100% MLW)")
        
        # Configuración 80% MLW
        lectura_80 = getCDO(Aircrafts[ac_idx], 80, initial_h_ft=6000.0)
        plt.plot(lectura_80[0], lectura_80[1], color=Colors[ac_idx+5], linestyle="--", alpha=0.5, linewidth=1.2,
                 label=f"{Aircrafts[ac_idx]} (80% MLW)")
        ac_idx += 1
        
    # 2. Dibujamos las líneas de restricción de la AIP de ENAIRE para esta ruta
    for wp in wps:
        # Barrera vertical del Waypoint
        plt.axvline(x=wp['x'], color="gray", linestyle="-", linewidth=0.7, alpha=0.4)
        
        # Nombre del Waypoint en la franja superior (libre de trazas de aeronaves)
        plt.text(wp['x'] + 1500, 11200, wp['name'], color="black", fontsize=8, fontweight="bold", rotation=90)
        
        # Techo de altitud máxima permitida (Línea de puntos carmesí)
        if wp['max'] is not None:
            plt.plot([wp['x'] - 16000, wp['x']], [wp['max'], wp['max']], color="crimson", linestyle=":", linewidth=1.5)
            plt.text(wp['x'] - 15000, wp['max'] + 150, f"Max:{wp['max']:.0f}m", color="crimson", fontsize=7, fontweight="bold")
            
        # Suelo de altitud mínima requerida (Línea de puntos azul)
        if wp['min'] is not None:
            plt.plot([wp['x'] - 16000, wp['x']], [wp['min'], wp['min']], color="royalblue", linestyle=":", linewidth=1.5)
            plt.text(wp['x'] - 15000, wp['min'] - 400, f"Min:{wp['min']:.0f}m", color="royalblue", fontsize=7, fontweight="bold")

    # 3. Configuración del cuadro de visualización actual
    plt.title(f"CDO Compliance & STAR Altitude Constraints - {star_name}", fontsize=11, fontweight="bold")
    plt.xlabel("Horizontal distance to the IAF, x [m]", fontsize=10)
    plt.ylabel("Pressure altitude, h [m]", fontsize=10)
    plt.xlim(-215000, 5000)
    plt.ylim(1000, 13000)
    plt.grid(True, linestyle="--", alpha=0.25)
    
    # Cuadro de leyendas posicionado de forma externa a la derecha
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize='8.5', frameon=True, shadow=True)
    
    # Dimensionamiento holgado de márgenes absolutos
    plt.subplots_adjust(left=0.08, right=0.75, top=0.92, bottom=0.1)
    
    # BARRERA BLOQUEANTE: Muestra la pantalla actual y retiene el bucle hasta su cierre
    plt.show()
    
    fig_num += 1

# =============================================================================
# EXPORTACIÓN AUTOMÁTICA DE ARCHIVOS CSV DE DATOS EN RUTA
# =============================================================================
print("\nGenerando tablas de datos finales...")
h_at_waypoints()
generate_scenario_1_csv()
generate_scenario_2_csv()
print("¡Todos los archivos CSV se han escrito correctamente en vuestro directorio!")
