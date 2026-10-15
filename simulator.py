import math
from datetime import datetime, timedelta

# CONSTANTES OFICIALES ISA Y FÍSICAS
g0 = 9.80665     # Gravedad estándar exacta (m/s^2)
R = 287.04       # Constante de gases ideales del aire (m^2/K s^2)
T0 = 288.15      # Temperatura estándar a nivel del mar (K)
p0 = 101325      # Presión estándar a nivel del mar (Pa)
delta_t = 1.0    # Paso de tiempo (1 segundo)

AIRCRAFT_PARAMS = {
    'B767-300ER': {
        'MLW': 145.150e3, 'S': 283.50, 'CD0_app': 0.014, 'CD2_app': 0.049,
        'CD0_clean': 0.0174, 'CD2_clean': 0.0459, 'hp_desc': 26418,
        'CT_desc_high': 0.064359, 'CT_desc_low': 0.055988, 'CT_desc_app': 0.12475,
        'CT1': 351670.0, 'CT2': 44673.0, 'CT3': 0.10129E-09,
        'CF1': 0.54005, 'CF2': 557.82
    },
    'B777-300': {
        'MLW': 237.680e3, 'S': 428.04, 'CD0_app': 0.0173, 'CD2_app': 0.0484,
        'CD0_clean': 0.0157, 'CD2_clean': 0.0420, 'hp_desc': 36122,
        'CT_desc_high': 0.044239, 'CT_desc_low': 0.041065, 'CT_desc_app': 0.092921,
        'CT1': 425770.0, 'CT2': 48987.0, 'CT3': 0.66146E-10,
        'CF1': 0.87843, 'CF2': 3689.7
    },
    'B737': {
        'MLW': 51.710e3, 'S': 124.65, 'CD0_app': 0.0270, 'CD2_app': 0.0441,
        'CD0_clean': 0.0235, 'CD2_clean': 0.0445, 'hp_desc': 30152,
        'CT_desc_high': 0.036336, 'CT_desc_low': 0.053395, 'CT_desc_app': 0.16440,
        'CT1': 145730.0, 'CT2': 55638.0, 'CT3': 0.14200E-10,
        'CF1': 0.94680, 'CF2': 100000.0
    },
    'A320-212': {
        'MLW': 64.500e3, 'S': 122.60, 'CD0_app': 0.0242, 'CD2_app': 0.0469,
        'CD0_clean': 0.0240, 'CD2_clean': 0.0375, 'hp_desc': 12398,
        'CT_desc_high': 0.045711, 'CT_desc_low': 0.027207, 'CT_desc_app': 0.13981,
        'CT1': 136050.0, 'CT2': 52238.0, 'CT3': 0.26637E-10,
        'CF1': 0.94000, 'CF2': 100000.0
    },
    'A319-131': {
        'MLW': 61.000e3, 'S': 122.60, 'CD0_app': 0.0284, 'CD2_app': 0.0376,
        'CD0_clean': 0.0280, 'CD2_clean': 0.31000E-01, 'hp_desc': 27726,
        'CT_desc_high': 0.083084, 'CT_desc_low': 0.051765, 'CT_desc_app': 0.14767,
        'CT1': 139000.0, 'CT2': 58900.0, 'CT3': 0.57200E-14,
        'CF1': 0.68800, 'CF2': 1670.0
    }
}

def density_ISA(h_m):
    """Calcula la densidad según el documento de la ISA (Pág 2 y 5)."""
    if h_m < 11000.0:
        T = T0 - 0.0065 * h_m
        P = p0 * (T / T0) ** 5.2561
    else:
        T = 216.65
        p11 = 22632.0  # Pa
        P = p11 * math.exp(-g0 * (h_m - 11000.0) / (R * T))
    return P / (R * T)

def get_thrust_descent(params, h_m):
    """Calcula el empuje Idle de BADA garantizando las unidades correctas."""
    h_ft = h_m / 0.3048
    # Ecuación empírica BADA original en pies
    T_max = params['CT1'] * (1.0 - (h_ft / params['CT2']) + params['CT3'] * (h_ft ** 2))
    
    if h_ft > params['hp_desc']:
        CT_desc = params['CT_desc_high']
    else:
        if h_ft > 6000.0:
            CT_desc = params['CT_desc_low']
        else:
            CT_desc = params['CT_desc_app']
            
    return CT_desc * T_max

def velocity_min_rate_descent(T_desc, rho, S, CD0, CD2, weight_kg):
    """Ecuación analítica limpia para la velocidad óptima de mínimo descenso."""
    L = weight_kg * g0
    termino_raiz = math.sqrt((T_desc**2 / (9.0 * CD0**2)) + (4.0 * CD2 * L**2 / (3.0 * CD0)))
    z = ((T_desc / (3.0 * CD0)) + termino_raiz) / (rho * S)
    return math.sqrt(z)

def getCDO(aircraft_model, MLW_percent, initial_h_ft=6000.0):
    """Simulación iterativa temporal de la trayectoria CDO hacia atrás."""
    params = AIRCRAFT_PARAMS[aircraft_model]
    
    # Inicialización en el IAF (x=0, h=6000 ft)
    x_current = 0.0                    
    h_m = initial_h_ft * 0.3048        
    weight = params['MLW'] * (MLW_percent / 100.0)  
    t_current = 0.0                    
    
    x_traj = [x_current]
    h_traj = [h_m]                     
    t_traj = [t_current]
    
    while (h_m / 0.3048) < 40000.0:
        h_ft = h_m / 0.3048
        rho = density_ISA(h_m)
        T_desc = get_thrust_descent(params, h_m)
        
        # Transición de flaps a 6000 ft según especificación BADA
        if h_ft >= 6000.0:
            CD0 = params['CD0_clean']
            CD2 = params['CD2_clean']
        else:
            CD0 = params['CD0_app']
            CD2 = params['CD2_app']
            
        v_ms = velocity_min_rate_descent(T_desc, rho, params['S'], CD0, CD2, weight)
        
        # Resistencia aerodinámica
        CL = (2 * weight * g0) / (params['S'] * (v_ms**2) * rho)
        CD = CD0 + CD2 * (CL**2)
        Drag = 0.5 * rho * (v_ms**2) * params['S'] * CD
        
        # Ángulo de planeo cinemático longitudinal
        sin_gamma = (T_desc - Drag) / (weight * g0)
        sin_gamma = max(-0.99, min(0.99, sin_gamma))  # Evitar singularidades verticales
        gamma = math.asin(sin_gamma)  
        
        v_vert = v_ms * math.sin(gamma)  
        v_hor = v_ms * math.cos(gamma)   
        
        # Integración de paso temporal inverso
        h_m -= v_vert * delta_t          
        x_current -= v_hor * delta_t     
        t_current -= delta_t             
        
        # Consumo inverso de masa de combustible (BADA 3.10)
        v_kt = v_ms / 0.514444           
        T_desc_kN = T_desc / 1000.0      
        eta = (params['CF1'] / 60.0) * (1.0 + (v_kt / params['CF2'])) 
        fuel_flow = eta * T_desc_kN                                   
        weight += fuel_flow * delta_t                                 
        
        x_traj.append(x_current)
        h_traj.append(h_m)
        t_traj.append(t_current)
        
    return [x_traj, h_traj, t_traj]

def h_at_waypoints():
    # 1. Ejecución de las simulaciones base (Unidades SI)
    x_767, h_767, _ = getCDO("B767-300ER", 100)
    x_777, h_777, _ = getCDO("B777-300", 100)
    x_737, h_737, _ = getCDO("B737", 100)
    x_a320, h_a320, _ = getCDO("A320-212", 100)
    x_a319, h_a319, _ = getCDO("A319-131", 100)

    x_767_80, h_767_80, _ = getCDO("B767-300ER", 80)
    x_777_80, h_777_80, _ = getCDO("B777-300", 80)
    x_737_80, h_737_80, _ = getCDO("B737", 80)
    x_a320_80, h_a320_80, _ = getCDO("A320-212", 80)
    x_a319_80, h_a319_80, _ = getCDO("A319-131", 80)

    # Nombres de los waypoints secuenciales para la salida del CSV
    wp_names = [
        "ALBER", "CUTXE", "UTHAN", "ENJUC", "UCREQ", "SLL",
        "CASPE", "MECUH", "VIBOK", "BL461", "SLL",
        "LOBAR", "PEKIS", "BL461", "SLL",
        "MARTA", "EBROX", "RES", "VLA", "BL463", "SLL",
        "MATEX", "SENIA", "RES", "VLA", "BL463", "SLL",
        "PUMAL", "BERGA", "KOSIT", "MAMUK", "UCREQ", "SLL"
    ]

    waypoints = [
        129084.40, 92970.40, 51485.60, 32039.60, 19260.80, 0.00,
        164087.20, 88155.20, 50930.00, 18520.00, 0.00,
        151308.40, 83340.00, 18520.00, 0.00,
        178347.60, 138900.00, 90192.40, 50930.00, 18520.00, 0.00,
        190015.20, 137048.00, 90192.40, 50930.00, 18520.00, 0.00,
        94452.00, 72598.40, 46300.00, 35373.20, 19260.80, 0.00
    ]

    # Convertimos las distancias de las trayectorias a valores positivos una sola vez
    pos_x_767 = [-x for x in x_767]
    pos_x_777 = [-x for x in x_777]
    pos_x_737 = [-x for x in x_737]
    pos_x_a320 = [-x for x in x_a320]
    pos_x_a319 = [-x for x in x_a319]

    pos_x_767_80 = [-x for x in x_767_80]
    pos_x_777_80 = [-x for x in x_777_80]
    pos_x_737_80 = [-x for x in x_737_80]
    pos_x_a320_80 = [-x for x in x_a320_80]
    pos_x_a319_80 = [-x for x in x_a319_80]

    with open("waypoints_altitude.csv", "w") as f:
        # Cabecera estructurada del CSV
        f.write("Waypoint,Distance [m],B767-300ER (100%),B777-300 (100%),B737 (100%),A320-212 (100%),A319-131 (100%),B767-300ER (80%),B777-300 (80%),B737 (80%),A320-212 (80%),A319-131 (80%)\n")
        
        for i in range(len(waypoints)):
            target = waypoints[i]
            
            # Algoritmo de optimización por cercanía: encuentra el índice del valor más próximo
            idx1 = min(range(len(pos_x_767)), key=lambda j: abs(pos_x_767[j] - target))
            idx2 = min(range(len(pos_x_777)), key=lambda j: abs(pos_x_777[j] - target))
            idx3 = min(range(len(pos_x_737)), key=lambda j: abs(pos_x_737[j] - target))
            idx4 = min(range(len(pos_x_a320)), key=lambda j: abs(pos_x_a320[j] - target))
            idx5 = min(range(len(pos_x_a319)), key=lambda j: abs(pos_x_a319[j] - target))
            
            idx6 = min(range(len(pos_x_767_80)), key=lambda j: abs(pos_x_767_80[j] - target))
            idx7 = min(range(len(pos_x_777_80)), key=lambda j: abs(pos_x_777_80[j] - target))
            idx8 = min(range(len(pos_x_737_80)), key=lambda j: abs(pos_x_737_80[j] - target))
            idx9 = min(range(len(pos_x_a320_80)), key=lambda j: abs(pos_x_a320_80[j] - target))
            idx10 = min(range(len(pos_x_a319_80)), key=lambda j: abs(pos_x_a319_80[j] - target))
            
            # Escritura limpia incluyendo metadatos de los puntos
            f.write(f"{wp_names[i]},{target:.2f},"
                    f"{h_767[idx1]:.4f},{h_777[idx2]:.4f},{h_737[idx3]:.4f},{h_a320[idx4]:.4f},{h_a319[idx5]:.4f},"
                    f"{h_767_80[idx6]:.4f},{h_777_80[idx7]:.4f},{h_737_80[idx8]:.4f},{h_a320_80[idx9]:.4f},{h_a319_80[idx10]:.4f}\n")

def generate_scenario_1_csv():
    """
    Genera el archivo 'scenario_1.csv' completando todas las columnas de la tabla 
    del Escenario 1 según las asignaciones y distancias de la v1.2 del SoW.
    """
    # 1. Definición de la flota oficial de Barcelona (SoW v1.2) con sus distancias de entrada
    scenarios_data = [
        {'model_label': 'B767', 'full_name': 'B767-300ER', 'star': 'ALBER1Z', 'mlw_pct': 80,  'dist_entry_m': 129084.40},
        {'model_label': 'B737', 'full_name': 'B737',       'star': 'PUMAL1Z', 'mlw_pct': 100, 'dist_entry_m': 94452.00},
        {'model_label': 'B777', 'full_name': 'B777-300',   'star': 'MARTA3Z', 'mlw_pct': 100, 'dist_entry_m': 178347.60},
        {'model_label': 'B767', 'full_name': 'B767-300ER', 'star': 'MATEX3Z', 'mlw_pct': 80,  'dist_entry_m': 190015.20},
        {'model_label': 'A319', 'full_name': 'A319-131',   'star': 'LOBAR2W', 'mlw_pct': 80,  'dist_entry_m': 151308.40},
        {'model_label': 'A320', 'full_name': 'A320-212',   'star': 'CASPE2W', 'mlw_pct': 100, 'dist_entry_m': 164087.20}
    ]
    
    # Hora de entrada unificada en crucero
    start_time = datetime.strptime("11:45:00", "%H:%M:%S")
    
    # 2. Extracción de datos del simulador
    for ac in scenarios_data:
        x_traj, h_traj, t_traj = getCDO(ac['full_name'], ac['mlw_pct'], initial_h_ft=6000.0)
        pos_x = [-x for x in x_traj]  # Convertimos distancias a valores positivos
        
        # Encontramos el índice más cercano al punto de entrada (STAR entry WP)
        idx = min(range(len(pos_x)), key=lambda j: abs(pos_x[j] - ac['dist_entry_m']))
        
        # Almacenamos los resultados físicos calculados
        ac['alt_entry_m'] = h_traj[idx]
        ac['time_to_iaf_s'] = abs(t_traj[idx])
        ac['toa_iaf'] = start_time + timedelta(seconds=ac['time_to_iaf_s'])

    # 3. SECUENCIACIÓN: Ordenamos los aviones según quién llega ANTES al IAF (ToA IAF)
    sorted_fleet = sorted(scenarios_data, key=lambda x: x['toa_iaf'])
    
    # 4. Escritura del archivo CSV definitivo
    with open("scenario_1.csv", "w") as f:
        # Cabecera idéntica a la diapositiva de clase
        f.write("Aircraft model,STAR,%MLW IAF,Dist. STAR entry WP to IAF [km],Alt. at STAR entry WP [m],Time from STAR entry WP to IAF [s],ToA IAF,Seq. Order,Sep. to previous at IAF [s]\n")
        
        for i, ac in enumerate(sorted_fleet):
            # Calcular la separación en segundos respecto al avión anterior
            if i == 0:
                separation = 0  # El primer avión no tiene precedente
            else:
                separation = (ac['toa_iaf'] - sorted_fleet[i-1]['toa_iaf']).total_seconds()
                
            f.write(f"{ac['model_label']},"
                    f"{ac['star']},"
                    f"{ac['mlw_pct']},"
                    f"{ac['dist_entry_m'] / 1000.0:.2f},"    # Pasamos metros a km
                    f"{ac['alt_entry_m']:.4f},"
                    f"{ac['time_to_iaf_s']:.1f},"
                    f"{ac['toa_iaf'].strftime('%H:%M:%S')},"
                    f"{i + 1},"                              # Orden secuencial (1º, 2º...)
                    f"{separation:.0f}\n")
                    
    print("¡Archivo 'scenario_1.csv' generado correctamente con todos los datos calculados!")

def generate_scenario_2_csv():
    """
    Genera el archivo 'scenario_2.csv' resolviendo la secuenciación ATC estricta
    con separación fija EXACTA de 2 minutos (120s) entre cada avión en el IAF.
    """
    fleet_data = [
        {'model_label': 'B767', 'full_name': 'B767-300ER', 'star': 'ALBER1Z', 'mlw_pct': 80,  'dist_entry_m': 129084.40},
        {'model_label': 'B737', 'full_name': 'B737',       'star': 'PUMAL1Z', 'mlw_pct': 100, 'dist_entry_m': 94452.00},
        {'model_label': 'B777', 'full_name': 'B777-300',   'star': 'MARTA3Z', 'mlw_pct': 100, 'dist_entry_m': 178347.60},
        {'model_label': 'B767', 'full_name': 'B767-300ER', 'star': 'MATEX3Z', 'mlw_pct': 80,  'dist_entry_m': 190015.20},
        {'model_label': 'A319', 'full_name': 'A319-131',   'star': 'LOBAR2W', 'mlw_pct': 80,  'dist_entry_m': 151308.40},
        {'model_label': 'A320', 'full_name': 'A320-212',   'star': 'CASPE2W', 'mlw_pct': 100, 'dist_entry_m': 164087.20}
    ]
    
    start_time = datetime.strptime("11:45:00", "%H:%M:%S")
    
    for ac in fleet_data:
        x_traj, _, t_traj = getCDO(ac['full_name'], ac['mlw_pct'], initial_h_ft=6000.0)
        pos_x = [-x for x in x_traj]
        idx = min(range(len(pos_x)), key=lambda j: abs(pos_x[j] - ac['dist_entry_m']))
        
        ac['time_to_iaf_s'] = abs(t_traj[idx])
        ac['toa_iaf_scen1'] = start_time + timedelta(seconds=ac['time_to_iaf_s'])

    # Ordenamos la flota siguiendo la secuencia descubierta en el Escenario 1
    sorted_fleet = sorted(fleet_data, key=lambda x: x['toa_iaf_scen1'])
    
    # ALGORITMO DE ESPACIADO FIJO EXACTO (Cadena secuencial rígida)
    with open("scenario_2.csv", "w") as f:
        f.write("Aircraft model,STAR,Weight at IAF,ToA IAF Scenario 2,Applied delay [s],ToA STAR entry WP Scenario 2\n")
        
        # El primer avión de la secuencia marca el origen inalterable (Retraso = 0)
        leader_iaf_time = sorted_fleet[0]['toa_iaf_scen1']
        
        for i, ac in enumerate(sorted_fleet):
            # Cada avión se posiciona en slots de exactamente i * 120 segundos
            ac['toa_iaf_scen2'] = leader_iaf_time + timedelta(seconds=i * 120)
            ac['applied_delay_s'] = (ac['toa_iaf_scen2'] - ac['toa_iaf_scen1']).total_seconds()
            
            # Hora real a la que debe cruzar el punto inicial en crucero
            ac['toa_star_entry_scen2'] = start_time + timedelta(seconds=ac['applied_delay_s'])
            
            f.write(f"{ac['model_label']},"
                    f"{ac['star']},"
                    f"{ac['mlw_pct']}% MLW,"
                    f"{ac['toa_iaf_scen2'].strftime('%H:%M:%S')},"
                    f"{ac['applied_delay_s']:.0f},"
                    f"{ac['toa_star_entry_scen2'].strftime('%H:%M:%S')}\n")
                    
    print("¡Archivo 'scenario_2.csv' corregido de forma estricta con espaciados fijos de 2 minutos exactos!")
