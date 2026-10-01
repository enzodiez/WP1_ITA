import matplotlib.pyplot as plt
import numpy as np

# Base de datos de parámetros de los aviones según el EUROCONTROL BADA
# Los valores extraídos corresponden a las columnas del B767-300ER y A320-212.
AIRCRAFT_PARAMS = {
    'B767-300ER': {
        'MLW': 145150.0,  # kg (0.145150E+03 tons * 1000)
        'S': 283.50,  # m^2 (0.28350E+03)
        'CD0_clean': 0.017400,  # (0.17400E-01)
        'CD2_clean': 0.045900,  # (0.45900E-01)
        'CD0_app': 0.014000,  # (0.14000E-01)
        'CD2_app': 0.049000,  # (0.49000E-01)
        'hp_desc': 26418,  # ft
        'CT_desc_high': 0.064359,  # (0.64359E-1)
        'CT_desc_low': 0.055988,  # (0.55988E-1)
        'CT_desc_app': 0.12475,  # (0.12475)
        'CT1': 351670.0,  # N (0.35167E+06)
        'CT2': 44673.0,  # ft (0.44673E+05)
        'CT3': 1.0129e-10,  # 1/ft^2 (0.10129E-09)
        'CF1': 0.54005,  # kg/(min*kN) (0.54005E+00)
        'CF2': 557.82  # kt (0.55782E+03)
    },
    'A320-212': {
        'MLW': 64500.0,  # kg (0.64500E+02 tons * 1000)
        'S': 122.60,  # m^2 (0.12260E+03)
        'CD0_clean': 0.024000,  # (0.24000E-01)
        'CD2_clean': 0.037500,  # (0.37500E-01)
        'CD0_app': 0.024200,  # (0.24200E-01)
        'CD2_app': 0.046900,  # (0.46900E-01)
        'hp_desc': 12398,  # ft
        'CT_desc_high': 0.045711,  # (0.45711E-1)
        'CT_desc_low': 0.027207,  # (0.27207E-1)
        'CT_desc_app': 0.13981,  # (0.13981)
        'CT1': 136050.0,  # N (0.13605E+06)
        'CT2': 52238.0,  # ft (0.52238E+05)
        'CT3': 2.6637e-10,  # 1/ft^2 (0.26637E-10)
        'CF1': 0.94000,  # kg/(min*kN) (0.94000E+00)
        'CF2': 100000.0  # kt (0.10000E+06)
    }
}


def getCDO(aircraft_model, MLW_percent):
    """
    Simula la trayectoria de descenso (hacia atrás) para un modelo de avión.
    """
    params = AIRCRAFT_PARAMS[aircraft_model]

    # Condiciones iniciales de simulación hacia atrás (IAF)
    x_current = 0.0  # Distancia inicial en metros
    h_current = 6000.0  # Altitud inicial en ft
    weight = params['MLW'] * (MLW_percent / 100.0)
    delta_t = 1.0  # Iteración de 1 segundo

    x_traj = [x_current]
    h_traj = [h_current]

    # Bucle iterativo hasta alcanzar FL400 (40,000 ft)
    while h_current < 40000.0:
        # Se asume la velocidad que minimiza la tasa de descenso (V_md)
        # NOTA: En un modelo real, v debe calcularse usando la atmósfera estándar ISA.
        # Para la simulación estructurada, asignaremos una velocidad constante aproximada.
        v_kt = 250.0
        v_ms = v_kt * 0.514444

        # 1. Calcular Empuje Máximo (T_max) y de Descenso (T_desc)
        T_max = params['CT1'] * (1 - (h_current / params['CT2']) + params['CT3'] * (h_current ** 2))

        # Selección del coeficiente de empuje según altitud y fase de descenso
        if h_current < params['hp_desc']:
            CT_desc = params['CT_desc_low']
        else:
            CT_desc = params['CT_desc_high']

        T_desc = CT_desc * T_max  # El empuje se asume siempre en "idle" para el descenso

        # 2. Coeficientes aerodinámicos (CD0 y CD2 cambian según la altitud por los flaps)
        if h_current < 10000.0:  # Aproximación de configuración approach
            CD0 = params['CD0_app']
            CD2 = params['CD2_app']
        else:
            CD0 = params['CD0_clean']
            CD2 = params['CD2_clean']

        # CL asumido para vuelo nivelado/descenso ligero (Sustentación = Peso)
        # L = 0.5 * rho * v^2 * S * CL => CL = (Weight * g) / (0.5 * rho * v^2 * S)
        # Para mantener el código autónomo sin el modelo completo de densidad ISA:
        CL = 0.5
        CD = CD0 + CD2 * (CL ** 2)

        # 3. Consumo de Combustible (Fuel Flow)
        eta = params['CF1'] * (1 + (v_kt / params['CF2']))
        FF = eta * (T_desc / 1000.0)  # T_desc en kN para la fórmula


        gamma = -0.052

        dh = v_ms * np.sin(gamma) * delta_t
        dx = v_ms * np.cos(gamma) * delta_t

        h_current -= (dh * 3.28084)  # Convertir m a ft
        x_current -= dx
        weight += (FF / 60.0) * delta_t  # Sumamos peso hacia atrás ya que se quemó combustible

        x_traj.append(x_current)
        h_traj.append(h_current)

    return x_traj, h_traj


# Script Principal (Main)
if __name__ == "__main__":
    plt.figure(figsize=(12, 6))

    # Generar todas las trayectorias iterando combinaciones sin solicitar inputs al usuario
    scenarios = [
        ('B767-300ER', 100),
        ('B767-300ER', 80),
        ('A320-212', 100),
        ('A320-212', 80)
    ]

    for model, mlw in scenarios:
        x, h = getCDO(model, mlw)
        # Convertir ft a metros para igualar la gráfica original
        h_meters = [alt / 3.28084 for alt in h]
        plt.plot(x, h_meters, label=f'{model} [{mlw}% MLW]')

    plt.title('Simulador de Operación de Descenso Continuo (CDO)')
    plt.xlabel('x [m]')
    plt.ylabel('h [m]')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.xlim(min(x), 0)
    plt.ylim(0, 12500)

    # Mostrar todas las trayectorias calculadas con un solo "clic"
    plt.show()