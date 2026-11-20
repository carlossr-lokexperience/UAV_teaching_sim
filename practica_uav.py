"""
PRACTICA UAV - NIVEL INTERMEDIO
===============================


COMO USARLO:
  1. Copia practica_uav.py dentro de la carpeta UAV_CLASS,
     al lado de main.py y config.py.
  2. Abre la terminal en esa carpeta y activa el entorno virtual.
  3. Ejecuta: python practica_uav.py
  4. Cambia los valores de la SECCION 1 y vuelve a ejecutar.

Objetivo: reconocer variables, funciones, condicionales, listas,
diccionarios, imports, objetos y el bucle de una simulacion.
"""

# IMPORTS: reutilizamos clases que YA existen en nuestro proyecto.
from config import Settings
from main import Application


# ============================================================
# SECCION 1: VARIABLES - CAMBIA ESTO EN CLASE
# ============================================================
ALTURA_INICIAL = 100.0         # Metros. Entre 30 y 220.
VELOCIDAD_INICIAL = 20.0       # m/s. Entre 12 y 32.
GUIADO = "PURE_PURSUIT"       # "PURE_PURSUIT" o "VECTOR_FIELD"
FUENTE = "MAP"               # "MAP" o "CAMERA"
ANTICIPACION = 65.0           # Metros. Afecta a PURE_PURSUIT.
ALABEO_MAXIMO = 35.0          # Grados. Entre 5 y 60.

# False: vuelo normal, controlado desde la ventana.
# True: ademas se ejecutan las ordenes de la SECCION 2.
ACTIVAR_MISION = True


# ============================================================
# SECCION 2: LISTA DE ORDENES
# ============================================================
# Cada elemento tiene DOS partes:
#   (segundo_de_vuelo, {orden_del_avion})
# Las llaves { } forman un diccionario: nombre -> valor.
# Los tiempos estan en segundos SIMULADOS, no de reloj.
MISION = [
    (5.0,  {"altitude_m": 140.0}),
    (12.0, {"speed_mps": 18.0}),
    (19.0, {"heading_delta_deg": 30.0}),  # Girar 30 grados a la derecha
    (28.0, {"mode": "FOLLOW_ROAD"}),      # Volver a seguir la carretera
    (36.0, {"guidance": "VECTOR_FIELD"}),
]


# ============================================================
# SECCION 3: FUNCIONES
# ============================================================
def preparar_avion():
    """Crea la configuracion inicial a partir de nuestras variables."""
    ajustes = Settings()                      # Creamos un OBJETO Settings.
    ajustes.altitude_m = ALTURA_INICIAL       # Modificamos un ATRIBUTO.
    ajustes.speed_mps = VELOCIDAD_INICIAL
    ajustes.guidance = GUIADO
    ajustes.source = FUENTE
    ajustes.lookahead_m = ANTICIPACION
    ajustes.bank_limit_deg = ALABEO_MAXIMO
    ajustes.mode = "FOLLOW_ROAD"
    ajustes.parser = "RULES"                 # No requiere API ni clave de IA.
    return ajustes.validate()                 # RETURN devuelve el resultado.


def comprobar_mision(app, pendientes):
    """Comprueba si toca ejecutar alguna orden de la mision."""
    tiempo = app.sim.aircraft.state.time       # Tiempo de vuelo simulado.

    # WHILE: puede ejecutar varias ordenes si el tiempo ha avanzado mucho.
    while pendientes and tiempo >= pendientes[0][0]:
        segundo, orden = pendientes.pop(0)     # Cogemos la siguiente orden.

        try:
            app.sim.apply_command(orden)      # Se valida y aplica al avion.
            app.sync_widgets()               # Actualiza el panel de la ventana.
            mensaje = f"Segundo {segundo:.0f}: aplicada {orden}"
            print(mensaje)                   # Muestra el mensaje en la terminal.
            app.status.config(text=mensaje)
        except ValueError as error:
            # Si pedimos, por ejemplo, 500 m de altura, se rechaza.
            print(f"Orden rechazada: {error}")
            app.status.config(text=f"Orden rechazada: {error}")

    # IF: si la ventana sigue abierta y quedan ordenes, volvemos a comprobar.
    if not app.closed and pendientes:
        app.root.after(200, comprobar_mision, app, pendientes)


def main():
    """Punto de entrada de esta practica."""
    ajustes = preparar_avion()                 # Llamada a una funcion.
    app = Application(ajustes)                 # Creamos la ventana y el avion.

    if ACTIVAR_MISION:                         # IF: decision segun un booleano.
        pendientes = list(MISION)              # Copia de la lista de ordenes.
        app.root.after(200, comprobar_mision, app, pendientes)

    print("Iniciando UAV_CLASS. Cierra la ventana para terminar.")
    app.run()                                  # Bucle grafico y de simulacion.


# Este IF permite ejecutar el archivo, pero no ejecutar main() al importarlo.
if __name__ == "__main__":
    main()


# ============================================================
# RETOS PARA LOS ALUMNOS
# ============================================================
# 1. Empieza a 120 metros y 22 m/s. Que observas?
# 2. Cambia PURE_PURSUIT por VECTOR_FIELD. Cambia el seguimiento?
# 3. Anade una orden para bajar a 90 metros a los 45 segundos.
# 4. Prueba un giro a la izquierda (angulo negativo).
# 5. Pon ACTIVAR_MISION = False. Quien controla ahora el avion?
# 6. Cambia una orden a 500 metros. Por que se rechaza?
#
# AVISO: es un simulador educativo, no codigo para controlar un UAV fisico.
