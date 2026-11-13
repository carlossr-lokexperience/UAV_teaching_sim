"""CONFIGURACION DEL TALLER. Empieza cambiando SOLO las variables de este bloque.
Unidades internas: metros, segundos y radianes; los parametros *_DEG usan grados.
No hay PX4, ROS, Gazebo, servicios web obligatorios ni comandos a un UAV real.
"""
from dataclasses import dataclass

# ============= CAMBIA ESTO EN CLASE =============
FLIGHT_MODE = "FOLLOW_ROAD"      # FOLLOW_ROAD / HEADING / MANUAL
GUIDANCE_METHOD = "PURE_PURSUIT" # PURE_PURSUIT / VECTOR_FIELD
ROAD_SOURCE = "MAP"             # MAP / CAMERA (CAMERA nunca consulta el mapa)
NAVIGATION_METHOD = "PERFECT"    # PERFECT / NOISY / FILTERED (alpha-beta, NO EKF)
CONTROL_METHOD = "P"            # P / PID (lazos educativos, no aerodinamica 6-DOF)
COMMAND_PARSER = "RULES"         # RULES / LLM; RULES NO es un modelo de lenguaje

TARGET_ALTITUDE_M = 100.0
TARGET_SPEED_MPS = 20.0
LOOKAHEAD_M = 65.0               # Solo Pure Pursuit: anticipacion sobre la curva
VECTOR_CONVERGENCE = 0.025       # 1/m; solo Vector Field
MAX_BANK_DEG = 35.0             # Limite operativo; se puede cambiar en vivo
WIND_EAST_MPS = 0.0
WIND_NORTH_MPS = 0.0
GPS_NOISE_M = 2.5
GNSS_AVAILABLE = True

# ============= LIMITES DEL MODELO =============
MIN_SPEED_MPS = 12.0            # Envolvente artificial; NO simula una perdida real
MAX_SPEED_MPS = 32.0
PHYSICAL_BANK_LIMIT_DEG = 60.0
MAX_ROLL_RATE_DEG_S = 45.0
MAX_ACCEL_MPS2 = 2.5
MAX_CLIMB_MPS = 4.0
MIN_ALTITUDE_M = 30.0
MAX_ALTITUDE_M = 220.0
ROLL_ACTUATOR_TAU_S = 0.18
CLIMB_TAU_S = 0.8

# ============= CONTROL: PRUEBA UNA GANANCIA A LA VEZ =============
ROLL_KP = 3.0
ROLL_KI = 0.25
ROLL_KD = 0.08
ALTITUDE_KP = 0.50
ALTITUDE_KI = 0.025
SPEED_KP = 0.70
SPEED_KI = 0.04
HEADING_KP = 1.0

# ============= CAMARA Y OPENCV =============
CAMERA_SIZE = 320               # Camara cuadrada y estabilizada: arriba = delante
CAMERA_HFOV_DEG = 100.0         # En terreno plano: ancho = 2*h*tan(FOV/2)
CAMERA_HZ = 10.0
PERCEPTION_ENABLED = True       # False permite ejecutar MAP sin OpenCV
ROAD_HSV_MIN = (0, 0, 35)       # Asfalto gris. H en [0,179]; S,V en [0,255]
ROAD_HSV_MAX = (179, 70, 110)
ROAD_MIN_PIXELS = 140
ROAD_LOST_TIMEOUT_S = 0.7       # Despues: HEADING, alas hacia nivel, sin mapa oculto
NAV_LOST_TIMEOUT_S = 5.0
GPS_HZ = 5.0
FILTER_ALPHA = 0.45
FILTER_BETA = 0.06

# ============= SIMULACION Y RENDER =============
PHYSICS_DT = 0.02               # 50 Hz; independiente del ritmo grafico
GUI_FPS = 25
CHASE_WIDTH = 800
CHASE_HEIGHT = 500
ROAD_WIDTH_M = 18.0
ROAD_SAMPLES = 600
SCENE_RANGE_M = 2000.0
RANDOM_SEED = 7
LOG_HZ = 5.0
SHOW_DEBUG = True
START_ROAD_FRACTION = 0.0

# ============= LLM OPCIONAL: SOLO TRADUCE TEXTO =============
# No se descarga ni instala un modelo. Configura un servicio YA disponible.
# OLLAMA usa /api/chat; OPENAI_COMPATIBLE usa /chat/completions.
# Las llamadas son asincronas: la simulacion NO espera al LLM.
LLM_PROVIDER = "OLLAMA"        # OLLAMA / OPENAI_COMPATIBLE
LLM_BASE_URL = "http://127.0.0.1:11434"
LLM_MODEL = ""                 # Nombre EXACTO de tu modelo; vacio da error explicito
LLM_API_KEY_ENV = "UAV_LLM_API_KEY"  # Nunca escribas claves en el repositorio
LLM_TIMEOUT_S = 15.0
LLM_MAX_RESPONSE_BYTES = 65536

@dataclass
class Settings:
    """Valores vivos. config.py se lee al arrancar; el panel modifica esta copia."""
    mode: str = FLIGHT_MODE
    guidance: str = GUIDANCE_METHOD
    source: str = ROAD_SOURCE
    navigation: str = NAVIGATION_METHOD
    controller: str = CONTROL_METHOD
    parser: str = COMMAND_PARSER
    altitude_m: float = TARGET_ALTITUDE_M
    speed_mps: float = TARGET_SPEED_MPS
    lookahead_m: float = LOOKAHEAD_M
    bank_limit_deg: float = MAX_BANK_DEG
    vector_gain: float = VECTOR_CONVERGENCE
    heading_rad: float = 0.0
    manual_bank_rad: float = 0.0
    wind_e: float = WIND_EAST_MPS
    wind_n: float = WIND_NORTH_MPS
    gnss: bool = GNSS_AVAILABLE

    def validate(self):
        import math
        choices = {"mode": ("FOLLOW_ROAD", "HEADING", "MANUAL"),
                   "guidance": ("PURE_PURSUIT", "VECTOR_FIELD"),
                   "source": ("MAP", "CAMERA"),
                   "navigation": ("PERFECT", "NOISY", "FILTERED"),
                   "controller": ("P", "PID"), "parser": ("RULES", "LLM")}
        for name, options in choices.items():
            if getattr(self, name) not in options:
                raise ValueError("%s debe ser uno de %s" % (name, options))
        bounds = {"altitude_m": (MIN_ALTITUDE_M, MAX_ALTITUDE_M),
                  "speed_mps": (MIN_SPEED_MPS, MAX_SPEED_MPS),
                  "lookahead_m": (15.0, 150.0), "bank_limit_deg": (5.0, PHYSICAL_BANK_LIMIT_DEG),
                  "vector_gain": (0.002, 0.1), "wind_e": (-12.0, 12.0), "wind_n": (-12.0, 12.0)}
        for name, (lo, hi) in bounds.items():
            value = getattr(self, name)
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError("%s fuera del intervalo [%s, %s]" % (name, lo, hi))
        return self
