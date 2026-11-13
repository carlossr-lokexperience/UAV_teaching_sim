# Razonamiento y fuentes: docs/REFERENCIAS.md (Coulter; Nelson et al.).
"""GUIADO: estado + camino -> referencias. Cambia aqui los algoritmos.
No importa OpenCV, world ni el LLM. Ambos metodos reciben la misma polilinea.
Pure Pursuit: a_lat = 2*Vg^2*error_derecha / distancia_objetivo^2.
Es una adaptacion educativa al giro coordinado, no NPFG ni PX4.
"""
from dataclasses import dataclass
import math
import numpy as np
from aircraft import G, wrap_pi
import config as C

@dataclass
class GuidanceCommand:
    bank_rad: float
    altitude_m: float
    speed_mps: float
    target: object = None
    cross_track_m: float = float("nan")
    status: str = "OK"


def project_on_path(position, path):
    """Proyeccion sobre segmentos, no solo sobre el waypoint mas cercano."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.all(np.isfinite(path)):
        raise ValueError("El camino necesita >=2 puntos XY finitos")
    d = path[1:] - path[:-1]
    lengths2 = np.sum(d*d, axis=1)
    t = np.clip(np.sum((np.asarray(position)-path[:-1])*d, axis=1) / np.maximum(lengths2, 1e-12), 0, 1)
    projections = path[:-1] + t[:, None]*d
    distances2 = np.sum((projections - position)**2, axis=1)
    distances2[lengths2 < 1e-10] = np.inf
    if not np.any(np.isfinite(distances2)):
        raise ValueError("El camino solo tiene puntos repetidos")
    i = int(np.argmin(distances2))
    tangent = d[i] / math.sqrt(lengths2[i])
    right = np.array([tangent[1], -tangent[0]])
    error = float(np.dot(np.asarray(position)-projections[i], right))
    return i, float(t[i]), projections[i], tangent, error


def point_ahead(path, index, fraction, distance):
    """Avanza distancia SOBRE el camino; en caminos abiertos termina en el extremo."""
    path = np.asarray(path, dtype=float)
    closed = bool(np.linalg.norm(path[0]-path[-1]) < 1e-6)
    current = path[index] + fraction*(path[index+1]-path[index])
    i = index
    for _ in range(len(path) + 1):
        endpoint = path[i+1]
        segment_length = float(np.linalg.norm(endpoint-current))
        if segment_length >= distance and segment_length > 1e-9:
            return current + distance*(endpoint-current)/segment_length
        distance -= segment_length
        current = endpoint
        i += 1
        if i >= len(path)-1:
            if not closed:
                return path[-1].copy()
            i = 0
            current = path[0]
    return current.copy()


def pure_pursuit(state, path, settings):
    i, t, closest, tangent, error = project_on_path((state.x, state.y), path)
    target = point_ahead(path, i, t, settings.lookahead_m)
    delta = target - np.array([state.x, state.y])
    right = np.array([math.cos(state.course), -math.sin(state.course)])
    distance2 = max(25.0, float(np.dot(delta, delta)))
    lateral_accel = 2 * state.ground_speed**2 * float(np.dot(delta, right)) / distance2
    bank = math.atan2(lateral_accel, G)
    return GuidanceCommand(bank, settings.altitude_m, settings.speed_mps, target, error)


def vector_field(state, path, settings):
    i, t, closest, tangent, error = project_on_path((state.x, state.y), path)
    right = np.array([tangent[1], -tangent[0]])
    direction = tangent - settings.vector_gain * error * right
    desired_course = math.atan2(direction[0], direction[1])
    # Curvatura local (feedforward), mas correccion de rumbo por realimentacion.
    path = np.asarray(path)
    j = min(i+1, len(path)-2)
    next_d = path[j+1]-path[j]
    length = max(1.0, float(np.linalg.norm(path[i+1]-path[i])))
    curvature = wrap_pi(math.atan2(next_d[0], next_d[1])-math.atan2(tangent[0], tangent[1]))/length
    curvature = float(np.clip(curvature, -0.05, 0.05))
    accel = state.ground_speed**2*curvature + state.ground_speed*C.HEADING_KP*wrap_pi(desired_course-state.course)
    target = closest + 50*direction/max(np.linalg.norm(direction), 1e-9)
    return GuidanceCommand(math.atan2(accel, G), settings.altitude_m, settings.speed_mps, target, error)


def heading_guidance(state, settings, status="HEADING"):
    error = wrap_pi(settings.heading_rad - state.heading)
    accel = state.airspeed * C.HEADING_KP * error
    return GuidanceCommand(math.atan2(accel, G), settings.altitude_m, settings.speed_mps, status=status)

# ANADE aqui otra funcion con la MISMA firma, y registrala en este diccionario.
METHODS = {"PURE_PURSUIT": pure_pursuit, "VECTOR_FIELD": vector_field}
