# API utilizada: docs/REFERENCIAS.md (OpenCV HSV/inRange).
"""PERCEPCION: imagen RGB de la camara -> carretera local estimada.
No importa world ni recibe el mapa. Solo imagen, pose estimada en su instante,
altura y geometria de camara. Terreno plano z=0, camara nadir estabilizada.
La navegacion sigue aportando pose/altura: esto NO es navegacion sin GNSS.
"""
from dataclasses import dataclass
import math
import numpy as np
import config as C
try:
    import cv2
except ImportError:
    cv2 = None

@dataclass
class PathObservation:
    points: object = None
    confidence: float = 0.0
    time: float = -1.0
    mask: object = None
    overlay: object = None
    reason: str = "SIN IMAGEN"

    @property
    def valid(self):
        return self.points is not None and len(self.points) >= 4 and self.confidence >= 0.25


def image_to_ground(pixels, pose, width, height, fov_deg=C.CAMERA_HFOV_DEG):
    """Pinhole. Imagen: u=derecha, v=abajo; ENU: x=este, y=norte.
    El centro optico y FOV son los mismos que en world.Camera.nadir().
    La pose pertenece al INSTANTE DE CAPTURA, no al ultimo paso de fisica.
    """
    pixels = np.asarray(pixels, dtype=float)
    focal = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    right_m = (pixels[:, 0] - width/2.0) * pose.altitude / focal
    ahead_m = (height/2.0 - pixels[:, 1]) * pose.altitude / focal
    right = np.array([math.cos(pose.heading), -math.sin(pose.heading)])
    forward = np.array([math.sin(pose.heading), math.cos(pose.heading)])
    return np.array([pose.x, pose.y]) + right_m[:, None]*right + ahead_m[:, None]*forward


def detect_road(rgb, pose):
    if cv2 is None:
        return PathObservation(time=pose.time, reason="OpenCV no instalado")
    if rgb is None or pose.altitude <= 0 or not pose.valid:
        return PathObservation(time=pose.time, reason="Imagen/pose no valida")
    # CAMBIA AQUI: color, kernel y seleccion de la linea central.
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array(C.ROAD_HSV_MIN, dtype=np.uint8), np.array(C.ROAD_HSV_MAX, dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return PathObservation(time=pose.time, mask=mask, overlay=rgb.copy(), reason="Sin asfalto")
    sizes = stats[1:, cv2.CC_STAT_AREA]
    largest = 1 + int(np.argmax(sizes))
    mask = ((labels == largest).astype(np.uint8))*255
    if sizes[largest-1] < C.ROAD_MIN_PIXELS:
        return PathObservation(time=pose.time, mask=mask, overlay=rgb.copy(), reason="Area insuficiente")
    height, width = mask.shape
    points = []
    last_u = width/2
    rows = list(range(height-8, 7, -8))  # de DETRAS hacia DELANTE
    for row in rows:
        xs = np.flatnonzero(mask[row])
        if len(xs) < 3:
            continue
        runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1)+1)
        runs = [r for r in runs if len(r) >= 3]
        if not runs:
            continue
        selected = min(runs, key=lambda r: abs(float(np.mean(r))-last_u))
        last_u = float(np.mean(selected))
        points.append((last_u, float(row)))
    overlay = rgb.copy()
    overlay[mask > 0] = (0.45*overlay[mask > 0] + 0.55*np.array([30, 220, 180])).astype(np.uint8)
    confidence = len(points)/max(1, len(rows))
    if len(points) < 6:
        return PathObservation(time=pose.time, mask=mask, overlay=overlay, reason="No hay linea suficiente")
    pixels = np.array(points)
    # Regularizacion geometrica: las filas discretas producen tangentes ruidosas.
    # Un polinomio cubico local suaviza la linea; NO utiliza el mapa del mundo.
    normalized_v = (pixels[:,1]-height/2)/(height/2)
    coeff = np.polyfit(normalized_v, pixels[:,0], min(3,len(points)-1))
    pixels[:,0] = np.clip(np.polyval(coeff,normalized_v),0,width-1)
    path = image_to_ground(pixels, pose, width, height)
    # No inventar una carretera fuera del FOV. Hay que VER un tramo por delante.
    forward = np.array([math.sin(pose.heading), math.cos(pose.heading)])
    ahead = (path - np.array([pose.x, pose.y])) @ forward
    if max(ahead) < 20.0:
        return PathObservation(time=pose.time, mask=mask, overlay=overlay, reason="Carretera fuera del frente")
    for u, v in pixels:
        cv2.circle(overlay, (int(u), int(v)), 2, (255, 190, 30), -1)
    cv2.polylines(overlay, [pixels.astype(np.int32)], False, (255, 190, 30), 2)
    return PathObservation(path, confidence, pose.time, mask, overlay, "OK")
