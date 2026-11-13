"""AUTOPILOTO: coordinador pequeño; no dibuja, no llama a un LLM y no lee sensores.
Recibe SOLO estado estimado y una fuente de carretera. Las funciones de guiado
son intercambiables. Sin orbita. Ante perdida de vision/navegacion: HEADING.
HEADING es una contingencia docente, NO una estrategia de seguridad real.
"""
import config as C
from guidance import METHODS, GuidanceCommand, heading_guidance

class Autopilot:
    def __init__(self, settings):
        self.settings = settings
        self.last_path = None
        self.last_path_time = -100.0
        self.source = settings.source
        self.status = "INICIO"
        self.reference = None

    def reset_tracking(self):
        self.last_path = None
        self.last_path_time = -100.0
        self.source = self.settings.source

    def set_mode(self, mode, state):
        if mode not in ("FOLLOW_ROAD", "HEADING", "MANUAL"):
            raise ValueError("Modo desconocido")
        self.settings.mode = mode
        if mode == "HEADING":
            self.settings.heading_rad = state.heading
        if mode == "MANUAL":
            self.settings.manual_bank_rad = 0.0
        self.reset_tracking()

    def _fallback(self, state, reason):
        self.settings.mode = "HEADING"
        self.settings.heading_rad = state.heading
        self.status = reason + " -> HEADING. Pulsa F para reactivar."
        return heading_guidance(state, self.settings, self.status)

    def update(self, state, map_path=None, observation=None):
        s = self.settings
        if s.source != self.source:
            self.reset_tracking()
        if s.mode == "MANUAL":
            self.reference = GuidanceCommand(s.manual_bank_rad, s.altitude_m, s.speed_mps, status="MANUAL ASISTIDO")
        elif s.mode == "HEADING":
            self.reference = heading_guidance(state, s, self.status if "->" in self.status else "HEADING")
        elif not state.valid:
            self.reference = self._fallback(state, "POSICION OBSOLETA")
        else:
            if s.source == "MAP":
                path = map_path
            else:
                # PROHIBIDO usar map_path en CAMERA: una sola interfaz de entrada.
                if observation is not None and observation.valid and 0 <= state.time-observation.time <= C.ROAD_LOST_TIMEOUT_S:
                    self.last_path = observation.points
                    self.last_path_time = observation.time
                path = self.last_path
                if path is None or state.time-self.last_path_time > C.ROAD_LOST_TIMEOUT_S:
                    self.reference = self._fallback(state, "CAMARA SIN CARRETERA")
                    return self.reference
            if path is None:
                self.reference = self._fallback(state, "SIN CAMINO")
            else:
                self.reference = METHODS[s.guidance](state, path, s)
                self.reference.status = s.guidance + " / " + s.source
                if s.source == "CAMERA" and s.guidance == "PURE_PURSUIT":
                    import numpy as np
                    if np.linalg.norm(self.reference.target-path[-1]) < 1e-6:
                        self.reference.status += " | objetivo limitado por FOV"
                self.status = self.reference.status
        return self.reference
