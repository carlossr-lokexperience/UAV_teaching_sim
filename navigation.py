"""NAVEGACION: produce estado estimado, nunca elige hacia donde volar.
PERFECT copia la verdad. NOISY simula GNSS lento. FILTERED aplica alpha-beta.
La perdida de GNSS usa dead reckoning sintetico con sesgo; NO es un INS/EKF real.
El simulador entrega la verdad aqui como generador de sensores, no al guiado.
"""
from dataclasses import replace
import math
import numpy as np
import config as C
from aircraft import State, wrap_pi

class Navigation:
    def __init__(self, seed=C.RANDOM_SEED):
        self.rng = np.random.default_rng(seed)
        self.estimate = None
        self.last_fix = -1.0
        self.last_update = -1.0
        self.was_method = None

    def reset(self):
        self.estimate = None
        self.last_fix = -1.0
        self.last_update = -1.0

    def estimate_state(self, truth, dt, method="PERFECT", gnss=True):
        if method != self.was_method:
            self.reset()
            self.was_method = method
        if self.estimate is None:
            self.estimate = replace(truth)
            self.last_fix = truth.time
        if method == "PERFECT" and gnss:
            self.estimate = replace(truth)
            self.last_fix = truth.time
            return replace(self.estimate)
        e = self.estimate
        # IMU/velocidad sinteticas: solo una aproximacion para el taller.
        e.heading = wrap_pi(truth.heading + float(self.rng.normal(0.0, 0.002)))
        e.bank, e.roll_rate, e.airspeed, e.vz = truth.bank, truth.roll_rate, truth.airspeed, truth.vz
        e.altitude = truth.altitude + float(self.rng.normal(0.0, 0.15))
        e.x += e.vx * dt
        e.y += e.vy * dt
        if not gnss:
            e.vx += 0.04 * dt
            e.vy -= 0.02 * dt
        if gnss and truth.time - self.last_fix >= 1.0 / C.GPS_HZ - 1e-8:
            interval = max(1.0 / C.GPS_HZ, truth.time - self.last_fix)
            z = np.array([truth.x, truth.y]) + self.rng.normal(0, C.GPS_NOISE_M, 2)
            residual = z - np.array([e.x, e.y])
            if method == "FILTERED":
                e.x += C.FILTER_ALPHA * residual[0]
                e.y += C.FILTER_ALPHA * residual[1]
                e.vx += C.FILTER_BETA * residual[0] / interval
                e.vy += C.FILTER_BETA * residual[1] / interval
            else:
                e.x, e.y = float(z[0]), float(z[1])
                e.vx, e.vy = truth.vx, truth.vy
            self.last_fix = truth.time
        e.time = truth.time
        e.fix_age = max(0.0, truth.time - self.last_fix)
        e.valid = e.fix_age <= C.NAV_LOST_TIMEOUT_S
        return replace(e)
