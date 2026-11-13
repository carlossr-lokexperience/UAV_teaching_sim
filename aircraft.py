"""PLANTA: movimiento del avion. NO contiene guiado, LLM ni acceso al mapa.
Modelo cinematico de giro coordinado: heading_dot = g*tan(bank)/airspeed.
ENU: x=este, y=norte, z=arriba; heading=0 norte, positivo hacia este.
Bank positivo = ala derecha abajo. No hay perdida, resistencia ni fuerzas 6-DOF.
"""
from dataclasses import dataclass, replace
import math
import config as C

G = 9.80665

def clip(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    altitude: float = C.TARGET_ALTITUDE_M
    heading: float = 0.0
    bank: float = 0.0
    airspeed: float = C.TARGET_SPEED_MPS
    vx: float = 0.0
    vy: float = C.TARGET_SPEED_MPS
    vz: float = 0.0
    roll_rate: float = 0.0
    time: float = 0.0
    valid: bool = True
    fix_age: float = 0.0

    @property
    def ground_speed(self):
        return math.hypot(self.vx, self.vy)

    @property
    def course(self):
        return math.atan2(self.vx, self.vy) if self.ground_speed > 0.5 else self.heading

@dataclass
class Actuation:
    """Interfaz EDUCATIVA, NO PWM de servos: tasas y aceleracion simplificadas."""
    roll_rate: float = 0.0   # rad/s
    climb_rate: float = 0.0  # m/s
    acceleration: float = 0.0  # m/s2

class Aircraft:
    def __init__(self, initial):
        self.state = replace(initial)

    def get_true_state(self):
        return replace(self.state)

    def update_physics(self, actuation, dt, wind=(0.0, 0.0)):
        if not 0 < dt <= 0.1:
            raise ValueError("dt debe estar entre 0 y 0.1 s")
        if not all(math.isfinite(v) for v in (actuation.roll_rate, actuation.climb_rate,
                                               actuation.acceleration, *wind)):
            raise ValueError("Actuacion o viento no finitos")
        s = self.state
        rate = clip(actuation.roll_rate, -math.radians(C.MAX_ROLL_RATE_DEG_S), math.radians(C.MAX_ROLL_RATE_DEG_S))
        s.roll_rate += (rate - s.roll_rate) * (1 - math.exp(-dt / C.ROLL_ACTUATOR_TAU_S))
        s.bank = clip(s.bank + s.roll_rate * dt, -math.radians(C.PHYSICAL_BANK_LIMIT_DEG), math.radians(C.PHYSICAL_BANK_LIMIT_DEG))
        s.airspeed = clip(s.airspeed + clip(actuation.acceleration, -C.MAX_ACCEL_MPS2, C.MAX_ACCEL_MPS2) * dt,
                          C.MIN_SPEED_MPS, C.MAX_SPEED_MPS)
        climb = clip(actuation.climb_rate, -C.MAX_CLIMB_MPS, C.MAX_CLIMB_MPS)
        s.vz += (climb - s.vz) * (1 - math.exp(-dt / C.CLIMB_TAU_S))
        s.heading = wrap_pi(s.heading + G * math.tan(s.bank) / s.airspeed * dt)
        horizontal = math.sqrt(max(1.0, s.airspeed ** 2 - s.vz ** 2))
        s.vx = horizontal * math.sin(s.heading) + wind[0]
        s.vy = horizontal * math.cos(s.heading) + wind[1]
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.altitude = max(0.0, s.altitude + s.vz * dt)
        s.time += dt
        return self.get_true_state()
