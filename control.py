"""CONTROL: referencias -> actuacion educativa. No conoce carretera ni imagenes.
La referencia puede pedir mas bank del posible: aqui se satura, y la planta
aplica ademas su propio limite fisico. No son salidas PWM ni un PX4 real.
"""
import math
import config as C
from aircraft import Actuation, clip

class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, limit=1.0):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.reset()

    def reset(self):
        self.integral, self.previous, self.derivative = 0.0, None, 0.0

    def update(self, error, dt):
        if dt <= 0:
            raise ValueError("dt debe ser positivo")
        raw_d = 0.0 if self.previous is None else (error-self.previous)/dt
        self.derivative += (1-math.exp(-dt/0.12))*(raw_d-self.derivative)
        candidate = self.integral + error*dt
        raw = self.kp*error + self.ki*candidate + self.kd*self.derivative
        result = clip(raw, -self.limit, self.limit)
        # Anti-windup: no acumular si el error empuja mas hacia la saturacion.
        if abs(raw) <= self.limit or error*raw < 0:
            self.integral = candidate
        self.previous = error
        return result

class Controller:
    def __init__(self):
        self.method = None
        self.bank_reference = 0.0
        self.reset()

    def reset(self):
        self.roll = PID(C.ROLL_KP, C.ROLL_KI, C.ROLL_KD, math.radians(C.MAX_ROLL_RATE_DEG_S))
        self.altitude = PID(C.ALTITUDE_KP, C.ALTITUDE_KI, 0, C.MAX_CLIMB_MPS)
        self.speed = PID(C.SPEED_KP, C.SPEED_KI, 0, C.MAX_ACCEL_MPS2)

    def update(self, state, ref, dt, settings):
        if settings.controller != self.method:
            self.reset()
            self.method = settings.controller
        limit = math.radians(min(settings.bank_limit_deg, C.PHYSICAL_BANK_LIMIT_DEG))
        self.bank_reference = clip(ref.bank_rad, -limit, limit)
        bank_error = self.bank_reference - state.bank
        alt_error = clip(ref.altitude_m, C.MIN_ALTITUDE_M, C.MAX_ALTITUDE_M)-state.altitude
        speed_error = clip(ref.speed_mps, C.MIN_SPEED_MPS, C.MAX_SPEED_MPS)-state.airspeed
        if self.method == "PID":
            return Actuation(self.roll.update(bank_error, dt), self.altitude.update(alt_error, dt), self.speed.update(speed_error, dt))
        return Actuation(clip(C.ROLL_KP*bank_error, -math.radians(C.MAX_ROLL_RATE_DEG_S), math.radians(C.MAX_ROLL_RATE_DEG_S)),
                          clip(C.ALTITUDE_KP*alt_error, -C.MAX_CLIMB_MPS, C.MAX_CLIMB_MPS),
                          clip(C.SPEED_KP*speed_error, -C.MAX_ACCEL_MPS2, C.MAX_ACCEL_MPS2))
