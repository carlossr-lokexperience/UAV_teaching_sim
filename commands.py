# API utilizada: docs/REFERENCIAS.md (Ollama y Chat Completions).
"""ORDENES: texto -> JSON permitido -> validacion -> configuracion.
RULES es un parser limitado, NO un LLM. LLM usa HTTP real (Ollama o compatible).
Nunca eval/exec, nunca instala nada, nunca genera codigo ni manda actuadores.
La GUI llama al LLM en un hilo; los cambios se aplican en el hilo de simulacion.
"""
import json
import math
import os
import re
import unicodedata
from dataclasses import replace, asdict
from urllib.request import Request, urlopen
from urllib.error import URLError
import config as C
from aircraft import wrap_pi

FIELDS = {
    "mode": {"type": "string", "enum": ["FOLLOW_ROAD", "HEADING", "MANUAL"]},
    "source": {"type": "string", "enum": ["MAP", "CAMERA"]},
    "guidance": {"type": "string", "enum": ["PURE_PURSUIT", "VECTOR_FIELD"]},
    "altitude_m": {"type": "number"}, "altitude_delta_m": {"type": "number"},
    "speed_mps": {"type": "number"}, "speed_delta_mps": {"type": "number"},
    "heading_deg": {"type": "number"}, "heading_delta_deg": {"type": "number"},
    "lookahead_m": {"type": "number"}, "bank_limit_deg": {"type": "number"},
}
COMMAND_SCHEMA = {"type": "object", "properties": FIELDS, "additionalProperties": False}


def normalize(text):
    return "".join(c for c in unicodedata.normalize("NFD", text.lower()) if unicodedata.category(c) != "Mn")


def parse_rules(text):
    """Formas soportadas documentadas en README; los numeros son cifras, no palabras."""
    if text.strip().startswith("{"):
        return json.loads(text)
    t = normalize(text)
    result = {}
    number = r"(-?\d+(?:[.,]\d+)?)"
    def num(value):
        return float(value.replace(",", "."))
    if re.search(r"\b(orbita|aterriza|despega|hover)\b", t):
        raise ValueError("No hay orbita, despegue ni aterrizaje: la practica empieza en vuelo")
    if "carretera" in t and re.search(r"sigue|seguir|activa|seguimiento", t):
        result["mode"] = "FOLLOW_ROAD"
    if "manual" in t:
        result["mode"] = "MANUAL"
    if "manten rumbo" in t or "mantener rumbo" in t:
        result["mode"] = "HEADING"
    if "camara" in t or "vision" in t:
        result["source"] = "CAMERA"
    if "mapa" in t:
        result["source"] = "MAP"
    if "pure pursuit" in t or "pursuit" in t:
        result["guidance"] = "PURE_PURSUIT"
    if "vector" in t or "campo" in t:
        result["guidance"] = "VECTOR_FIELD"
    # 'sube A 120' es absoluto; 'sube 20' es incremento.
    height = re.search(r"\b(sube|baja|ascender|desciende)\s+(a\s+)?"+number+r"\s*(?:m(?:etros)?\b)?", t)
    if height:
        sign = -1 if height.group(1) in ("baja", "desciende") else 1
        result["altitude_m" if height.group(2) else "altitude_delta_m"] = num(height.group(3))*(1 if height.group(2) else sign)
    else:
        height = re.search(r"(?:altura|altitud)\s*(?:a|de|=)?\s*"+number, t)
        if not height:
            height = re.search(r"\ba\s+"+number+r"\s*metros\b", t)
        if height:
            result["altitude_m"] = num(height.group(1))
    speed = re.search(r"(?:(aumenta|reduce|baja|sube)\s+(?:la\s+)?)?velocidad\s*(a\s+|de\s+|=\s*)?"+number, t)
    if speed:
        delta = bool(speed.group(1)) and not speed.group(2)
        value = num(speed.group(3))
        if delta and speed.group(1) in ("reduce", "baja"):
            value = -value
        result["speed_delta_mps" if delta else "speed_mps"] = value
    else:
        speed = re.search(number+r"\s*m/s", t)
        if speed:
            result["speed_mps"] = num(speed.group(1))
    turn = re.search(r"gira\s+"+number+r"\s*(?:grados)?\s*(?:a\s+la\s+)?(izquierda|derecha)", t)
    if turn:
        result["heading_delta_deg"] = num(turn.group(1))*(-1 if turn.group(2) == "izquierda" else 1)
    absolute_heading = re.search(r"rumbo\s*(?:a\s+)?"+number, t)
    if absolute_heading:
        result["heading_deg"] = num(absolute_heading.group(1))
    for pattern, key in [(r"(?:lookahead|anticipacion)\s*(?:a|de|=)?\s*", "lookahead_m"),
                         (r"(?:alabeo|bank)\s*(?:maximo)?\s*(?:a|de|=)?\s*", "bank_limit_deg")]:
        match = re.search(pattern+number, t)
        if match:
            result[key] = num(match.group(1))
    if not result:
        raise ValueError("Orden no reconocida. Ejemplo: sigue la carretera con camara; lookahead 80")
    return result


def validate_command(command, settings, state):
    """Devuelve una COPIA validada. O se aplica toda la orden, o no se aplica nada."""
    if not isinstance(command, dict) or not command:
        raise ValueError("Se espera un objeto JSON no vacio")
    unknown = set(command)-set(FIELDS)
    if unknown:
        raise ValueError("Campos no permitidos: " + ", ".join(sorted(unknown)))
    for key, value in command.items():
        spec = FIELDS[key]
        if spec["type"] == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(key + " necesita un numero finito")
        elif value not in spec["enum"]:
            raise ValueError("Valor no permitido para " + key)
    for absolute, relative in [("altitude_m", "altitude_delta_m"), ("speed_mps", "speed_delta_mps"), ("heading_deg", "heading_delta_deg")]:
        if absolute in command and relative in command:
            raise ValueError("No mezcles " + absolute + " y " + relative)
    candidate = replace(settings)
    for field in ("mode", "source", "guidance", "altitude_m", "speed_mps", "lookahead_m", "bank_limit_deg"):
        if field in command:
            setattr(candidate, field, command[field])
    if "altitude_delta_m" in command:
        candidate.altitude_m += command["altitude_delta_m"]
    if "speed_delta_mps" in command:
        candidate.speed_mps += command["speed_delta_mps"]
    if "heading_deg" in command or "heading_delta_deg" in command:
        if command.get("mode", "HEADING") != "HEADING":
            raise ValueError("Girar a un rumbo activa HEADING; no mezcles con seguir carretera")
        candidate.mode = "HEADING"
        candidate.heading_rad = wrap_pi(math.radians(command["heading_deg"]) if "heading_deg" in command
                                        else state.heading+math.radians(command["heading_delta_deg"]))
    elif candidate.mode == "HEADING" and settings.mode != "HEADING":
        candidate.heading_rad = state.heading
    if candidate.mode == "MANUAL" and settings.mode != "MANUAL":
        candidate.manual_bank_rad = 0.0
    return candidate.validate()


def call_llm(text, settings, opener=None):
    """HTTP estandar; no requiere SDK. El servidor/modelo lo configura el profesor."""
    if not C.LLM_MODEL:
        raise ValueError("Configura LLM_MODEL en config.py; no se ha descargado ningun modelo")
    system = ("Traduce instrucciones de un simulador educativo de avion a UN JSON. "
              "No generes codigo ni explicaciones. No inventes capacidades ni parametros. "
              "No hay orbita, aterrizaje ni despegue. Rechaza orden desconocida devolviendo {}. "
              "sube a=altitud absoluta, sube sin a=incremento. Girar izquierda: delta negativo. "
              "Girar cambia a HEADING. Los limites son: velocidad 12..32 m/s, altura 30..220 m, "
              "lookahead 15..150 m, bank 5..60 grados. Schema: " + json.dumps(COMMAND_SCHEMA) + " Consignas actuales: " + json.dumps(asdict(settings)))
    payload = {"model": C.LLM_MODEL, "stream": False,
               "messages": [{"role": "system", "content": system}, {"role": "user", "content": text}]}
    headers = {"Content-Type": "application/json"}
    if C.LLM_PROVIDER == "OLLAMA":
        url = C.LLM_BASE_URL.rstrip("/") + "/api/chat"
        payload.update({"format": COMMAND_SCHEMA, "options": {"temperature": 0}})
    elif C.LLM_PROVIDER == "OPENAI_COMPATIBLE":
        url = C.LLM_BASE_URL.rstrip("/") + "/chat/completions"
        payload.update({"temperature": 0, "response_format": {"type": "json_object"}})
        key = os.environ.get(C.LLM_API_KEY_ENV, "")
        if key:
            headers["Authorization"] = "Bearer " + key
    else:
        raise ValueError("LLM_PROVIDER desconocido")
    request = Request(url, data=json.dumps(payload).encode("utf-8"), headers=headers, method="POST")
    try:
        with (opener or urlopen)(request, timeout=C.LLM_TIMEOUT_S) as response:
            raw = response.read(C.LLM_MAX_RESPONSE_BYTES + 1)
        if len(raw) > C.LLM_MAX_RESPONSE_BYTES:
            raise ValueError("Respuesta del LLM demasiado grande")
        data = json.loads(raw.decode("utf-8"))
        content = data["message"]["content"] if C.LLM_PROVIDER == "OLLAMA" else data["choices"][0]["message"]["content"]
        return json.loads(content)
    except (URLError, TimeoutError, OSError, KeyError, IndexError, json.JSONDecodeError) as exc:
        raise ValueError("LLM no disponible/respuesta invalida: " + str(exc)) from exc
