# Validacion de esta entrega

## Ejecutado en este entorno

- Linux, Python 3.13.5, NumPy 2.3.5, Pillow 12.3.0, OpenCV 4.13.0 y Tk 8.6.
- Renderizado 3D y deteccion desde la imagen real, no imagenes precalculadas.
- 53 pruebas unitarias superadas: geometria, sentidos de giro, saturaciones,
  anti-windup, limites, comandos, proyeccion de camara, perdida de vision/GNSS,
  validacion atomica y formato HTTP de LLM con respuestas simuladas.
- Cuatro ejecuciones integradas de 120 segundos simulados, con los mismos pasos
  de fisica y algoritmos de la aplicacion. CAMERA genera y procesa cada imagen.
- Ventana ejecutada bajo Xvfb en Linux. Smoke test de envio de ordenes, cambio a
  CAMERA, teclas de giro y liberacion, proteccion del foco de escritura, pausa,
  captura, reinicio y cierre limpio.
- Comprobada la adaptacion a una pantalla de 1366 x 768. La captura de la entrega
  es una captura real de la ventana, no un mockup.
- Sintaxis de los diez modulos revisada con la gramatica de Python 3.9.

## Resultados de seguimiento

Configuracion base: 20 m/s, altura 100 m, lookahead 65 m, alabeo maximo 35 grados,
sin viento, navegacion PERFECT. El error se mide frente a la carretera real SOLO
para evaluacion. Estos numeros no son prestaciones de una aeronave real.

| Fuente | Guiado | Duracion | Error RMS | Error maximo | Mantiene FOLLOW_ROAD |
|---|---|---|---|---|---|
| MAP | PURE_PURSUIT | 120 s | 0.385 m | 0.860 m | SI |
| MAP | VECTOR_FIELD | 120 s | 0.045 m | 0.132 m | SI |
| CAMERA | PURE_PURSUIT | 120 s | 0.509 m | 1.210 m | SI |
| CAMERA | VECTOR_FIELD | 120 s | 0.306 m | 0.579 m | SI |

El trazado tiene 18 m de ancho. No se garantiza seguimiento con cualquier combinacion
de velocidad, alabeo, lookahead, viento y altura: provocar saturacion o perdida de
vision forma parte de los ejercicios. La tabla corresponde al ajuste base.

Datos reproducibles en `flight_results.json`.

## NO validado aqui

- Ejecucion en Windows, macOS o en el ordenador concreto del profesor.
- Ejecucion del proyecto bajo un interprete Python 3.10: es la referencia de
  instalacion recomendada, pero el interprete de pruebas disponible es 3.13.5.
- Instalacion limpia por pip en cada version y sistema operativo.
- Conexion a un modelo LLM real o proveedor externo. El adaptador HTTP y el parseo
  se han probado con respuestas simuladas, no con una clave/modelo real.
- Cualquier conexion con PX4, QGC o una aeronave fisica: no se implementa.
- Seguimiento visual en cruces, oclusiones complejas, terreno irregular o datos reales.

## Repetir las comprobaciones

```bash
python -m unittest discover -s tests -v
python tests/validate_flights.py
python tests/ui_smoke.py
python main.py --headless --source CAMERA --seconds 30 --output prueba
```

`ui_smoke.py` requiere una pantalla. En Linux sin monitor:

```bash
xvfb-run -a -s '-screen 0 1366x768x24' python tests/ui_smoke.py
```

El resto no necesita pantalla. `tests/validate_flights.py` puede tardar mas en un
ordenador lento, porque CAMERA renderiza y procesa imagenes de verdad.

## Interpretacion correcta

El proyecto es una plataforma docente funcional en el entorno de prueba, no un
producto certificado ni una promesa de ausencia de errores en todos los equipos.
Antes de clase, ejecuta la prueba base en el ordenador que vas a utilizar.
