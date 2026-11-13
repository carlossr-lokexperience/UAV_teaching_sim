# Guion practico para el profesor

## Antes de entrar

Ejecuta `python main.py` y prueba MAP/CAMERA y los dos guiados. Si vas a usar LLM,
configura y prueba el proveedor/modelo antes de la clase. Deja RULES como respaldo,
explicando claramente que no es IA. No instales un modelo ni PX4 durante la clase.

Lleva `main.py`, `config.py`, `commands.py`, `guidance.py` y `perception.py` abiertos.
Los alumnos solo necesitan el ZIP o clon del repositorio y su entorno Python.

## Propuesta de taller de 2 horas

| Minutos | Accion en pantalla | Cambio que hacen ellos |
|---|---|---|
| 0-10 | Ejecutar, cambiar altura/velocidad y comparar las dos camaras | Abrir config.py y localizar selecciones |
| 10-30 | LLM o RULES -> JSON -> validacion -> Settings | Anadir una orden basica y probar un valor fuera de limites |
| 30-45 | Recorrer Simulation.step() sin reexplicar toda la teoria | Identificar donde entran la pose y el camino |
| 45-70 | Seguir por mapa, modificar lookahead y limite de alabeo | Probar 30/65/110 m a la misma velocidad |
| 70-90 | MAP -> CAMERA, mostrar RGB y deteccion | Cambiar HSV; provocar y explicar una perdida de carretera |
| 90-105 | Pure Pursuit -> Vector Field | Comparar; modificar VECTOR_CONVERGENCE, no lookahead |
| 105-120 | Elegir una mejora y guardar CSV/capturas | Entregar configuracion y explicar el comportamiento observado |

La base se entrega completa para que la clase produzca resultados. No hace falta
programar el mundo ni un autopiloto desde cero. Tu version y la de los alumnos son
la misma: cambiar configuraciones no exige otra rama de codigo.

## Secuencia de demostracion sugerida

1. MAP + PURE_PURSUIT, 20 m/s, 100 m, 65 m de lookahead, bank maximo 35 grados.
2. Cambiar a 30 m de anticipacion y despues a 110 m: observar objetivo y curvas.
3. Bajar el alabeo permitido a 10 grados y subir velocidad a 30 m/s: ver saturacion.
4. Reiniciar, cambiar a CAMERA: ahora la referencia viene de la imagen.
5. Abrir `detect_road()`: color -> mascara -> linea -> metros.
6. Cambiar a VECTOR_FIELD. Explicar por que ahora el parametro relevante es otro.
7. Escribir `gira 30 grados a la derecha`; comprobar HEADING, no FOLLOW_ROAD.
8. Escribir `sigue la carretera con camara`; comprobar si aun entra en el FOV.
9. Probar `velocidad 100`: la orden se rechaza sin cambiar altura ni modo.
10. Si hay LLM preparado, pedir la misma capacidad con frases diferentes y mostrar
    el JSON que devuelve. La salida sigue pasando por validate_command().

## Experimentos independientes

- **Navegacion:** NOISY y FILTERED con la misma semilla; comparar estimacion y verdad.
- **GNSS:** tecla N; el dead reckoning sintetico deriva y despues se declara obsoleto.
- **Percepcion:** ampliar/reducir umbrales HSV. No confundir mascara mala con guiado malo.
- **Control:** P/PID; modificar una ganancia y mantener lo demas fijo.
- **Velocidad y giro:** a mismo alabeo, mas velocidad requiere mas radio de giro.
- **Viento:** poner WIND_EAST_MPS=3 y comparar rumbo con curso sobre el terreno.

## Actividad para entregar

Una captura, un CSV, valores utilizados y una explicacion de tres frases:
que cambiaste, que esperabas y que observaste. Un solo factor por comparacion.
No valorar solo una trayectoria bonita: deben explicar de donde salio la referencia.
