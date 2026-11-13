# Arquitectura y contratos

## Las tres decisiones que no hay que mezclar

1. **Modo:** que hacer: seguir carretera, mantener rumbo o pilotaje asistido.
2. **Guiado:** como seguir: Pure Pursuit o Vector Field.
3. **Fuente:** de donde sale la carretera: mapa o imagen.

No existe `ORBIT`. La carretera cerrada solo permite repetir vueltas al circuito.

## Flujo por iteracion

```text
TECLADO / REGLAS / LLM
    -> commands.validate_command() -> Settings (consignas, nunca actuadores)

Aircraft.true_state ---------> World.render_downward() -> RGB
         |                                                |
         v                                                v
Navigation.estimate_state() -----------------------> Perception
         |                                    pose -> PathObservation
         |                                                |
         +---------> Autopilot <--------------------------+
                         |                 alternativa: camino de MAP
                         v
                  GuidanceCommand
                         |
                         v
                  Controller.update()
                         |
                         v
                     Actuation
                         |
                         v
                Aircraft.update_physics()
```

El render necesita la verdad del simulador. El guiado no la recibe. La evaluacion
puede compararla con las consignas, sin devolverla al control.

## Contratos de datos, unidades y convenciones

### State (aircraft.py)

- `x,y`: este y norte, metros. `altitude`: altura sobre el plano z=0.
- `heading`: radianes desde norte, sentido horario. No es un yaw ENU matematico.
- `bank`: radianes, positivo ala derecha abajo.
- `airspeed`: velocidad respecto al aire; `vx,vy,vz`: velocidad sobre el terreno.
- `course`: direccion real sobre el terreno, puede diferir del rumbo con viento.
- `time`: segundos simulados; `fix_age`: edad del ultimo GNSS; `valid`: pose util.

La simulacion se integra con `PHYSICS_DT=0.02`. No confundas FPS de dibujo y Hz de
control. La vision se actualiza a `CAMERA_HZ`; las detecciones llevan timestamp.

### PathObservation (perception.py)

`points`: polilinea XY en metros, ordenada de detras hacia delante.
`confidence`, `time`, `mask`, `overlay`, `reason` documentan calidad y edad.
No contiene una referencia al mapa. Su proyeccion usa la pose estimada en la
captura, no una pose posterior.

### GuidanceCommand (guidance.py)

`bank_rad`, `altitude_m`, `speed_mps` son referencias. `target`, `cross_track_m`
y `status` son diagnosticos. Ambos guiados devuelven este mismo contrato.

### Actuation (aircraft.py)

`roll_rate` (rad/s), `climb_rate` (m/s) y `acceleration` (m/s2).
Es una interfaz de planta simplificada, NO alerones/elevador/PWM. La planta aplica
limites y respuesta gradual. Se ha evitado fingir que simulamos aerodinamica completa.

## Que archivo modificar para cada experimento

| Experimento | Archivo |
|---|---|
| Elegir configuraciones | config.py |
| Un nuevo algoritmo lateral | guidance.py; registrar en METHODS y enumeraciones |
| Limitar alabeo solicitado / variar ganancias | control.py y config.py |
| Limites fisicos, respuesta de alabeo, viento | aircraft.py y config.py |
| Cambiar la carretera o apariencia | world.py |
| Mejorar deteccion por color | perception.py |
| Ruido o filtro de posicion | navigation.py |
| Anadir una orden | commands.py: schema, parser y validacion |
| Reaccionar a una referencia perdida | autopilot.py |
| Cambiar controles de ventana / registros | main.py |

Al anadir una opcion a la configuracion, actualiza su validacion y el desplegable.
No necesitas tocar main.py para modificar las ecuaciones de un guiado existente.

## Guiado implementado

**Pure Pursuit:** proyeccion al segmento mas cercano, avance por longitud de arco
en la polilinea, objetivo adelantado y aceleracion lateral geometrica.
El avance se mide SOBRE la carretera; el denominador usa la distancia euclidea
al objetivo. Es una variante docente, no una copia exacta de un paquete de robotica.

`a_lat = 2 * Vg**2 * e_right / distance_to_target**2`

`bank_ref = atan2(a_lat, g)`

**Vector Field:** rumbo de la tangente mas convergencia lateral, con una correccion
proporcional de rumbo y anticipacion de curvatura. La implementacion es simplificada,
inspirada en la idea de campo vectorial, no una reproduccion literal de los articulos.

`direction = tangent - k * cross_track_error * right_normal`

Ambos pueden pedir mas alabeo del permitido. El controlador limita la consigna;
no existe una garantia de seguir cualquier curva a cualquier velocidad.

## Camara y geometria

Ambas vistas se renderizan desde triangulos del mismo mundo 3D. La inferior es
pinhole, nadir y estabilizada. El avion y las ayudas amarillas se excluyen del sensor.

Para un pixel `(u,v)`, una imagen de ancho W/alto H y altura h:

```text
f = (W/2) / tan(HFOV/2)
right_m = (u-W/2) * h/f
ahead_m = (H/2-v) * h/f
world_xy = estimated_position + right_m*right_axis + ahead_m*forward_axis
```

Esto requiere altura, actitud de camara y suelo plano conocidos. Seguir una carretera
por color no equivale a estimar la posicion con vision. Una futura VIO/matching
debe alimentar navigation.py; una segmentacion de carretera alimenta perception.py.

## Limites y fallos

La velocidad minima/maxima y limites de tasas se aplican en la planta. El alabeo
maximo operativo se aplica antes, en el control. El aspecto visual no determina esos
limites. El viento es una perturbacion del movimiento sobre el terreno.

Una orden se valida completa antes de cambiar Settings. El LLM no toca la planta ni
el bucle de vuelo. Una camara obsoleta o perdida de posicion activa HEADING; no se
habilita una fuente de datos alternativa sin que el alumno lo vea.

## Futuro adaptador PX4: lo que SI es reutilizable

Reutilizables: traduccion/validacion de comandos, interfaces de estado, percepcion y
objetivos de trayectoria. Cambiar la planta exige convertir unidades y marcos.

- Enviar una Mission de waypoints a PX4 reutiliza la mision, pero **PX4 hace el guiado**.
- Probar nuestro guiado requiere una interfaz externa compatible con esa version,
  tipo de vehiculo y consignas. No basta con enviar Actuation por MAVLink.
- Las consignas de banco, altitud y velocidad no forman por si mismas un mensaje
  MAVLink valido. Hay que seleccionar y verificar la interfaz.

No se incluye un adaptador ficticio ni se afirma compatibilidad de vuelo no probada.
