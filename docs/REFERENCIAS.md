# Referencias tecnicas y alcance

El codigo de esta entrega es original y educativo. No se ha copiado el codigo de
PX4, de los articulos ni de un motor 3D. Estas referencias explican las ideas y las
interfaces de bibliotecas utilizadas.

## Guiado

- R. Craig Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*,
  CMU-RI-TR-92-01. Concepto geometrico de seguimiento y anticipacion.
  https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm
- Nelson, Barber, McLain y Beard, *Vector Field Path Following for Small Unmanned
  Aerial Vehicles*. Referencia conceptual de campos vectoriales para UAV.
  https://scholarsarchive.byu.edu/facpub/1536/

`guidance.py` contiene adaptaciones simplificadas. No se afirma reproducir las
leyes completas ni los resultados de los articulos.

## Vision

- OpenCV, umbrales HSV con `inRange`:
  https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html
- OpenCV, transformaciones de color:
  https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html

La reconstruccion de carretera es segmentacion por color y proyeccion sobre plano,
no SLAM ni deteccion neuronal.

## Render e interfaz

- Pillow, `ImageDraw` (rasterizacion de poligonos):
  https://pillow.readthedocs.io/en/stable/reference/ImageDraw.html
- Pillow, integracion con Tk:
  https://pillow.readthedocs.io/en/stable/reference/ImageTk.html
- Python, Tkinter:
  https://docs.python.org/3/library/tkinter.html
- NumPy:
  https://numpy.org/doc/stable/

La proyeccion pinhole y el renderizador de triangulos se implementan dentro de
`world.py`. No se necesitan ni se distribuyen recursos de terceros.

## Lenguaje natural

- Ollama, salidas estructuradas y schema en `/api/chat`:
  https://docs.ollama.com/capabilities/structured-outputs
- OpenAI, formato JSON y salidas estructuradas:
  https://developers.openai.com/api/docs/guides/structured-outputs?api-mode=chat
- Chat Completions, referencia de la interfaz usada por proveedores compatibles:
  https://developers.openai.com/api/reference/resources/chat/subresources/completions/methods/create

El modo compatible requiere que el proveedor/modelo acepte esa API y JSON mode.
La validacion local sigue siendo obligatoria: JSON valido no significa una orden
fisicamente valida, ni que el modelo haya entendido bien la intencion del usuario.

## Futuro PX4

- Arquitectura PX4:
  https://docs.px4.io/v1.16/en/concept/architecture
- Offboard y restricciones segun tipo de vehiculo:
  https://docs.px4.io/v1.16/en/flight_modes/offboard

No hay una conexion PX4 implementada o validada en este proyecto.
