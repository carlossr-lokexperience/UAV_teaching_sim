"""
INTRODUCCION A PYTHON - CLASE DE UAV
====================================
Un unico archivo para aprender Python desde cero.
No necesita librerias externas ni tiene relacion con el simulador UAV_CLASS.

EJECUCION (Windows, PowerShell):
    python intro_python_uav.py

"""

# =============================================================
# 1. IMPRIMIR EN PANTALLA
# =============================================================
# Un comentario empieza con #. Python no ejecuta los comentarios.
print("\n1. MI PRIMER PROGRAMA")
print("Hola, soy un UAV")


# =============================================================
# 2. VARIABLES Y TIPOS DE DATOS
# =============================================================
# Una variable es un nombre asociado a un valor.
print("\n2. VARIABLES")
altura = 150                 # int: numero entero
velocidad = 20.5             # float: numero decimal
modo = "AUTOMATICO"          # str: texto
motor_encendido = True       # bool: verdadero o falso
objetivo = None              # None: no hay un valor asignado

print("Altura:", altura, "m")
print("Velocidad:", velocidad, "m/s")
print("Modo:", modo)
print("Motor encendido:", motor_encendido)

# Podemos modificar una variable haciendo operaciones.
altura = altura + 20
print("Nueva altura:", altura, "m")


# =============================================================
# 3. DECISIONES: IF, ELIF, ELSE
# =============================================================
print("\n3. DECISIONES")
if velocidad > 30:
    print("Velocidad demasiado alta")
elif velocidad < 12:
    print("Velocidad demasiado baja")
else:
    print("Velocidad dentro del intervalo")

# == compara dos valores; = asigna un valor a una variable.
if modo == "AUTOMATICO" and motor_encendido:
    print("El avion esta listo para control automatico")


# =============================================================
# 4. BUCLES: FOR Y WHILE
# =============================================================
print("\n4. BUCLES")

# for: repite algo un numero concreto de veces.
for segundo in range(3):
    print("Segundo de simulacion:", segundo)

# while: repite mientras se cumpla una condicion.
altura_prueba = 100
while altura_prueba < 115:
    altura_prueba = altura_prueba + 5
    print("Ascendiendo a:", altura_prueba, "m")


# =============================================================
# 5. FUNCIONES: DEF, PARAMETROS Y RETURN
# =============================================================
print("\n5. FUNCIONES")

# Una funcion recibe datos, realiza operaciones y devuelve un resultado.
def calcular_nueva_altura(altura_actual, metros_a_subir):
    nueva_altura = altura_actual + metros_a_subir  # Variable LOCAL
    return nueva_altura

altura = calcular_nueva_altura(altura, 30)
print("Altura despues de llamar a la funcion:", altura, "m")


# =============================================================
# 6. LISTAS, TUPLAS Y DICCIONARIOS
# =============================================================
print("\n6. ESTRUCTURAS DE DATOS")

alturas_objetivo = [100, 120, 150]   # list: se puede modificar
posicion = (10, 20)                 # tuple: no se modifica elemento a elemento
estado = {"altura": altura, "velocidad": velocidad}  # dict: clave -> valor

print("Primera altura objetivo:", alturas_objetivo[0])  # Los indices empiezan en 0
print("Posicion XY:", posicion)
print("Altura guardada en el diccionario:", estado["altura"])


# =============================================================
# 7. VARIABLES GLOBALES Y LOCALES
# =============================================================
print("\n7. ALCANCE DE LAS VARIABLES")

ALTURA_MAXIMA = 200  # Variable GLOBAL de este archivo (mayusculas por convencion)

def comprobar_altura(altura_actual):
    # 'altura_actual' es un parametro y existe dentro de esta funcion.
    # Podemos LEER la variable global ALTURA_MAXIMA.
    if altura_actual > ALTURA_MAXIMA:
        return "Demasiado alto"
    return "Altura permitida"

print(comprobar_altura(altura))
# Nota: para cambiar una global desde una funcion existe 'global',
# pero normalmente es mas claro pasar datos como parametros.


# =============================================================
# 8. CLASES, OBJETOS, SELF Y METODOS
# =============================================================
print("\n8. CLASES Y OBJETOS")

# Una clase es una plantilla para crear objetos.
class Avion:
    def __init__(self, nombre, altura_inicial):
        # self = el objeto concreto que estamos creando o usando.
        self.nombre = nombre
        self.altura = altura_inicial  # Atributo del objeto

    def subir(self, metros):
        # Este metodo modifica la altura DE ESTE avion.
        self.altura = self.altura + metros

# Creamos dos OBJETOS diferentes de la misma clase.
avion1 = Avion("UAV-1", 100)
avion2 = Avion("UAV-2", 150)
avion1.subir(25)

print(avion1.nombre, "vuela a", avion1.altura, "m")
print(avion2.nombre, "vuela a", avion2.altura, "m")


# =============================================================
# 9. IMPORTS: REUTILIZAR CODIGO DE OTROS MODULOS
# =============================================================
print("\n9. IMPORTS")

import math  # 'math' es un modulo de la biblioteca estandar de Python.
print("Raiz cuadrada de 25:", math.sqrt(25))

# En un proyecto con varios archivos seria parecido a esto:
# from guidance import pure_pursuit
# La funcion se define en guidance.py y se utiliza desde otro archivo.


# =============================================================
# 10. ERRORES Y PRUEBAS SENCILLAS
# =============================================================
print("\n10. MANEJO DE ERRORES")

try:
    numero = int("abc")  # No se puede convertir 'abc' a numero entero.
except ValueError:
    print("Error controlado: el texto no es un numero")

print("\nFIN: ya has utilizado las principales piezas de Python.")

# =============================================================
# RETOS PARA EL ALUMNO
# =============================================================
# 1. Cambia la velocidad inicial para activar otro mensaje del if.
# 2. Haz que el while termine en 130 metros en lugar de 115.
# 3. Cambia la funcion para que TAMBIEN pueda bajar metros.
# 4. Crea un tercer avion y prueba el metodo subir().
# 5. Anade un metodo bajar() a la clase Avion.
