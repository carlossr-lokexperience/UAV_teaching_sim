"""MUNDO Y CAMARAS. Renderizador 3D ligero con NumPy + Pillow (sin motor externo).
Geometria 3D, proyeccion perspectiva, recorte cercano y orden de profundidad.
No es un mapa 2D disfrazado: ambas camaras proyectan los mismos triangulos 3D.
La camara nadir excluye avion y ayudas visuales; esa imagen alimenta OpenCV.
Limitacion deliberada: terreno plano y painter's algorithm, sin z-buffer/sombras.
El mundo no determina como gira el avion. Toda la dinamica esta en aircraft.py.
"""
from dataclasses import dataclass
import math
import numpy as np
from PIL import Image, ImageDraw
import config as C
from aircraft import State

@dataclass
class Camera:
    eye: object
    right: object
    up: object
    forward: object
    width: int
    height: int
    hfov_deg: float

    @classmethod
    def look_at(cls, eye, target, width, height, hfov_deg=68.0, up=(0, 0, 1)):
        eye = np.asarray(eye, dtype=float)
        forward = np.asarray(target, dtype=float)-eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, np.asarray(up, dtype=float))
        right /= max(np.linalg.norm(right), 1e-9)
        true_up = np.cross(right, forward)
        return cls(eye, right, true_up, forward, width, height, hfov_deg)

    @classmethod
    def nadir(cls, state):
        forward = np.array([math.sin(state.heading), math.cos(state.heading), 0.0])
        return cls.look_at((state.x, state.y, state.altitude), (state.x, state.y, 0),
                           C.CAMERA_SIZE, C.CAMERA_SIZE, C.CAMERA_HFOV_DEG, forward)

    def project(self, points):
        p = np.asarray(points)-self.eye
        xyz = p @ np.column_stack((self.right, self.up, self.forward))
        focal = self.width / (2*math.tan(math.radians(self.hfov_deg)/2))
        z = np.maximum(xyz[..., 2], 1e-6)
        screen = np.stack((self.width/2+focal*xyz[..., 0]/z,
                           self.height/2-focal*xyz[..., 1]/z), axis=-1)
        return screen, xyz[..., 2]

class Mesh:
    def __init__(self):
        self.triangles, self.colors, self.layers = [], [], []

    def polygon(self, points, color, layer=2):
        points = np.asarray(points, dtype=float)
        for i in range(1, len(points)-1):
            self.triangles.append([points[0], points[i], points[i+1]])
            self.colors.append(tuple(int(v) for v in color))
            self.layers.append(layer)

    def box(self, center, size, color, layer=2):
        cx, cy, cz = center
        sx, sy, sz = np.asarray(size)/2
        v = [(cx+x*sx, cy+y*sy, cz+z*sz) for x,y,z in
             [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        faces = [(0,1,2,3),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7),(4,5,6,7)]
        for ids, factor in zip(faces, (0.65,0.78,0.88,0.74,0.85,1.0)):
            self.polygon([v[i] for i in ids], np.array(color)*factor, layer)

    def pyramid(self, center, radius, height, color):
        x,y,z = center
        v = [(x-radius,y-radius,z),(x+radius,y-radius,z),(x+radius,y+radius,z),(x-radius,y+radius,z)]
        for i in range(4):
            self.polygon([v[i],v[(i+1)%4],(x,y,z+height)], np.array(color)*(0.75+0.07*i))

    def arrays(self):
        return np.asarray(self.triangles, dtype=float), np.asarray(self.colors, dtype=np.uint8), np.asarray(self.layers)


def make_road():
    """Carretera cerrada para repetir vueltas; NO hay un modo de orbita.
    Cambia la curva aqui, manteniendo radios compatibles con velocidad y alabeo.
    """
    t = np.linspace(0, 2*math.pi, C.ROAD_SAMPLES+1)
    return np.column_stack((580*np.sin(t)+65*np.sin(3*t), 480*np.cos(t)+45*np.sin(2*t)))


def _clip_near(polygon, near=1.0):
    output = []
    previous = polygon[-1]
    for current in polygon:
        inside, was_inside = current[2] >= near, previous[2] >= near
        if inside != was_inside:
            ratio = (near-previous[2])/(current[2]-previous[2])
            output.append(previous+ratio*(current-previous))
        if inside:
            output.append(current)
        previous = current
    return output


def render_mesh(triangles, colors, layers, camera, background=(166, 204, 224)):
    """Render CPU, sin OpenCV: MAP puede funcionar aunque no actives percepcion."""
    image = Image.new("RGB", (camera.width, camera.height), background)
    draw = ImageDraw.Draw(image)
    basis = np.column_stack((camera.right, camera.up, camera.forward))
    xyz = (triangles-camera.eye) @ basis
    focal = camera.width/(2*math.tan(math.radians(camera.hfov_deg)/2))
    valid = (xyz[:,:,2].max(axis=1) > 1.0) & (xyz[:,:,2].min(axis=1) < C.SCENE_RANGE_M)
    # Culling vectorizado: solo dibujamos triangulos que pueden aparecer en pantalla.
    safe_z = np.maximum(xyz[:,:,2], 1.0)
    u = camera.width/2 + focal*xyz[:,:,0]/safe_z
    v = camera.height/2 - focal*xyz[:,:,1]/safe_z
    entirely_front = xyz[:,:,2].min(axis=1) >= 1.0
    outside = (u.max(axis=1)<0) | (u.min(axis=1)>=camera.width) | (v.max(axis=1)<0) | (v.min(axis=1)>=camera.height)
    valid &= ~(entirely_front & outside)
    indices = np.flatnonzero(valid)
    # Terreno, asfalto y objetos: capas para eliminar z-fighting en el suelo plano.
    indices = indices[np.lexsort((-xyz[indices,:,2].mean(axis=1), layers[indices]))]
    for idx in indices:
        p = xyz[idx]
        if np.min(p[:,2]) < 1.0:
            p = np.asarray(_clip_near(p))
            if len(p) < 3:
                continue
        uv = np.column_stack((camera.width/2+focal*p[:,0]/p[:,2], camera.height/2-focal*p[:,1]/p[:,2]))
        if uv[:,0].max() < 0 or uv[:,0].min() >= camera.width or uv[:,1].max() < 0 or uv[:,1].min() >= camera.height:
            continue
        draw.polygon([tuple(v) for v in np.clip(uv,-50000,50000)], fill=tuple(int(c) for c in colors[idx]))
    return image

class World:
    def __init__(self):
        self.road = make_road()
        self.mesh = Mesh()
        self.create_world()
        self.scene_tri, self.scene_col, self.scene_layers = self.mesh.arrays()
        self.airplane = self._aircraft_mesh()
        self.trail = []

    def get_road_map(self):
        # Solo MAP lo usa para guiar. La evaluacion puede leerlo para medir error.
        return self.road.copy()

    def initial_state(self):
        i = min(len(self.road)-2, int(C.START_ROAD_FRACTION*(len(self.road)-1)))
        p, q = self.road[i], self.road[i+1]
        heading = math.atan2(q[0]-p[0],q[1]-p[1])
        speed = C.TARGET_SPEED_MPS
        return State(x=float(p[0]), y=float(p[1]), heading=heading,
                     vx=speed*math.sin(heading), vy=speed*math.cos(heading))

    def create_world(self):
        rng = np.random.default_rng(C.RANDOM_SEED)
        # Parcelas grandes coloreadas; sin imagenes, descargas ni modelos externos.
        for x in range(-1800,1800,200):
            for y in range(-1800,1800,200):
                palettes = [(88,137,75),(103,150,82),(113,149,76),(122,155,85),(89,142,91)]
                color = palettes[int(rng.integers(len(palettes)))]
                self.mesh.polygon([(x,y,0),(x+200,y,0),(x+200,y+200,0),(x,y+200,0)],color,0)
        # Lago fuera del trazado: no se parece al asfalto en HSV.
        self.mesh.polygon([(-290,-100,0.02),(-170,-190,0.02),(30,-155,0.02),(100,-65,0.02),(-10,50,0.02),(-210,40,0.02)],(47,137,181),1)
        # Carretera: segmentos con normales medias, ancho constante.
        d = np.roll(self.road[:-1],-1,axis=0)-np.roll(self.road[:-1],1,axis=0)
        normal = np.column_stack((d[:,1],-d[:,0]))/np.linalg.norm(d,axis=1)[:,None]
        normal = np.vstack((normal,normal[0]))
        left, right = self.road-C.ROAD_WIDTH_M/2*normal, self.road+C.ROAD_WIDTH_M/2*normal
        for i in range(len(self.road)-1):
            self.mesh.polygon([(left[i,0],left[i,1],0.04),(left[i+1,0],left[i+1,1],0.04),
                               (right[i+1,0],right[i+1,1],0.04),(right[i,0],right[i,1],0.04)], (66,69,72),1)
            if i % 5 < 2:
                a,b = self.road[i],self.road[i+1]
                n = normal[i]*0.35
                self.mesh.polygon([(a[0]-n[0],a[1]-n[1],0.06),(b[0]-n[0],b[1]-n[1],0.06),
                                   (b[0]+n[0],b[1]+n[1],0.06),(a[0]+n[0],a[1]+n[1],0.06)],(237,224,167),1)
        # Pueblo, hangar y torre (colores saturados para no falsear el umbral gris).
        for x,y,height in [(180,100,18),(225,130,23),(260,85,15),(310,155,20),(170,170,12)]:
            self.mesh.box((x,y,height/2),(27,22,height),(201,163,107))
            self.mesh.pyramid((x,y,height),18,9,(170,71,48))
        self.mesh.box((390,260,15),(12,12,30),(211,116,58))
        self.mesh.box((390,260,31),(19,19,4),(47,94,138))
        # Arboles alejados del asfalto. Al situarlos se usa el mapa, no al detectarlo.
        for _ in range(110):
            x,y = rng.uniform(-1200,1200,2)
            if np.min(np.linalg.norm(self.road-[x,y],axis=1)) < 35:
                continue
            height = float(rng.uniform(8,19))
            self.mesh.box((x,y,height*.18),(2,2,height*.36),(113,72,38))
            self.mesh.pyramid((x,y,height*.25),height*.28,height*.75,(37,100,59))
        # Relieve SOLO decorativo lejos de la carretera; suelo de la proyeccion sigue z=0.
        for x,y,r,h in [(-1300,1400,240,160),(1000,1450,260,210),(1450,-1000,300,190),(-1400,-1400,280,140)]:
            self.mesh.pyramid((x,y,0),r,h,(127,141,105))

    @staticmethod
    def _aircraft_mesh():
        m = Mesh()
        # Ejes locales: X ala derecha, Y morro, Z arriba.
        m.box((0,0,0),(1.25,7,1.0),(233,237,238))
        m.polygon([(-.62,3.5,-.3),(.62,3.5,-.3),(0,5.0,0),(.62,3.5,.3),(-.62,3.5,.3)],(210,65,45))
        m.box((0,0.8,.1),(12.8,1.5,.22),(230,236,239))
        m.box((-5.8,.8,.24),(1.1,1.5,.08),(42,113,171))
        m.box((5.8,.8,.24),(1.1,1.5,.08),(42,113,171))
        m.box((0,-2.7,.2),(4.6,1.05,.15),(48,117,172))
        m.polygon([(0,-3.2,.1),(0,-1.5,.1),(0,-2.8,2.1)],(205,63,47))
        m.box((0,1.4,.65),(1.0,1.8,.4),(35,89,127))
        return m.arrays()

    def aircraft_vertices(self,state):
        r = np.array([math.cos(state.heading),-math.sin(state.heading),0.0])
        f = np.array([math.sin(state.heading),math.cos(state.heading),0.0])
        z = np.array([0.,0.,1.])
        bank = state.bank
        rb = r*math.cos(bank)-z*math.sin(bank)
        ub = r*math.sin(bank)+z*math.cos(bank)
        pitch = math.atan2(state.vz,max(state.airspeed,1.0))
        fp,up = f*math.cos(pitch)+ub*math.sin(pitch), -f*math.sin(pitch)+ub*math.cos(pitch)
        return self.airplane[0] @ np.column_stack((rb,fp,up)).T + np.array([state.x,state.y,state.altitude])

    def render_downward(self,state):
        camera = Camera.nadir(state)
        # NO marcador, NO trayectoria, NO icono del avion en la imagen del sensor.
        image = render_mesh(self.scene_tri,self.scene_col,self.scene_layers,camera)
        return np.asarray(image).copy()

    def render_chase(self,state,reference=None,debug=True,width=None,height=None):
        forward = np.array([math.sin(state.heading),math.cos(state.heading),0.0])
        position = np.array([state.x,state.y,state.altitude])
        eye = position-forward*48+np.array([0,0,23])
        target = position+forward*34-np.array([0,0,5])
        camera = Camera.look_at(eye,target,width or C.CHASE_WIDTH,height or C.CHASE_HEIGHT)
        tris = np.concatenate((self.scene_tri,self.aircraft_vertices(state)))
        cols = np.concatenate((self.scene_col,self.airplane[1]))
        layers = np.concatenate((self.scene_layers,self.airplane[2]))
        image = render_mesh(tris,cols,layers,camera)
        if debug:
            draw = ImageDraw.Draw(image)
            if len(self.trail)>1:
                pts,depth = camera.project(np.array(self.trail))
                for i in range(len(pts)-1):
                    if min(depth[i:i+2])>1 and max(abs(pts[i]))<10000 and max(abs(pts[i+1]))<10000:
                        draw.line([tuple(pts[i]),tuple(pts[i+1])],fill=(242,197,69),width=2)
            if reference is not None and reference.target is not None:
                x,y = reference.target
                pts,depth = camera.project([[x,y,1],[x,y,state.altitude]])
                if np.all(depth>1) and np.all(np.abs(pts)<10000):
                    draw.line([tuple(pts[0]),tuple(pts[1])],fill=(255,182,42),width=2)
                    u,v = pts[1]
                    draw.ellipse((u-5,v-5,u+5,v+5),outline=(255,207,65),width=3)
        return image

    def add_trail(self,state):
        point = (state.x,state.y,state.altitude)
        if not self.trail or np.linalg.norm(np.array(point)-self.trail[-1])>2:
            self.trail.append(point)
            if len(self.trail)>200:
                self.trail.pop(0)
