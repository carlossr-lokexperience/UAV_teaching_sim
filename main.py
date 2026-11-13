"""PUNTO DE ENTRADA. Lee Simulation.step(): es el recorrido completo de la informacion.
GUI separada del bucle logico; --headless prueba los mismos algoritmos sin Tk.
No hace instalaciones ni busca procesos, no toca PX4 ni usa puertos de vuelo.
"""
import argparse
import csv
import json
import math
import queue
import sys
import threading
import time
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
import config as C
from aircraft import Aircraft, clip
from navigation import Navigation
from perception import detect_road, PathObservation, cv2
from guidance import project_on_path
from control import Controller
from autopilot import Autopilot
from commands import parse_rules, validate_command, call_llm
from world import World

ROOT = Path(__file__).resolve().parent

class Simulation:
    """Unica instancia de avion. Todo el estado esta aqui, nunca en variables globales."""
    def __init__(self, settings=None, log=True, camera=True):
        self.settings = (settings or C.Settings()).validate()
        self.world = World()
        self.aircraft = Aircraft(self.world.initial_state())
        self.navigation = Navigation()
        self.autopilot = Autopilot(self.settings)
        self.controller = Controller()
        self.estimate = self.aircraft.get_true_state()
        self.settings.heading_rad = self.estimate.heading
        self.observation = PathObservation()
        self.rgb = None
        self.camera_frame_time = -1.0
        self.reference = None
        self.next_camera = 0.0
        self.camera_enabled = camera
        self.next_log = 0.0
        self.logfile = None
        self.writer = None
        self.metrics = []
        if log:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.run_dir = ROOT / 'logs' / stamp
            self.run_dir.mkdir(parents=True, exist_ok=True)
            self.logfile = (self.run_dir/'vuelo.csv').open('w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.logfile)
            self.writer.writerow(['t_s','true_x_m','true_y_m','est_x_m','est_y_m','alt_m','airspeed_mps',
                                  'groundspeed_mps','heading_deg','bank_deg','bank_request_deg','bank_limited_deg',
                                  'mode','source','guidance','lookahead_m','estimated_error_m','true_error_m',
                                  'vision_confidence','vision_valid','fix_age_s','status'])
            (self.run_dir/'config.json').write_text(json.dumps(asdict(self.settings),indent=2),encoding='utf-8')
        else:
            self.run_dir = None

    def step(self, dt=C.PHYSICS_DT):
        truth = self.aircraft.get_true_state()
        # 1. NAVEGACION: solo este modulo simula sensores a partir de la verdad.
        self.estimate = self.navigation.estimate_state(truth, dt, self.settings.navigation, self.settings.gnss)
        # 2. CAMARA: render del mundo real; percepcion proyecta con la POSE ESTIMADA.
        if self.camera_enabled and truth.time >= self.next_camera-1e-8:
            self.rgb = self.world.render_downward(truth)
            self.camera_frame_time = truth.time
            if C.PERCEPTION_ENABLED:
                self.observation = detect_road(self.rgb, replace(self.estimate))
            self.next_camera = truth.time + 1.0/C.CAMERA_HZ
        # 3. GUIADO: CAMERA nunca recibe world.road, ni siquiera como respaldo.
        path = self.world.get_road_map() if self.settings.source == 'MAP' else None
        self.reference = self.autopilot.update(self.estimate, path, self.observation)
        # 4. CONTROL -> 5. PLANTA. El LLM no participa en este lazo.
        actuation = self.controller.update(self.estimate, self.reference, dt, self.settings)
        self.aircraft.update_physics(actuation, dt, (self.settings.wind_e,self.settings.wind_n))
        self.world.add_trail(self.aircraft.state)
        # 6. EVALUACION, nunca realimentada al guiado: error contra la carretera real.
        if truth.time >= self.next_log-1e-8:
            true_error = abs(project_on_path((truth.x,truth.y),self.world.road)[4])
            self.metrics.append((truth.time,true_error,self.settings.mode,self.observation.valid))
            if len(self.metrics)>100000:
                self.metrics.pop(0)
            if self.writer:
                self.writer.writerow([round(truth.time,4),truth.x,truth.y,self.estimate.x,self.estimate.y,
                    truth.altitude,truth.airspeed,truth.ground_speed,math.degrees(truth.heading)%360,math.degrees(truth.bank),
                    math.degrees(self.reference.bank_rad),math.degrees(self.controller.bank_reference),
                    self.settings.mode,self.settings.source,self.settings.guidance,self.settings.lookahead_m,
                    self.reference.cross_track_m,true_error,self.observation.confidence,self.observation.valid,
                    self.estimate.fix_age,self.reference.status])
                self.logfile.flush()
            self.next_log = truth.time+1.0/C.LOG_HZ

    def apply_command(self, command):
        old_mode,old_source = self.settings.mode,self.settings.source
        candidate = validate_command(command,self.settings,self.estimate)
        # Misma instancia, para mantener referencias de autopilot y GUI.
        self.settings.__dict__.update(asdict(candidate))
        if old_source != candidate.source or old_mode != candidate.mode:
            self.autopilot.reset_tracking()
            self.controller.reset()
        self.autopilot.status = 'ORDEN ACEPTADA'
        if self.run_dir:
            with (self.run_dir/'ordenes.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps({'time':self.aircraft.state.time,'command':command,'applied':asdict(candidate)})+'\n')

    def save_views(self, folder):
        folder = Path(folder)
        folder.mkdir(parents=True,exist_ok=True)
        chase = self.world.render_chase(self.aircraft.state,self.reference,C.SHOW_DEBUG)
        chase.save(folder/'exterior.png')
        if self.rgb is not None:
            Image.fromarray(self.rgb).save(folder/'camara.png')
        if self.observation.overlay is not None:
            Image.fromarray(self.observation.overlay).save(folder/'percepcion.png')
        if self.observation.mask is not None:
            Image.fromarray(self.observation.mask).save(folder/'mascara.png')
        return chase

    def close(self):
        if self.logfile:
            self.logfile.close()
            self.logfile = None

class Application:
    """La GUI no contiene matematicas de guiado; solo botones, imagenes y comandos."""
    def __init__(self, settings, duration=None, screenshot=None):
        import tkinter as tk
        from tkinter import ttk
        from PIL import ImageTk
        self.tk,self.ttk,self.ImageTk = tk,ttk,ImageTk
        self.root = tk.Tk()
        self.root.title('UAV CLASS | Carretera, vision y lenguaje natural')
        self.root.configure(bg='#101d2b')
        self.view_height = max(250, min(C.CHASE_HEIGHT, self.root.winfo_screenheight()-420))
        self.camera_display = max(105, (self.view_height-25)//2)
        self.sim = Simulation(settings)
        self.duration,self.screenshot_path = duration,screenshot
        self.paused,self.closed = False,False
        self.keys = set()
        self.last_wall = time.perf_counter()
        self.accumulator = 0.0
        self.job_queue = queue.Queue()
        self.job_busy = False
        self.generation = 0
        self.last_camera_time = -1
        self.frames = 0
        self.window_seconds = 0
        self.fps = 0.0
        self.variables = {}
        self.create_ui()
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.bind('<KeyPress>',self.key_down)
        self.root.bind('<KeyRelease>',self.key_up)
        self.root.bind('<FocusOut>',lambda event:self.keys.clear())
        self.root.after(20,self.update)
        print('UAV_CLASS iniciado. No hay despegue: el avion ya esta en vuelo.')
        print('Logs:',self.sim.run_dir)

    def label(self,parent,text,size=10,fg='#d9e5eb',**kwargs):
        widget=self.tk.Label(parent,text=text,font=('Arial',size),bg='#101d2b',fg=fg,**kwargs)
        return widget

    def create_ui(self):
        tk,ttk=self.tk,self.ttk
        style=ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('TFrame',background='#101d2b')
        style.configure('TButton',padding=(8,5))
        outer=ttk.Frame(self.root,padding=12); outer.pack(fill='both',expand=True)
        self.label(outer,'UAV CLASS   /   LABORATORIO DE GUIADO',17,fg='#65d7be',anchor='w').pack(fill='x')
        self.label(outer,'Simulacion 3D educativa | Un avion | Sin PX4 | Arranque en vuelo',10,anchor='w').pack(fill='x',pady=(0,8))
        images=ttk.Frame(outer); images.pack(fill='x')
        left=ttk.Frame(images); left.pack(side='left')
        self.label(left,'CAMARA EXTERIOR  |  amarillo: trayectoria y objetivo',10,anchor='w').pack(fill='x')
        self.chase_label=tk.Label(left,bg='#263849',bd=0); self.chase_label.pack()
        self.chase_label.bind('<Button-1>',lambda event:self.root.focus_set())
        right=ttk.Frame(images,padding=(10,0,0,0)); right.pack(side='left',fill='y')
        self.label(right,'CAMARA INFERIOR (RGB)',10,anchor='w').pack(fill='x')
        self.nadir_label=tk.Label(right,bg='#263849',bd=0); self.nadir_label.pack()
        self.label(right,'OPENCV / LINEA DETECTADA',10,anchor='w').pack(fill='x',pady=(5,0))
        self.vision_label=tk.Label(right,bg='#263849',bd=0); self.vision_label.pack()
        self.hud=self.label(outer,'',11,anchor='w',justify='left'); self.hud.pack(fill='x',pady=7)
        row=ttk.Frame(outer); row.pack(fill='x')
        choices=[('mode','Modo',['FOLLOW_ROAD','HEADING','MANUAL']),('source','Carretera',['MAP','CAMERA']),
                 ('guidance','Guiado',['PURE_PURSUIT','VECTOR_FIELD']),('navigation','Navegacion',['PERFECT','NOISY','FILTERED']),
                 ('controller','Control',['P','PID']),('parser','Ordenes',['RULES','LLM'])]
        for field,label,values in choices:
            cell=ttk.Frame(row,padding=(0,0,10,0)); cell.pack(side='left')
            self.label(cell,label,9,anchor='w').pack(fill='x')
            var=tk.StringVar(value=getattr(self.sim.settings,field)); self.variables[field]=var
            box=ttk.Combobox(cell,textvariable=var,values=values,state='readonly',width=max(len(v) for v in values)+3)
            box.pack(); box.bind('<<ComboboxSelected>>',lambda event,f=field:self.choose(f))
        sliders=ttk.Frame(outer); sliders.pack(fill='x',pady=7)
        for field,title,lo,hi in [('lookahead_m','Anticipacion (m)',15,150),('speed_mps','Velocidad (m/s)',12,32),
                                 ('bank_limit_deg','Alabeo max. (grados)',5,60),('altitude_m','Altura (m)',30,220)]:
            cell=ttk.Frame(sliders,padding=(0,0,12,0)); cell.pack(side='left',fill='x',expand=True)
            var=tk.DoubleVar(value=getattr(self.sim.settings,field)); self.variables[field]=var
            self.label(cell,title,9,anchor='w').pack(fill='x')
            scale=tk.Scale(cell,from_=lo,to=hi,resolution=1,orient='horizontal',variable=var,
                           length=230,command=lambda value,f=field:self.set_number(f,value),
                           bg='#101d2b',fg='#e4edf2',troughcolor='#33465a',highlightthickness=0)
            scale.pack(fill='x')
        commandrow=ttk.Frame(outer); commandrow.pack(fill='x')
        self.entry=ttk.Entry(commandrow,font=('Arial',11)); self.entry.pack(side='left',fill='x',expand=True)
        self.entry.bind('<Return>',lambda event:self.submit())
        ttk.Button(commandrow,text='Enviar orden',command=self.submit).pack(side='left',padx=5)
        ttk.Button(commandrow,text='Pausa',command=self.toggle_pause).pack(side='left',padx=3)
        ttk.Button(commandrow,text='Reiniciar',command=self.reset).pack(side='left',padx=3)
        ttk.Button(commandrow,text='Captura',command=self.capture).pack(side='left',padx=3)
        self.label(outer,'Ej.: sigue la carretera con camara | sube a 120 metros | gira 30 grados a la derecha | lookahead 80',9,anchor='w').pack(fill='x',pady=(6,0))
        self.label(outer,'F carretera  M mapa  C camara  G guiado  H rumbo  A manual  N GNSS  Espacio pausa  R reiniciar | Flechas: giro/altura; W/S: velocidad',9,anchor='w').pack(fill='x')
        self.status=self.label(outer,'RULES funciona sin IA. Para un LLM real configura el proveedor/modelo en config.py.',10,fg='#f5c55b',anchor='w',wraplength=1040,justify='left')
        self.status.pack(fill='x',pady=(5,0))
        # Primera imagen antes del mainloop: no hay paneles vacios durante el arranque.
        self.sim.step()
        self.refresh_images()
        self.root.resizable(False,False)

    def choose(self,field):
        value=self.variables[field].get()
        try:
            if field in ('mode','source','guidance'):
                self.sim.apply_command({field:value})
            else:
                setattr(self.sim.settings,field,value)
                self.sim.settings.validate()
            self.status.config(text=field+' = '+value)
        except ValueError as exc:
            self.status.config(text=str(exc))
        self.sync_widgets()
        self.root.focus_set()

    def set_number(self,field,value):
        setattr(self.sim.settings,field,float(value))

    def sync_widgets(self):
        for field,var in self.variables.items():
            value=getattr(self.sim.settings,field)
            if var.get()!=value:
                var.set(value)

    def submit(self):
        text=self.entry.get().strip()
        if not text:
            return 'break'
        try:
            if self.sim.settings.parser=='RULES':
                command=parse_rules(text)
                self.sim.apply_command(command)
                self.status.config(text='Aceptado: '+json.dumps(command,ensure_ascii=False))
                self.entry.delete(0,'end')
                self.sync_widgets()
            elif self.job_busy:
                self.status.config(text='Ya hay una peticion al LLM. El vuelo sigue sin bloquearse.')
            else:
                self.job_busy=True
                generation=self.generation
                snapshot=replace(self.sim.settings)
                def worker():
                    try:
                        result=call_llm(text,snapshot); error=None
                    except Exception as exc:
                        result=None; error=str(exc)
                    self.job_queue.put((generation,result,error))
                threading.Thread(target=worker,daemon=True).start()
                self.status.config(text='LLM: interpretando. El guiado continua de forma independiente.')
        except (ValueError,json.JSONDecodeError) as exc:
            self.status.config(text='No se aplica: '+str(exc))
        return 'break'

    def toggle_pause(self):
        self.paused=not self.paused
        self.keys.clear()
        self.status.config(text='PAUSA' if self.paused else 'Simulacion en marcha')
        self.root.focus_set()

    def reset(self):
        self.sim.close()
        self.sim=Simulation()
        self.generation+=1
        self.job_busy=False
        self.keys.clear()
        self.last_camera_time=-1
        self.accumulator=0
        self.paused=False
        self.sim.step()
        self.sync_widgets()
        self.status.config(text='Reinicio con config.py. Los registros anteriores se conservan.')
        self.root.focus_set()

    def capture(self):
        folder=self.sim.run_dir / ('captura_%06d'%int(self.sim.aircraft.state.time*10))
        self.sim.save_views(folder)
        self.status.config(text='Guardado: '+str(folder))
        self.root.focus_set()

    def key_down(self,event):
        # Nunca pilotar mientras se escribe en un cuadro de texto o combobox.
        if self.root.focus_get()!=self.root:
            if event.keysym=='Escape':
                self.root.focus_set()
            return
        key=event.keysym.lower()
        if key in self.keys:
            return
        self.keys.add(key)
        s=self.sim.settings
        try:
            if key=='space': self.toggle_pause()
            elif key=='r': self.reset()
            elif key=='f': self.sim.apply_command({'mode':'FOLLOW_ROAD'})
            elif key=='h': self.sim.apply_command({'mode':'HEADING'})
            elif key=='a': self.sim.apply_command({'mode':'MANUAL'})
            elif key=='m': self.sim.apply_command({'source':'MAP','mode':'FOLLOW_ROAD'})
            elif key=='c': self.sim.apply_command({'source':'CAMERA','mode':'FOLLOW_ROAD'})
            elif key=='g': self.sim.apply_command({'guidance':'VECTOR_FIELD' if s.guidance=='PURE_PURSUIT' else 'PURE_PURSUIT'})
            elif key=='n': s.gnss=not s.gnss
            elif key in ('return','kp_enter'): self.entry.focus_set(); self.keys.clear()
        except ValueError as exc:
            self.status.config(text=str(exc))
        self.sync_widgets()

    def key_up(self,event):
        self.keys.discard(event.keysym.lower())

    def pilot_keys(self,dt):
        s=self.sim.settings
        turn=int('right' in self.keys)-int('left' in self.keys)
        if s.mode=='MANUAL':
            s.manual_bank_rad=turn*math.radians(s.bank_limit_deg)*0.8
        elif turn:
            if s.mode!='HEADING':
                self.sim.autopilot.set_mode('HEADING',self.sim.estimate)
            s.heading_rad += turn*math.radians(25)*dt
        s.altitude_m=clip(s.altitude_m+(int('up' in self.keys)-int('down' in self.keys))*12*dt,C.MIN_ALTITUDE_M,C.MAX_ALTITUDE_M)
        s.speed_mps=clip(s.speed_mps+(int('w' in self.keys)-int('s' in self.keys))*3*dt,C.MIN_SPEED_MPS,C.MAX_SPEED_MPS)

    def refresh_images(self):
        from PIL import ImageTk
        image=self.sim.world.render_chase(self.sim.aircraft.state,self.sim.reference,C.SHOW_DEBUG,height=self.view_height)
        self.chase_photo=ImageTk.PhotoImage(image)
        self.chase_label.configure(image=self.chase_photo)
        if self.sim.rgb is not None and self.sim.camera_frame_time!=self.last_camera_time:
            raw=Image.fromarray(self.sim.rgb).resize((self.camera_display,self.camera_display),Image.Resampling.BILINEAR)
            self.nadir_photo=ImageTk.PhotoImage(raw); self.nadir_label.configure(image=self.nadir_photo)
            visual=self.sim.observation.overlay
            if visual is None:
                visual=self.sim.rgb
            # Opcion de mascara: cambiar esta linea por Image.fromarray(observation.mask).
            processed=Image.fromarray(visual).resize((self.camera_display,self.camera_display),Image.Resampling.BILINEAR)
            self.vision_photo=ImageTk.PhotoImage(processed); self.vision_label.configure(image=self.vision_photo)
            self.last_camera_time=self.sim.camera_frame_time

    def update(self):
        if self.closed:
            return
        start=time.perf_counter()
        elapsed=min(0.25,start-self.last_wall)
        self.last_wall=start
        while not self.job_queue.empty():
            generation,command,error=self.job_queue.get_nowait()
            if generation!=self.generation:
                continue
            self.job_busy=False
            if error:
                self.status.config(text='LLM: '+error)
            else:
                try:
                    self.sim.apply_command(command)
                    self.status.config(text='LLM -> '+json.dumps(command,ensure_ascii=False))
                    self.entry.delete(0,'end')
                except ValueError as exc:
                    self.status.config(text='LLM rechazado: '+str(exc))
        if not self.paused:
            self.accumulator+=elapsed
            # A lo sumo 10 pasos por frame: evita una espiral de carga en equipos lentos.
            steps=0
            while self.accumulator>=C.PHYSICS_DT and steps<10:
                self.pilot_keys(C.PHYSICS_DT)
                self.sim.step()
                self.accumulator-=C.PHYSICS_DT
                steps+=1
            self.accumulator=min(self.accumulator,0.2)
        self.refresh_images()
        s=self.sim.aircraft.state
        cfg=self.sim.settings
        ref=self.sim.reference
        self.frames+=1; self.window_seconds+=elapsed
        if self.window_seconds>=1:
            self.fps=self.frames/self.window_seconds
            self.frames=0; self.window_seconds=0
        error=ref.cross_track_m if ref else float('nan')
        self.hud.config(text=('ALT %.1f / %.0f m   V %.1f / %.0f m/s   RUMBO %03.0f   ALABEO %+.1f / limite %.0f grados   FPS %.0f\n'
            '%s  |  %s  |  error lateral %.1f m  |  vision %.0f%%  |  GNSS %s%s')%
            (s.altitude,cfg.altitude_m,s.airspeed,cfg.speed_mps,math.degrees(s.heading)%360,math.degrees(s.bank),cfg.bank_limit_deg,self.fps,
             cfg.mode,ref.status if ref else '',error,100*self.sim.observation.confidence,'SI' if cfg.gnss else 'NO','  [PAUSA]' if self.paused else ''))
        self.sync_widgets()
        if self.duration is not None and s.time>=self.duration:
            if self.screenshot_path:
                self.root.update_idletasks()
                from PIL import ImageGrab
                x,y=self.root.winfo_rootx(),self.root.winfo_rooty()
                ImageGrab.grab(bbox=(x,y,x+self.root.winfo_width(),y+self.root.winfo_height())).save(self.screenshot_path)
            self.close(); return
        used=time.perf_counter()-start
        self.root.after(max(1,int(1000*(1.0/C.GUI_FPS-used))),self.update)

    def close(self):
        if self.closed: return
        self.closed=True
        self.keys.clear()
        self.sim.close()
        self.root.destroy()

    def run(self):
        try:
            self.root.focus_force()
            self.root.mainloop()
        finally:
            self.sim.close()


def main():
    parser=argparse.ArgumentParser(description='UAV_CLASS: seguimiento de carretera 3D en Python')
    parser.add_argument('--source',choices=['MAP','CAMERA'])
    parser.add_argument('--guidance',choices=['PURE_PURSUIT','VECTOR_FIELD'])
    parser.add_argument('--headless',action='store_true',help='Prueba sin ventana, misma simulacion')
    parser.add_argument('--seconds',type=float,default=None,help='Duracion simulada; GUI cierra al finalizar')
    parser.add_argument('--screenshot',type=str,help='Captura de GUI al terminar --seconds')
    parser.add_argument('--output',type=str,default=None,help='Directorio de capturas del modo headless')
    args=parser.parse_args()
    if args.seconds is not None and args.seconds < C.PHYSICS_DT:
        parser.error("--seconds debe ser al menos %.2f" % C.PHYSICS_DT)
    settings=C.Settings()
    if args.source: settings.source=args.source
    if args.guidance: settings.guidance=args.guidance
    if settings.source=='CAMERA' and (cv2 is None or not C.PERCEPTION_ENABLED):
        parser.error('CAMERA necesita OpenCV y PERCEPTION_ENABLED=True')
    if args.headless:
        sim=Simulation(settings,camera=True)
        duration=args.seconds if args.seconds is not None else 20.0
        try:
            for _ in range(int(duration/C.PHYSICS_DT)):
                sim.step()
            errors=np.array([m[1] for m in sim.metrics])
            summary={'seconds':sim.aircraft.state.time,'mode':sim.settings.mode,'source':sim.settings.source,
                     'guidance':sim.settings.guidance,'cross_track_rms_m':float(np.sqrt(np.mean(errors**2))),
                     'cross_track_max_m':float(np.max(errors)),'vision_last_valid':bool(sim.observation.valid),
                     'vision_confidence':sim.observation.confidence}
            if args.output: sim.save_views(args.output)
            if sim.run_dir: (sim.run_dir/'resumen.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            print(json.dumps(summary,indent=2))
        finally:
            sim.close()
    else:
        try:
            Application(settings,args.seconds,args.screenshot).run()
        except ImportError as exc:
            raise SystemExit('Dependencia ausente: %s. Instala requirements.txt; en Linux Tk: python3-tk.'%exc)

if __name__=='__main__':
    main()
