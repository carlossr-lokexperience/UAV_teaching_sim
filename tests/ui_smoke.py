"""Smoke test de GUI real, necesita pantalla (o Xvfb en Linux).
No forma parte de unittest discover. Ejecutar: python tests/ui_smoke.py
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import config as C
from main import Application

app=Application(C.Settings())
errors=[]

def check():
    try:
        app.root.update()
        assert app.root.winfo_height() <= app.root.winfo_screenheight()-20
        # Entrada de texto y validacion real de la GUI.
        app.entry.insert(0,'sube a 120 metros')
        app.submit()
        assert app.sim.settings.altitude_m==120
        app.entry.insert(0,'sigue la carretera con camara')
        app.submit()
        assert app.sim.settings.source=='CAMERA'
        # Eventos de teclado: manual, pulsar, soltar y no pilotar al escribir.
        app.root.focus_force(); app.root.update()
        app.root.event_generate('<KeyPress-a>'); app.root.event_generate('<KeyRelease-a>')
        assert app.sim.settings.mode=='MANUAL'
        app.root.event_generate('<KeyPress-Right>'); app.pilot_keys(.1)
        assert app.sim.settings.manual_bank_rad>0
        app.root.event_generate('<KeyRelease-Right>'); app.pilot_keys(.1)
        assert app.sim.settings.manual_bank_rad==0
        before=app.sim.settings.mode
        app.entry.focus_force(); app.root.update()
        app.root.event_generate('<KeyPress-f>')
        assert app.sim.settings.mode==before
        app.keys.clear()
        app.toggle_pause(); assert app.paused
        app.toggle_pause(); assert not app.paused
        app.capture()
        app.reset()
        assert app.sim.settings.mode==C.FLIGHT_MODE
        print('GUI OK: tamano, texto, CAMERA, teclado, liberacion, foco, pausa, captura, reinicio',flush=True)
    except Exception as exc:
        errors.append(exc)
    finally:
        app.close()
app.root.after(600,check)
app.run()
if errors:
    raise errors[0]
