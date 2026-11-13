"""Validacion larga y reproducible. No necesita pantalla; CAMERA renderiza imagenes reales.
Ejecuta desde la raiz: python tests/validate_flights.py
"""
import json
import sys
import time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from main import Simulation
from config import Settings, PHYSICS_DT

def run():
    results=[]
    duration=120.0
    for source in ['MAP','CAMERA']:
        for guidance in ['PURE_PURSUIT','VECTOR_FIELD']:
            settings=Settings(source=source,guidance=guidance)
            sim=Simulation(settings,log=False,camera=(source=='CAMERA'))
            start=time.perf_counter()
            for i in range(int(duration/PHYSICS_DT)):
                sim.step()
                if i % 5000 == 0:
                    print(source,guidance,'t=',round(sim.aircraft.state.time),flush=True)
            errors=np.array([row[1] for row in sim.metrics])
            modes=set(row[2] for row in sim.metrics)
            result=dict(source=source,guidance=guidance,seconds=duration,
                        rms_m=float(np.sqrt(np.mean(errors**2))),max_m=float(errors.max()),
                        maintained_follow=(modes=={'FOLLOW_ROAD'}),
                        wall_seconds=time.perf_counter()-start)
            assert result['maintained_follow'],result
            assert result['max_m']<8,result
            results.append(result)
            print(json.dumps(result),flush=True)
            sim.close()
    out=Path(__file__).resolve().parents[1]/'docs'/'flight_results.json'
    out.write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__': run()
