"""Acumula progresivamente evidencia LiDAR y reevalúa hipótesis cada N barridos.
Prototipo en Python/NumPy: no controla el vehículo ni equivale a un coste del P4."""
import argparse, json, time, tracemalloc
from collections import Counter
from pathlib import Path
import numpy as np
from segment_parking_cloud import build_hypothesis

def run(data, every):
    scans=data.get('scans',[])
    if not data.get('camera'):
        return {'status':'sin_deteccion_magenta','snapshots':[]}
    cells=Counter();raw_count=0;records=[];insertion_ms=[]
    forward=float(data.get('lidar_forward_cm',10))
    tracemalloc.start()
    for index,scan in enumerate(scans,1):
        start=time.perf_counter();p=scan['estimated_current'];rays=np.asarray(scan['rays'],dtype=float)
        if len(rays):
            rays=rays[(rays[:,3]>0)&(rays[:,2]>0)&(rays[:,2]<=200)]
            theta=np.deg2rad(p['theta']);a=np.deg2rad(rays[:,1])+theta
            xy=np.column_stack((p['x']+np.sin(theta)*forward+np.sin(a)*rays[:,2],p['z']+np.cos(theta)*forward+np.cos(a)*rays[:,2]))
            grid,counts=np.unique(np.rint(xy).astype(int),axis=0,return_counts=True)
            cells.update({(int(q[0]),int(q[1])):int(n) for q,n in zip(grid,counts)})
            raw_count+=len(rays)
        insertion_ms.append((time.perf_counter()-start)*1000)
        if index%every and index!=len(scans):continue
        points=np.asarray(sorted(k for k,v in cells.items() if v>=2),dtype=float).reshape((-1,2))
        start=time.perf_counter();h=build_hypothesis(data,precomputed_cloud=(points,raw_count))
        elapsed=(time.perf_counter()-start)*1000;current,peak=tracemalloc.get_traced_memory()
        records.append({'scans':index,'elapsed_sim_s':round(scan['scan_end_s']-data['trigger_s'],3),
                        'segmentation_wall_ms_with_tracemalloc':round(elapsed,3),'python_current_bytes':current,
                        'python_peak_bytes':peak,'total_cells':len(cells),'supported_cells':len(points),'hypothesis':h})
    tracemalloc.stop();insertion_ms.sort()
    q=lambda p:round(insertion_ms[max(0,int(np.ceil(p*len(insertion_ms)))-1)],3) if insertion_ms else None
    return {'camera_trigger_s':data['trigger_s'],'scans_total':len(scans),'every_scans':every,
            'insertion_wall_ms_with_tracemalloc':{'p50':q(.5),'p95':q(.95),'max':q(1)},'snapshots':records}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('input',type=Path);ap.add_argument('--every',type=int,default=10);ap.add_argument('--output',type=Path);a=ap.parse_args()
    if a.every<=0:ap.error('--every debe ser positivo')
    result=run(json.loads(a.input.read_text(encoding='utf-8')),a.every)
    text=json.dumps(result,ensure_ascii=True)
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text+'\n',encoding='utf-8')
    print(text)
if __name__=='__main__':main()
