"""Extrae hipótesis geométricas de un barrido guardado, sin geometría verdadera.
Prototipo offline para investigación: numpy requerido. No controla el vehículo ni
confirma un parqueo; separa evidencia visible de una hipótesis de pareja.
Uso: python simulator/tools/segment_parking_cloud.py archivo.json
"""
import argparse
import json
import math
import os
import sys
import numpy as np
from pathlib import Path
def cloud_from_scans(data):
    clouds = []
    forward = float(data.get('lidar_forward_cm', 10))
    for scan in data['scans']:
        p = scan['estimated_current']
        rays = np.asarray(scan['rays'], dtype=float)
        if not len(rays):
            continue
        rays = rays[(rays[:, 3] > 0) & (rays[:, 2] > 0) & (rays[:, 2] <= 200)]
        if not len(rays):
            continue
        theta = math.radians(p['theta'])
        a = np.deg2rad(rays[:, 1]) + theta
        clouds.append(np.column_stack((p['x'] + math.sin(theta)*forward + np.sin(a)*rays[:, 2],
                                       p['z'] + math.cos(theta)*forward + np.cos(a)*rays[:, 2])))
    if not clouds:
        return np.empty((0,2)), 0
    raw = np.concatenate(clouds)
    # Colapsar a celdas de 1 cm y exigir al menos dos retornos por celda.
    grid, counts = np.unique(np.rint(raw).astype(int), axis=0, return_counts=True)
    return grid[counts >= 2].astype(float), len(raw)
def build_hypothesis(data, lot_length_cm=39.0, limiter_width_cm=20.0, limiter_thickness_cm=2.0, precomputed_cloud=None):
    if not all(math.isfinite(x) and x > 0 for x in (lot_length_cm, limiter_width_cm, limiter_thickness_cm)):
        raise ValueError("Las dimensiones del cajón deben ser finitas y positivas")
    camera = data.get('camera')
    if not camera:
        return {'status':'sin_deteccion_magenta', 'reason':'la cámara no proporcionó una detección magenta'}
    if not camera.get('pose'):
        return {'status':'sin_pose_de_camara', 'reason':'no hay pose estimada al capturar la detección'}
    points, raw_count = precomputed_cloud if precomputed_cloud is not None else cloud_from_scans(data)
    if not len(points):
        return {'status':'sin_puntos'}
    pose, obj = camera['pose'], camera['object']
    th = math.radians(pose['theta']); st, ct = math.sin(th), math.cos(th)
    origin = np.array([pose['x'] + st*camera['camera_forward_cm'] + ct*camera['camera_lateral_cm'],
                       pose['z'] + ct*camera['camera_forward_cm'] - st*camera['camera_lateral_cm']])
    d = points - origin
    right, front = d[:,0]*ct-d[:,1]*st, d[:,0]*st+d[:,1]*ct
    angle = np.degrees(np.arctan2(right, front))
    focal = camera['focal_px']
    # Como hay una caja recortada o un cálculo de distancia incierto, se usa como
    # región aproximada, no se trata el borde de la caja como límite exacto.
    lo = math.degrees(math.atan((obj['minX']-80)/focal)) - 8
    hi = math.degrees(math.atan((obj['maxX']+1-80)/focal)) + 8
    depth = obj['distance']
    region = points[(front > 0) & (angle >= lo) & (angle <= hi) &
                    (np.hypot(right,front) >= max(5, depth*.3-12)) &
                    (np.hypot(right,front) <= depth*2+45)]
    if len(region)<20:
        return {'status':'evidencia_insuficiente','region_cells':len(region),'raw_returns':raw_count}
    # Hough sobre celdas en la región visual; se buscan paredes largas, sin
    # conocer el ángulo ni la posición real del cajón.
    walls=[]
    for deg in range(0,180,2):
        a=math.radians(deg); n=np.array([math.cos(a),math.sin(a)]); t=np.array([-n[1],n[0]])
        bins=np.rint(region @ n).astype(int)
        values,counts=np.unique(bins,return_counts=True)
        for bin_,count in zip(values,counts):
            if count<20:continue
            mask=np.abs(region @ n-bin_)<=1.25; x=region[mask]; along=x@t
            span=float(np.ptp(along))
            if len(x)>=20 and span>=30:
                walls.append((len(x),span,n,t,float(np.mean(x@n))))
    if not walls:
        return {'status':'sin_pared_suficiente','region_cells':len(region),'raw_returns':raw_count}
    walls.sort(key=lambda x:(x[0],x[1]),reverse=True)
    if os.environ.get('DEBUG_SEGMENTATION'):
        print(json.dumps([{'support':w[0],'span':round(w[1],2),'angle':round(math.degrees(math.atan2(w[2][1],w[2][0])),2),'offset':round(w[4],2)} for w in walls[:12]]),file=sys.stderr)
    best_support=walls[0][0]
    close_walls=[w for w in walls if w[0]>=0.95*best_support]
    # Si orientaciones muy parecidas tienen respaldo casi igual, una sola pared
    # seleccionada no refleja la incertidumbre; registrar ambigüedad.
    best_normal=walls[0][2]
    competing=[w for w in close_walls if math.degrees(math.acos(min(1.0,abs(float(np.dot(best_normal,w[2]))))))>=1.9]
    _,wall_span,n,t,r=walls[0]
    # Orientar la normal hacia el coche.
    if np.dot(origin,n)<r:n=-n;t=-t;r=-r
    dist=points@n-r; along=points@t
    # Las caras del tope protruyen desde la pared: buscar franjas estrechas en s
    # con retornos a distintas distancias normales, hasta el ancho conocido de 20 cm.
    candidates=points[(dist>=2.5)&(dist<=limiter_width_cm+2.5)]
    if not len(candidates):
        return {'status':'sin_topes_suficientes','wall':{'normal':n.tolist(),'offset_cm':r,'support_span_cm':wall_span},'raw_returns':raw_count}
    cd=candidates@n-r; cs=candidates@t
    # Espacio útil alrededor de la observación: no se conoce aún la posición exacta.
    region_s=region@t; low,high=float(region_s.min()-30),float(region_s.max()+50)
    scores=[]
    for s in np.arange(math.floor(low),math.ceil(high)+1):
        group=np.abs(cs-s)<=2.0
        vals=cd[group]
        if len(vals)>=5 and np.ptp(vals)>=6:
            scores.append((float(s),len(vals),float(np.ptp(vals))))
    groups=[]
    for row in scores:
        if not groups or row[0]-groups[-1][-1][0]>1.01:groups.append([])
        groups[-1].append(row)
    markers=[]
    for group in groups:
        # Un pico muy ancho es una región ambigua, no dos topes inventados.
        if group[-1][0]-group[0][0]>13:continue
        s=sum(x[0]*x[1] for x in group)/sum(x[1] for x in group)
        vals=[x for x in group if abs(x[0]-s)<=2]
        markers.append({'along_cm':s,'support':max(x[1] for x in vals),'extent_cm':max(x[2] for x in vals)})
    choices=[]
    for i,a in enumerate(markers):
        for b in markers[i+1:]:
            spacing=abs(a['along_cm']-b['along_cm'])
            # Geometría provisional del chasis de 26 cm: longitud del cajón 39 cm,
            # más 2 cm de tope. Se trata de un parámetro de sede, no una coordenada.
            if abs(spacing-(lot_length_cm+limiter_thickness_cm))<=6:choices.append((a['support']+b['support'],a,b,spacing))
    def marker(m):
        foot=n*r+t*m['along_cm'];end=foot+n*limiter_width_cm
        evidence=cd[np.abs(cs-m['along_cm'])<=2]
        low,high=np.percentile(evidence,[5,95])
        return {'wall_end':foot.tolist(),'lane_end_nominal':end.tolist(),
                'evidence_start':(foot+n*low).tolist(),'evidence_end':(foot+n*high).tolist(),
                'support_cells':m['support'],'visible_extent_cm':m['extent_cm']}
    wall={'normal':n.tolist(),'offset_cm':r,'support_span_cm':wall_span,'region_cells':len(region),
          'best_support_cells':int(best_support),'competing_orientations':len(competing)}
    if not choices:
        return {'status':'sin_pareja_suficiente','wall':wall,'candidate_markers':[marker(m) for m in markers], 'raw_returns':raw_count,'grid_cells':len(points)}
    choices.sort(key=lambda q:q[0],reverse=True)
    _,a,b,spacing=choices[0]
    return {'status':'hipotesis_ambigua' if competing or len(choices)>1 else 'hipotesis_no_confirmada','wall':wall,'markers':[marker(a),marker(b)],'spacing_cm':spacing,
            'alternatives':len(choices),'ambiguity_reasons':(['orientaciones de pared competidoras'] if competing else []) + (['varias parejas compatibles'] if len(choices)>1 else []),'raw_returns':raw_count,'grid_cells':len(points),
            'warning':'geometria candidata; no confirma identidad ni ausencia de obstáculos'}
def main():
    parser=argparse.ArgumentParser();parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,help='guardar hipótesis sin etiquetas verdaderas')
    parser.add_argument('--lot-length',type=float,default=39.0,help='longitud supuesta del cajón en cm')
    parser.add_argument('--limiter-width',type=float,default=20.0,help='longitud supuesta de cada tope en cm')
    parser.add_argument('--limiter-thickness',type=float,default=2.0,help='grosor supuesto de cada tope en cm')
    args=parser.parse_args()
    hypothesis=build_hypothesis(json.loads(args.input.read_text(encoding='utf-8')),args.lot_length,args.limiter_width,args.limiter_thickness)
    hypothesis['geometry_assumptions_cm']={'lot_length':args.lot_length,'limiter_width':args.limiter_width,'limiter_thickness':args.limiter_thickness}
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(hypothesis,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(hypothesis,ensure_ascii=True))

if __name__=='__main__':main()
