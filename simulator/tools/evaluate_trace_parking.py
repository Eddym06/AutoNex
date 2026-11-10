"""Reproduce barridos de una pasada con C nativo y evalúa después con verdad.
Uso: python simulator/tools/evaluate_trace_parking.py archivo.json [...]"""
import json, math, os, pathlib, subprocess, sys, tempfile
ROOT=pathlib.Path(__file__).resolve().parents[2]
MAIN=ROOT/'LIDAR System/main'
with tempfile.TemporaryDirectory() as tmp:
    exe=pathlib.Path(tmp)/('trace_parking.exe' if os.name=='nt' else 'trace_parking')
    subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(MAIN),
                    str(MAIN/'trace_map.c'),str(MAIN/'trace_parking.c'),
                    str(ROOT/'simulator/tools/trace_parking_replay.c'),'-lm','-o',str(exe)],check=True)
    for file in sys.argv[1:]:
        data=json.loads(pathlib.Path(file).read_text(encoding='utf-8'))
        cam=data.get('camera')
        if not cam or not cam.get('pose'):
            print(json.dumps({'file':pathlib.Path(file).name,'status':'no_camera_pose'}));continue
        p=cam['pose'];o=cam['object'];f=cam['focal_px']
        # El receptor del P4 conserva el centro, no los bordes de la caja.
        # Aproxima su cálculo de rumbo y amplía la región sin usar esos bordes.
        center=round((o['minX']+o['maxX']+1)/2)
        bearing=math.degrees(math.atan((center-80)/f*math.cos(math.atan(30/118.6))))
        lo,hi=bearing-10,bearing+10
        distance=o['distance']/math.cos(math.radians(bearing))
        limit=int(os.environ.get('TRACE_PARK_MAX_SCANS','0'))
        scans=data['scans'][:limit] if limit>0 else data['scans']
        lines=[' '.join(map(str,[p['x'],p['z'],p['theta'],cam['camera_forward_cm'],cam['camera_lateral_cm'],lo,hi,distance,41,data['lidar_forward_cm'],len(scans)]))]
        for scan in scans:
            q=scan['estimated_current'];rays=scan['rays']
            lines.append(f"{q['x']} {q['z']} {q['theta']} {len(rays)}")
            lines.extend(f'{r[1]} {r[2]} {int(r[3])}' for r in rays)
        run=subprocess.run([str(exe)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
        if run.stderr: sys.stderr.write(run.stderr)
        result=json.loads(run.stdout)
        # La verdad se obtiene solo tras el cálculo C; no se pasa al detector.
        js="const W=require('./simulator/web/sim.js');let f=W.field.randomLayout(+process.argv[1],{direction:process.argv[2],signs:0,parking:true});W.field.fitToRobot(f,{length:W.robot.PROFILES.largo26.bodyLength,rear:W.robot.PROFILES.largo26.rearOverhang});console.log(JSON.stringify(f.lot.limiters.map(b=>[b.x,f.lot.wallCoord])));"
        truth=json.loads(subprocess.check_output(['node','-e',js,str(data['seed']),data['direction']],cwd=ROOT,text=True))
        error=None
        if result['status']==1:
            a,b=result['points'];d=lambda x,y:math.hypot(x[0]-y[0],x[1]-y[1])
            pair1=[d(a,truth[0]),d(b,truth[1])];pair2=[d(a,truth[1]),d(b,truth[0])]
            error=[round(x,2) for x in min([pair1,pair2],key=max)]
        print(json.dumps({'file':pathlib.Path(file).name,'result':result,'wall_end_errors_cm':error}))
