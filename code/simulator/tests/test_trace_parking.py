"""Regresión del detector C sobre pasadas, oclusión y distractor reales simulados."""
import json
import pathlib
import subprocess
import sys
import tempfile

root=pathlib.Path(__file__).resolve().parents[2]
data=root/'simulator/mediciones/barridos'
files=[data/('barridos_ccw_semilla1_distancia45'+suffix+'.json') for suffix in ('','_occlusion','_distractor')]
with tempfile.TemporaryDirectory() as tmp:
    no_camera=pathlib.Path(tmp)/'no_camera.json'
    no_camera.write_text('{"camera":null,"scans":[]}',encoding='utf-8')
    cmd=[sys.executable,str(root/'simulator/tools/evaluate_trace_parking.py'),*[str(f) for f in files],str(no_camera)]
    run=subprocess.run(cmd,text=True,capture_output=True,check=True)
    clear,occluded,distractor,missing=[json.loads(line) for line in run.stdout.splitlines()]
    assert clear['result']['status']==1 and max(clear['wall_end_errors_cm'])<3,clear
    assert occluded['result']['status']!=1,occluded
    assert distractor['result']['status']!=1,distractor
    assert missing['status']=='no_camera_pose',missing
print('PASS: detector C encuentra candidato despejado y conserva incertidumbre ante oclusión, distractor y falta de cámara')
