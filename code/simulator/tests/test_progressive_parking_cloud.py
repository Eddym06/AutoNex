"""Verifica acumulación progresiva y equivalencia con procesamiento por lote."""
import json
import sys
from pathlib import Path
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'simulator/tools'))
from progressive_parking_cloud import run
from segment_parking_cloud import build_hypothesis
base=root/'simulator/mediciones/barridos'
for suffix,expected in [('', 'hipotesis_no_confirmada'),('_occlusion','hipotesis_ambigua'),('_distractor','hipotesis_ambigua')]:
    data=json.loads((base/f'barridos_ccw_semilla1_distancia45{suffix}.json').read_text(encoding='utf8'))
    result=run(data,10)
    assert result['snapshots'][-1]['hypothesis']==build_hypothesis(data)
    assert result['snapshots'][-1]['hypothesis']['status']==expected
    assert result['snapshots'][0]['hypothesis']['status'] not in ('hipotesis_no_confirmada','hipotesis_ambigua')
assert run({'camera':None,'scans':[]},10)['status']=='sin_deteccion_magenta'
print('PASS: acumulación progresiva, lote equivalente y casos ambiguos')
