"""Registra y verifica la correspondencia entre el WASM y sus fuentes."""
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WASM = ROOT / 'simulator/web/wro_core.wasm'
MANIFEST = ROOT / 'simulator/web/wro_core.manifest.json'
DIRS = ('LIDAR System/main', 'LIDAR System/orchestrator', 'Vision System/main', 'shared')


def fingerprint():
    digest = hashlib.sha256()
    files = [ROOT / 'simulator/wasm/core_api.c', ROOT / 'simulator/wasm/build.ps1', ROOT / 'simulator/wasm/manifest.py',
             ROOT / 'simulator/tools/bench/rebuild_wasm.sh']
    for directory in DIRS:
        files.extend(p for p in (ROOT / directory).rglob('*') if p.suffix in ('.c', '.h'))
    for path in sorted(files, key=lambda p: p.relative_to(ROOT).as_posix()):
        digest.update(path.relative_to(ROOT).as_posix().encode('utf-8') + b'\0')
        # Git puede extraer texto con CRLF en Windows y LF en Linux.
        digest.update(path.read_bytes().replace(b'\r\n', b'\n'))
        digest.update(b'\0')
    return {'sources_sha256': digest.hexdigest(), 'wasm_sha256': hashlib.sha256(WASM.read_bytes()).hexdigest()}


def verify():
    if not MANIFEST.exists() or not WASM.exists():
        raise RuntimeError('WASM sin manifiesto: recompila con simulator/wasm/build.ps1')
    if json.loads(MANIFEST.read_text(encoding='utf-8')) != fingerprint():
        raise RuntimeError('WASM desactualizado: recompila con simulator/wasm/build.ps1')


if __name__ == '__main__':
    if sys.argv[1:] == ['--write']:
        MANIFEST.write_text(json.dumps(fingerprint(), indent=2) + '\n', encoding='utf-8')
    else:
        verify()
