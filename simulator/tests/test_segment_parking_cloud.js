/* Prueba el prototipo offline sin emplear geometria verdadera como entrada. */
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),os=require('node:os'),path=require('node:path'),{spawnSync}=require('node:child_process');
const tools=path.join(__dirname,'../tools'),data=path.join(__dirname,'../mediciones/barridos');
const out=path.join(os.tmpdir(),`autonex-hipotesis-test-${process.pid}.json`);
function call(file,extra=[]){const p=spawnSync('python',[path.join(tools,'segment_parking_cloud.py'),file,...extra],{encoding:'utf8',timeout:30000});assert.equal(p.status,0,p.stderr);return JSON.parse(p.stdout);}
const normal=call(path.join(data,'barridos_ccw_semilla1_distancia45.json'),['--output',out]);
assert.equal(normal.status,'hipotesis_no_confirmada');assert.equal(normal.markers.length,2);
assert(normal.markers.every(m=>m.evidence_start&&m.evidence_end),'cada tope contiene intervalo observado');
assert.deepEqual(JSON.parse(fs.readFileSync(out,'utf8')),normal,'la hipotesis guardada coincide');
const blocked=call(path.join(data,'barridos_ccw_semilla1_distancia45_occlusion.json'));
assert.equal(blocked.status,'hipotesis_ambigua','la oclusion conocida no se confirma');
const decoy=call(path.join(data,'barridos_ccw_semilla1_distancia45_distractor.json'));
assert.equal(decoy.status,'hipotesis_ambigua','varias parejas compatibles no se confirman');
assert(decoy.alternatives>1);
const empty=path.join(os.tmpdir(),`autonex-sin-camara-${process.pid}.json`);fs.writeFileSync(empty,JSON.stringify({camera:null}));
assert.equal(call(empty).status,'sin_deteccion_magenta','no inventar hipotesis sin referencia');
/* Cambiar de marco no debe revelar ni fijar una coordenada absoluta del cajón. */
const transformed=JSON.parse(fs.readFileSync(path.join(data,'barridos_ccw_semilla1_distancia45.json'),'utf8'));
const angle=Math.PI/6,co=Math.cos(angle),si=Math.sin(angle), transform=p=>{
 const x=p.x,z=p.z;p.x=co*x+si*z+13;p.z=-si*x+co*z-8;p.theta+=30;
};
transform(transformed.camera.pose);
for(const scan of transformed.scans){transform(scan.estimated_previous);transform(scan.estimated_current);}
const rotated=path.join(os.tmpdir(),`autonex-rotado-${process.pid}.json`);fs.writeFileSync(rotated,JSON.stringify(transformed));
const rotatedResult=call(rotated);assert.equal(rotatedResult.status,'hipotesis_no_confirmada');
const expected=normal.markers.map(m=>{const q={x:m.wall_end[0],z:m.wall_end[1],theta:0};transform(q);return q;});
for(const m of rotatedResult.markers)assert(Math.min(...expected.map(q=>Math.hypot(m.wall_end[0]-q.x,m.wall_end[1]-q.z)))<2.5,'la hipótesis sigue el cambio de marco');
console.log('PASS: hipótesis guardada, evidencia visible y rechazo de ambigüedad');
