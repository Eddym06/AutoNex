/* Verifica el protocolo experimental de barrido: activacion por camara y
 * cobertura diferente con o sin oclusion. Ejecuta el firmware WASM simulado. */
'use strict';
const assert=require('node:assert/strict'),{spawnSync}=require('node:child_process'),path=require('node:path');
const script=path.join(__dirname,'../tools/measure_parking_sweep.js');
function run(extra){
 const env={...process.env,DIR:'ccw',SEED:'1',OFFSET_CM:'45'};
 for(const name of ['OCCLUDER','DISABLE_MAGENTA','EXPORT_DIR'])delete env[name];Object.assign(env,extra);
 const p=spawnSync(process.execPath,[script,'1'],{env,encoding:'utf8',timeout:60000});
 assert.equal(p.status,0,p.stderr||'fallo el experimento');return JSON.parse(p.stdout.trim());
}
const clear=run({}),blind=run({DISABLE_MAGENTA:'1'}),occluded=run({OCCLUDER:'1'});
assert(clear.trigger_t!==null&&clear.used_scans>0&&clear.mapPoints.end>0,'la deteccion inicia la acumulacion');
assert.equal(blind.trigger_t,null,'sin deteccion no se inicia la acumulacion');
assert.equal(blind.mapPoints.end,0,'sin deteccion no se crea nube');
assert.equal(occluded.contacts.wall+occluded.contacts.limiter+occluded.contacts.sign,0,'el ensayo con oclusion no golpea al obstaculo');
for(let i=0;i<2;i++){
 const a=clear.result[i],b=occluded.result[i];
 assert(a.idealBinsWithMatchedRayEndPose>0,'ambos topes tienen rayos observados en el escenario libre');
 assert(b.idealVisibleAfterTrigger<a.idealVisibleAfterTrigger,'el obstaculo reduce visibilidad geometrica');
 assert.equal(a.idealVisibilityMask.length,a.perimeter_or_segment_bins_cm);
 assert.equal(b.registeredRayMaskEnd.length,b.perimeter_or_segment_bins_cm);
}
console.log('PASS: activacion magenta, barridos acumulados y oclusion medible');
