/* El banco de rutas nunca debe ejecutar hipótesis ambiguas. Escenario despejado
 * comprueba que la ruta geometrica candidata se pueda evaluar frente al juez. */
'use strict';
const assert=require('node:assert/strict'),path=require('node:path'),{spawnSync}=require('node:child_process');
const data=path.join(__dirname,'../mediciones/barridos'), script=path.join(__dirname,'../tools/compare_parking_routes.js');
const files=['barridos_ccw_semilla1_distancia45.json','barridos_ccw_semilla1_distancia45_occlusion.json','barridos_ccw_semilla1_distancia45_distractor.json'].map(x=>path.join(data,x));
const p=spawnSync(process.execPath,[script,...files],{encoding:'utf8',timeout:60000});assert.equal(p.status,0,p.stderr);
const rows=p.stdout.trim().split(/\r?\n/).map(JSON.parse);assert.equal(rows.length,3);
assert(rows[0].prefix_scans<130,'el plan no utiliza barridos futuros');
assert.equal(rows[0].results.mapa.found,true);assert.equal(rows[0].results.mapa.idealOverlapSamples,0);assert.equal(rows[0].results.mapa.idealParkingFull,true);
assert.notEqual(rows[1].status,'hipotesis_no_confirmada');assert(!rows[1].results);
assert.notEqual(rows[2].status,'hipotesis_no_confirmada');assert(!rows[2].results);
console.log('PASS: ruta experimental, comparación ideal y rechazo de hipótesis ambiguas');
