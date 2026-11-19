/* Evaluación contra geometría verdadera solo después de generar los prefijos. */
'use strict';const fs=require('node:fs'),path=require('node:path');const W=require(path.join(__dirname,'../web/sim.js'));
const [input,progress]=process.argv.slice(2);if(!input||!progress)throw new Error('barridos.json progreso.json');
const d=JSON.parse(fs.readFileSync(input,'utf8')),p=JSON.parse(fs.readFileSync(progress,'utf8'));
const field=W.field.randomLayout(d.seed,{direction:d.direction,signs:0,parking:true,startInLot:false,startOnTrack:false});W.field.fitToRobot(field,{length:26,rear:3.775});
const lot=field.lot,truth=lot.limiters.map(b=>({x:b.x,z:lot.wallCoord})),dist=(a,b)=>Math.hypot(a[0]-b.x,a[1]-b.z);
const rows=p.snapshots.map(s=>{const h=s.hypothesis;let maxError=null;if(h.markers){const a=h.markers[0].wall_end,b=h.markers[1].wall_end;maxError=+Math.min(Math.max(dist(a,truth[0]),dist(b,truth[1])),Math.max(dist(a,truth[1]),dist(b,truth[0]))).toFixed(3);}return {scans:s.scans,elapsed_sim_s:s.elapsed_sim_s,status:h.status,max_marker_error_cm:maxError};});
console.log(JSON.stringify({input:path.basename(input),rows}));
