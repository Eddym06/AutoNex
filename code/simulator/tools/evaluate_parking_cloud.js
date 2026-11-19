/* Evaluador externo del segmentador. La geometría verdadera se construye solo
 * después de ejecutar el prototipo y no se entrega al segmentador. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),{spawnSync}=require('node:child_process');
const W=require(path.join(__dirname,'../web/sim.js'));
const input=process.argv[2];if(!input){console.error('Uso: node simulator/tools/evaluate_parking_cloud.js archivo.json [...]');process.exit(1);}
for(const file of process.argv.slice(2)){
 const d=JSON.parse(fs.readFileSync(file,'utf8'));
 const p=spawnSync('python',[path.join(__dirname,'segment_parking_cloud.py'),file],{encoding:'utf8',maxBuffer:2*1024*1024});
 if(p.status!==0)throw new Error(p.stderr||'fallo el segmentador');const out=JSON.parse(p.stdout);
 const field=W.field.randomLayout(d.seed,{direction:d.direction,signs:0,parking:true,startInLot:false,startOnTrack:false});
 W.field.fitToRobot(field,{length:W.robot.PROFILES.largo26.bodyLength,rear:W.robot.PROFILES.largo26.rearOverhang});
 const lot=field.lot, truth=lot.limiters.map(b=>({x:b.x,z:lot.wallCoord}));
 let metric=null;
 if(['hipotesis_no_confirmada','hipotesis_ambigua'].includes(out.status)){
  const a=out.markers[0].wall_end,b=out.markers[1].wall_end;
  const dist=(p,q)=>Math.hypot(p[0]-q.x,p[1]-q.z);
  const d1=[dist(a,truth[0]),dist(b,truth[1])],d2=[dist(a,truth[1]),dist(b,truth[0])];
  const e=Math.max(...d1)<=Math.max(...d2)?d1:d2;
  const n=out.wall.normal,off=out.wall.offset_cm,c=(lot.a0+lot.a1)/2;
  metric={marker_errors_cm:e.map(x=>+x.toFixed(3)),max_marker_error_cm:+Math.max(...e).toFixed(3),
    wall_offset_at_lot_cm:+Math.abs(n[0]*c+n[1]*lot.wallCoord-off).toFixed(3),
    wall_angle_error_deg:+(Math.acos(Math.min(1,Math.abs(n[1])))*180/Math.PI).toFixed(3)};
 }
 console.log(JSON.stringify({file:path.basename(file),direction:d.direction,seed:d.seed,offset_cm:d.offset_cm,occluded:path.basename(file).includes('_occlusion'),status:out.status,metrics:metric,hypothesis:out}));
}
