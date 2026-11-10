/* Comparación externa con la geometría verdadera: nunca alimenta al controlador. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const round = n => +n.toFixed(2);
function distance(x, z, b) {
  const a = W.geometry.boxAxes(b), dx = x-b.x, dz = z-b.z;
  const qx = dx*a.right[0]+dz*a.right[1], qz = dx*a.forward[0]+dz*a.forward[1];
  return Math.hypot(Math.max(0,Math.abs(qx)-b.w/2), Math.max(0,Math.abs(qz)-b.d/2));
}
function summary(values) {
  if (!values.length) return null;
  values.sort((a,b)=>a-b);
  const q = f => round(values[Math.ceil(values.length*f)-1]);
  return { n:values.length, p50:q(.5), p95:q(.95), max:q(1), above5:values.filter(x=>x>5).length };
}
function perimeter(b) {
  const a=W.geometry.boxAxes(b), points=[];
  for (const [length, fixed, alongRight] of [[b.w,-b.d/2,true],[b.w,b.d/2,true],[b.d,-b.w/2,false],[b.d,b.w/2,false]]) {
    const n=Math.max(1,Math.ceil(length));
    for(let i=0;i<n;i++) {
      const along=-length/2+(i+.5)*length/n, x=alongRight?along:fixed, z=alongRight?fixed:along;
      points.push([b.x+x*a.right[0]+z*a.forward[0],b.z+x*a.right[1]+z*a.forward[1]]);
    }
  }
  return points;
}
function surfaceCoverage(box, cells, excludes=[]) {
  /* La separación de 7,5 cm evita contar como tope una muestra cercana a la pared. */
  const samples=perimeter(box).filter(([x,z])=>excludes.every(b=>distance(x,z,b)>7.5)), tolerance=3.75;
  const near=samples.filter(([x,z])=>cells.some(p=>Math.hypot(p[0]-x,p[1]-z)<=tolerance)).length;
  return {samples:samples.length, near, percent:samples.length?round(100*near/samples.length):null, tolerance_cm:tolerance};
}
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 const results=[];
 for(const direction of ['ccw','cw']) for(const seed of [1,3]) {
  const field=W.field.randomLayout(seed,{direction,signs:3,parking:true,startInLot:true});
  const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
  const poseErrors=[];
  while(sim.t<180&&!sim.finished) {
   sim.run(1);
   const a=sim.cmd&&sim.cmd.arena, p=sim.robot.pose();
   if(a&&a.valid)poseErrors.push(Math.hypot(a.x-p.x,a.z-p.z));
  }
  const map=core.traceMap(), cells=[], colored={red:[],green:[],magenta:[]};
  for(let row=0;row<map.size;row++)for(let col=0;col<map.size;col++) {
    const v=map.cells[row*map.size+col], p=[map.originCm+(col+.5)*map.cellCm,map.originCm+(row+.5)*map.cellCm];
    if(v&1)cells.push(p); if(v&2)colored.red.push(p); if(v&4)colored.green.push(p); if(v&8)colored.magenta.push(p);
  }
  const boxes=sim.renderLidarBoxes().filter(b=>b.h>=sim.lidar.cfg.height);
  const errors=cells.map(([x,z])=>Math.min(...boxes.map(b=>distance(x,z,b))));
  const limiters=sim.field.lot.limiters.map(b=>surfaceCoverage(b,cells,sim.field.walls));
  const walls=sim.field.walls.map(b=>surfaceCoverage(b,cells));
  const wallSamples=walls.reduce((n,v)=>n+v.samples,0),wallNear=walls.reduce((n,v)=>n+v.near,0);
  const semantic={};
  for(const [color,pts] of Object.entries(colored)) {
    const targets=color==='magenta'?sim.field.lot.limiters:sim.field.signs.filter(s=>s.color===color);
    semantic[color]={cells:pts.length,within_5cm:pts.filter(([x,z])=>targets.some(b=>distance(x,z,b)<=5)).length};
  }
  const out={direction,seed,seconds:round(sim.t),end_reason:sim.result().endReason,scans:map.scans,cells:cells.length,clipped:map.clipped,semantic,
    distance_to_nearest_true_solid_cm:summary(errors), pose_error_cm:summary(poseErrors),
    wall_perimeter:{samples:wallSamples,near:wallNear,percent:round(100*wallNear/wallSamples),tolerance_cm:3.75},
    limiter_perimeters_away_from_walls:limiters};
  results.push(out);
  console.log(JSON.stringify(out));
 }
 if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(results,null,2)+'\n');
})().catch(e=>{console.error(e);process.exitCode=1});
