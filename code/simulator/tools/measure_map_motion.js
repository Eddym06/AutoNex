/* Inspecciona mapa local contra verdad a entrega y al horizonte futuro para el que se
 * compensa el barrido. No modifica el controlador ni garantiza correspondencia exacta. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const sum=a=>a.reduce((x,y)=>x+y,0);
function local(x,z,b){const a=W.geometry.boxAxes(b),dx=x-b.x,dz=z-b.z;return {x:dx*a.right[0]+dz*a.right[1],z:dx*a.forward[0]+dz*a.forward[1]};}
function dist(x,z,b){const p=local(x,z,b);return Math.hypot(Math.max(0,Math.abs(p.x)-b.w/2),Math.max(0,Math.abs(p.z)-b.d/2));}
function inside(x,z,b,margin){const p=local(x,z,b);return Math.abs(p.x)<b.w/2-margin && Math.abs(p.z)<b.d/2-margin;}
function assess(m,pose,boxes,signs){
 let occ=0,far=0,free=0,falseFree=0,freeInsideSign=0;
 for(let row=0;row<m.n;row++)for(let col=0;col<m.n;col++){
  const f=(m.n/2-row)*m.cell,x=(col-m.n/2)*m.cell;if(Math.abs(f)>80||Math.abs(x)>80)continue;
  const flags=m.map[row*m.n+col];if(!(flags&3))continue;
  const wx=pose.x+Math.cos(pose.yaw)*x+Math.sin(pose.yaw)*f,wz=pose.z-Math.sin(pose.yaw)*x+Math.cos(pose.yaw)*f;
  if(flags&2){occ++;if(Math.min(...boxes.map(b=>dist(wx,wz,b)))>5)far++;}
  else if(flags&1){free++;if(boxes.some(b=>inside(wx,wz,b,.75))){falseFree++;if(signs.some(b=>inside(wx,wz,b,.75)))freeInsideSign++;}}
 }
 return {occ,far,free,falseFree,freeInsideSign};
}
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm'))), secs=Number(process.argv[2]||30);
 for(const direction of ['ccw','cw'])for(const signs of [0,3,7]){
  const seed=direction==='ccw'?1:2,field=W.field.randomLayout(seed,{direction,signs,parking:false,feasible:true,allowGates:false});
  const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
  const rows=[];let scans=0,pending=null;const orig=sim.handle;
  sim.handle=function(e){orig.call(this,e);if(e.type!=='scan'||++scans%10!==0||this.t<2)return;
   const m={map:core.map(),n:core.mapSize(),cell:core.mapCellCm()};
   const delivery=assess(m,this.robot.pose(),this.renderLidarBoxes(),this.field.signs);
   pending={m,due:e.payload.tEnd+0.10,delivery,t:+this.t.toFixed(2),yaw_rate_dps:+(this.robot.yawRate*180/Math.PI).toFixed(2),delay_ms:+((this.t-e.payload.tEnd)*1000).toFixed(1)};
  };
  while(sim.t<secs&&!sim.finished){sim.tick();if(pending && sim.t+1e-8>=pending.due){
   rows.push({t:pending.t,yaw_rate_dps:pending.yaw_rate_dps,delay_ms:pending.delay_ms,comparison_future_t:+sim.t.toFixed(2),delivery:pending.delivery,future:assess(pending.m,sim.robot.pose(),sim.renderLidarBoxes(),sim.field.signs)});pending=null;
  }}
  const total={};for(const mode of ['delivery','future']){total[mode]={};for(const key of ['occ','far','free','falseFree','freeInsideSign'])total[mode][key]=sum(rows.map(r=>r[mode][key]));total[mode].scans_with_false_free=rows.filter(r=>r[mode].falseFree>0).length;}
  console.log(JSON.stringify({direction,seed,signs,scans_sampled:rows.length,t:+sim.t.toFixed(2),end:sim.result().endReason,totals:total,rows}));
 }
})().catch(e=>{console.error(e);process.exitCode=1});
