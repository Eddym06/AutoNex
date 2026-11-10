/* Pose de campo a la entrega de cada barrido, comparada con la verdad en ese instante.
 * node simulator/tools/measure_arena_sync.js [semilla=1] [segundos=100]
 * Hay retardo entre captura del barrido y entrega: se informa, no se elimina. */
'use strict';
const fs=require('node:fs'),path=require('node:path');const web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const wrap=a=>((a+180)%360+360)%360-180;
const stats=a=>{if(!a.length)return null;const b=[...a].sort((x,y)=>x-y);const q=v=>+b[Math.ceil(v*b.length)-1].toFixed(3);return {n:b.length,p50:q(.5),p95:q(.95),max:q(1)};};
(async()=>{
 const seed=Number(process.argv[2]||1), secs=Number(process.argv[3]||100), direction=seed%2?'ccw':'cw';
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 const field=W.field.randomLayout(seed,{direction,signs:0,parking:false});
 const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
 const group={straight:{pos:[],yaw:[]},turn:{pos:[],yaw:[]}}, latency=[];let all=0,valid=0;
 const original=sim.handle;sim.handle=function(e){ original.call(this,e);if(e.type!=='scan')return;all++;if(!this.cmd || !this.cmd.arena.valid)return;valid++;
  const p=this.robot.pose(),a=this.cmd.arena,g=Math.abs(this.robot.yawRate*180/Math.PI)>15?group.turn:group.straight;
  g.pos.push(Math.hypot(a.x-p.x,a.z-p.z));g.yaw.push(Math.abs(wrap(a.theta-p.yaw*180/Math.PI)));latency.push((this.t-e.payload.tEnd)*1000);
 };
 while(sim.t<secs && !sim.finished)sim.tick();
 console.log(JSON.stringify({seed,direction,scans:all,valid,delivery_delay_ms:stats(latency),straight:{position_cm:stats(group.straight.pos),heading_deg:stats(group.straight.yaw)},turn:{position_cm:stats(group.turn.pos),heading_deg:stats(group.turn.yaw)},turn_definition_abs_yaw_rate_gt_dps:15}));
})().catch(e=>{console.error(e);process.exitCode=1});
