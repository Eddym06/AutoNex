/* Mide pose relativa, margen fisico, vision y coste WASM durante una ronda completa.
 * node simulator/tools/measure_parking.js [semillas=4] [segundos=180]
 * DIR=cw|ccw limita el sentido; SEED fija la semilla. Medidas solo de simulacion. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { performance } = require('node:perf_hooks');
const web = path.join(__dirname, '../web');
const W = require(path.join(web, 'sim.js')); require(path.join(web, 'core.js'));
const bytes = fs.readFileSync(path.join(web, 'wro_core.wasm'));
const wrap = a => ((a + 180) % 360 + 360) % 360 - 180;
const stats = a => { if (!a.length) return null; const b = [...a].sort((x,y) => x-y); const q = n => +b[Math.ceil(n*b.length)-1].toFixed(3); return { n:b.length, p50:q(.5), p95:q(.95), max:q(1) }; };
const segDist = (p,a,b) => { const dx=b[0]-a[0], dz=b[1]-a[1]; const t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dz)/(dx*dx+dz*dz))); return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dz); };
function boxDistance(a,b) {
  if (W.geometry.overlapBoxes(a,b)) return 0;
  const x=W.geometry.corners(a), y=W.geometry.corners(b); let min=Infinity;
  for (let i=0;i<4;i++) for (let j=0;j<4;j++) {
    min=Math.min(min,segDist(x[i],y[j],y[(j+1)%4]),segDist(y[j],x[i],x[(i+1)%4]));
  }
  return min;
}
function rev(v) { let last=0,n=0; for (const x of v) { if(Math.abs(x)<2)continue; const s=Math.sign(x); if(last && last!==s)n++; last=s; } return n; }
(async () => {
  const core=await W.core.load(bytes), dirs=process.env.DIR ? [process.env.DIR] : ['ccw','cw'];
  const seeds=process.env.SEED ? [Number(process.env.SEED)] : Array.from({length:Number(process.argv[2] || 4)},(_,i)=>i+1);
  const seconds=Number(process.argv[3] || 180);
  for (const direction of dirs) for (const seed of seeds) {
    const demo=process.env.MODE==='entrada';
    const field=W.field.randomLayout(seed,{direction,signs:0,parking:true,startInLot:!demo,startOnTrack:demo});
    const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
    const centre=(field.lot.a0+field.lot.a1)/2, ccw=direction==='ccw';
    const rows={cruise:{pos:[],yaw:[],u:[],w:[],cpu:[]},maneuver:{pos:[],yaw:[],u:[],w:[],cpu:[]}};
    const frame={total:0,magenta:0,parking:0,parkingMagenta:0,near:0,nearMagenta:0};
    const assoc={scans:0,magenta:0,confirmed:0};
    const speeds=[]; let movingBack=false, scans=0, lost=0, minClear=Infinity, minClearTime=null, first=null,last=null;
    let timeScan=0; const scan0=core.scan; core.scan=function(...args) { const t=performance.now(); const out=scan0.apply(this,args); timeScan=performance.now()-t; return out; };
    const handle0=sim.handle; sim.handle=function(e) {
      handle0.call(this,e); if(e.type!=='scan') return;
      assoc.scans++;
      const colored=(this.snapshot.debug && this.snapshot.debug.colored || []).filter(o=>o.color===3);
      if(colored.length) assoc.magenta++;
      if(colored.some(o=>o.confirmed)) assoc.confirmed++;
      if(!this.cmd || this.cmd.phase!=='PARKING') return;
      const c=this.cmd, pose=this.robot.pose(), p=c.park; scans++;
      if(c.speed < -2) movingBack=true;
      const group=movingBack ? rows.maneuver : rows.cruise;
      const u=(ccw?1:-1)*(pose.x-centre), w=pose.z-field.lot.wallCoord;
      const yaw=pose.yaw*180/Math.PI, th=wrap(ccw?90-yaw:yaw+90);
      group.u.push(p.u-u); group.w.push(p.w-w); group.pos.push(Math.hypot(p.u-u,p.w-w)); group.yaw.push(Math.abs(wrap(p.theta-th))); group.cpu.push(timeScan);
      if(p.lost>0) lost++;
      if(!first) first={t:+this.t.toFixed(2),u:+u.toFixed(2),w:+w.toFixed(2),theta:+th.toFixed(2)};
      last={t:+this.t.toFixed(2),u:+u.toFixed(2),w:+w.toFixed(2),theta:+th.toFixed(2)};
    };
    const capture0=sim.capture; sim.capture=function() { capture0.call(this); const v=this.snapshot.vision; const hit=!!(v && v.objects.some(o=>o.color==='magenta')); frame.total++; if(hit)frame.magenta++; if(this.cmd && this.cmd.phase==='PARKING'){frame.parking++; if(hit)frame.parkingMagenta++;} const pose=this.robot.pose(); const near=Math.hypot(pose.x-centre,pose.z-field.lot.wallCoord)<100; if(near){frame.near++;if(hit)frame.nearMagenta++;} };
    while(sim.t<seconds && !sim.finished) {
      sim.tick(); if(!sim.cmd || sim.cmd.phase!=='PARKING')continue;
      speeds.push(sim.robot.v);
      const body=sim.robot.body();
      for(const b of field.walls.concat(field.lot.limiters)) { const d=boxDistance(body,b); if(d<minClear){minClear=d;minClearTime=sim.t;} }
    }
    core.scan=scan0;
    const r=sim.result(); const groups={}; for(const [key,val] of Object.entries(rows))groups[key]={position_cm:stats(val.pos),heading_deg:stats(val.yaw),scan_cpu_ms:stats(val.cpu),u_bias_cm:val.u.length?+(val.u.reduce((a,b)=>a+b,0)/val.u.length).toFixed(3):null,w_bias_cm:val.w.length?+(val.w.reduce((a,b)=>a+b,0)/val.w.length).toFixed(3):null};
    console.log(JSON.stringify({direction,seed,end:r.endReason,parking:r.parking,points:r.points,contacts:r.contacts,reversals:rev(speeds),scans,lost_scans:lost,min_clearance_cm:Number.isFinite(minClear)?+minClear.toFixed(3):null,min_clearance_t_s:minClearTime===null?null:+minClearTime.toFixed(2),first,last,groups,frames:frame,association:assoc}));
  }
})().catch(e=>{console.error(e);process.exitCode=1});
