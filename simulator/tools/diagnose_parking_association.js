/* Diagnostico de los filtros de asociacion actuales con poses inmoviles.
 * Reconstruccion de los filtros en JS solo para diagnostico: no altera firmware.
 * La cercania a un tope no prueba por si sola correspondencia visual correcta. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const RAD=Math.PI/180,DEG=180/Math.PI;
function distBox(x,z,b){const a=W.geometry.boxAxes(b),dx=x-b.x,dz=z-b.z;return Math.hypot(Math.max(0,Math.abs(dx*a.right[0]+dz*a.right[1])-b.w/2),Math.max(0,Math.abs(dx*a.forward[0]+dz*a.forward[1])-b.d/2));}
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 let sum={cases:0,cameraEntries:0,actualConfirmed:0,windowCandidates:0,windowOnLimiter:0,nearestLimiter:0,nearestOther:0,compactCandidates:0,compactOnLimiter:0,onlyWallCandidates:0,diagDisagreement:0};
 for(const direction of ['ccw','cw'])for(const u of [-50,-35,-20,-5,10])for(const w of [27,35,45,60]){
  const field=W.field.randomLayout(1,{direction,signs:0,parking:true,startOnTrack:true});
  const sim=new W.sim.Simulation(core,{field,seed:1,startDelay:1e6,robot:W.robot.PROFILES.largo26});
  const centre=(field.lot.a0+field.lot.a1)/2,sgn=direction==='ccw'?1:-1;
  sim.robot.reset({x:centre+sgn*u,z:field.lot.wallCoord+w,yaw:sgn*Math.PI/2});
  const orig=sim.handle;let data={direction,u,w,entries:0,confirmed:0,window:0,windowLimiter:0,nearestLimiter:0,nearestOther:0,compact:0,compactLimiter:0,disagree:0};
  sim.handle=function(e){orig.call(this,e);if(e.type!=='scan'||this.t<0.4)return;
   const det=(this.snapshot.debug.colored||[]).find(x=>x.color===3);if(!det)return;
   data.entries++;if(det.confirmed)data.confirmed++;
   // Entrada no confirmada conserva la estimacion de camara; si esta confirmada
   // no puede reconstruirse la estimacion original, asi que solo se cuentan entradas no confirmadas.
   if(det.confirmed)return;
   const bearing=Math.atan2(det.x,det.forward-13)*DEG,r=Math.hypot(det.x,det.forward-13);
   const lo=.5*r-12,hi=1.5*r+12,tol=5+Math.atan(3.5/Math.max(r,5))*DEG;
   const hits=e.payload.points.filter(p=>p.valid&&p.distance>0).map(p=>({x:Math.sin(p.angle*RAD)*p.distance,f:10+Math.cos(p.angle*RAD)*p.distance})).filter(p=>Math.max(Math.abs(p.x),Math.abs(p.f))<=200);
   const candidates=hits.filter(p=>{const f=p.f-13,range=Math.hypot(p.x,f);return f>0 && range>=lo && range<=hi && Math.abs(Math.atan2(p.x,f)*DEG-bearing)<=tol;});
   const onLimiter=p=>{const pose=this.robot.pose(),x=pose.x+Math.cos(pose.yaw)*p.x+Math.sin(pose.yaw)*p.f,z=pose.z-Math.sin(pose.yaw)*p.x+Math.cos(pose.yaw)*p.f;return field.lot.limiters.some(b=>distBox(x,z,b)<=2);};
   const compact=candidates.filter(p=>!hits.some(q=>{const d=Math.hypot(p.x-q.x,p.f-q.f);return d>7.5 && d<=12;}));
   if(candidates.length){
    data.window++;
    const nearest=candidates.reduce((best,p)=>Math.hypot(p.x,p.f-13)<Math.hypot(best.x,best.f-13)?p:best);
    if(onLimiter(nearest))data.nearestLimiter++;else data.nearestOther++;
   }
   if(candidates.some(onLimiter))data.windowLimiter++;
   if(compact.length)data.compact++;if(compact.some(onLimiter))data.compactLimiter++;
   if(compact.length)data.disagree++; // no confirmado por C a pesar de candidato en replica
  };
  while(sim.t<1.5 && !sim.finished)sim.tick();
  sum.cases++;sum.cameraEntries+=data.entries;sum.actualConfirmed+=data.confirmed;sum.windowCandidates+=data.window;sum.windowOnLimiter+=data.windowLimiter;sum.nearestLimiter+=data.nearestLimiter;sum.nearestOther+=data.nearestOther;sum.compactCandidates+=data.compact;sum.compactOnLimiter+=data.compactLimiter;sum.diagDisagreement+=data.disagree;if(data.window&&!data.windowLimiter)sum.onlyWallCandidates++;
  if(data.entries)console.log(JSON.stringify(data));
 }
 console.log('TOTAL '+JSON.stringify(sum));
})().catch(e=>{console.error(e);process.exitCode=1});
