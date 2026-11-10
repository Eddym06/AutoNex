/* Prototipo de investigacion: segmentos LiDAR y deteccion magenta en poses inmoviles.
 * No alimenta ni modifica el firmware. La etiqueta geometrica usa verdad del simulador
 * y NO comprueba que la imagen realmente corresponda al mismo objeto. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const RAD=Math.PI/180,DEG=180/Math.PI;
function distBox(x,z,b){const a=W.geometry.boxAxes(b),dx=x-b.x,dz=z-b.z;return Math.hypot(Math.max(0,Math.abs(dx*a.right[0]+dz*a.right[1])-b.w/2),Math.max(0,Math.abs(dx*a.forward[0]+dz*a.forward[1])-b.d/2));}
function segments(points){let current=[],groups=[];const flush=()=>{if(current.length>=3)groups.push(current);current=[];};
 for(const q of points){if(!q.valid||q.distance<=0){flush();continue;}const a=q.angle*RAD,p={x:Math.sin(a)*q.distance,f:10+Math.cos(a)*q.distance};
  if(current.length && Math.hypot(p.x-current.at(-1).x,p.f-current.at(-1).f)>3.5)flush();current.push(p);
 } flush();return groups;
}
function predict(points,vision){
 const obj=vision?.objects.find(o=>o.color==='magenta');if(!obj)return {detected:false};
 const center=Math.round((obj.minX+obj.maxX+1)/2),pitch=Math.atan(30/118.6),bearing=Math.atan((center-80)/118.6*Math.cos(pitch))*DEG;
 const range=obj.distance/Math.cos(bearing*RAD),lo=.5*range-12,hi=1.5*range+12,tol=5+Math.atan(3.5/Math.max(range,5))*DEG;
 const viable=[];for(const group of segments(points)){
  let span=0;for(const a of group)for(const b of group)span=Math.max(span,Math.hypot(a.x-b.x,a.f-b.f));
  if(span<8||span>26)continue;
  const hits=group.filter(p=>{const f=p.f-13,r=Math.hypot(p.x,f),a=Math.atan2(p.x,f)*DEG;return f>0&&r>=lo&&r<=hi&&Math.abs(a-bearing)<=tol;});
  if(!hits.length)continue;
  const near=hits.reduce((a,b)=>Math.hypot(a.x,a.f-13)<Math.hypot(b.x,b.f-13)?a:b);
  viable.push({point:near,range:Math.hypot(near.x,near.f-13),span,segments:group.length});
 }
 viable.sort((a,b)=>a.range-b.range);
 return {detected:true,candidates:viable.length,chosen:viable[0]||null};
}
/* Hipotesis de pared recta y pareja de topes; usa solo muestras LiDAR y una
 * caja magenta. Rechaza cuando no hay suficiente evidencia; no alimenta al robot. */
function predictPair(points,vision){
 const obj=vision?.objects.find(o=>o.color==='magenta');if(!obj)return null;
 const c=Math.round((obj.minX+obj.maxX+1)/2), pitch=Math.atan(30/118.6);
 const bearing=Math.atan((c-80)/118.6*Math.cos(pitch))*DEG, range=obj.distance/Math.cos(bearing*RAD);
 const left=Math.atan((obj.minX-80)/118.6*Math.cos(pitch))*DEG-1.5;
 const right=Math.atan((obj.maxX+1-80)/118.6*Math.cos(pitch))*DEG+1.5;
 const hits=points.filter(p=>p.valid&&p.distance>0&&p.distance<=100).map(p=>({x:Math.sin(p.angle*RAD)*p.distance,f:10+Math.cos(p.angle*RAD)*p.distance}));
 const walls=[];
 for(let h=0;h<36;h++){
  const t=h*5*RAD,nx=Math.cos(t),nf=Math.sin(t),bins=new Map();
  for(const p of hits){const k=Math.round((nx*p.x+nf*p.f)/2);bins.set(k,(bins.get(k)||0)+1);}
  for(const [bin,count] of bins){if(count<15)continue;const r=bin*2;
   const wall=hits.filter(p=>Math.abs(nx*p.x+nf*p.f-r)<=2);
   if(wall.length<15)continue;
   const ss=wall.map(p=>-nf*p.x+nx*p.f);if(Math.max(...ss)-Math.min(...ss)<45)continue;
   walls.push({nx,nf,r,count:wall.length});
  }
 }
 let best=null;
 for(const wall of walls)for(const side of [-1,1]){
  const others=hits.map(p=>({p,s:-wall.nf*p.x+wall.nx*p.f,d:side*(wall.nx*p.x+wall.nf*p.f-wall.r)}))
   .filter(q=>q.d>=2.5&&q.d<=22).sort((a,b)=>a.s-b.s);
  const groups=[];let group=[];const flush=()=>{if(group.length>=3){const ds=group.map(q=>q.d),ss=group.map(q=>q.s);const depth=Math.max(...ds)-Math.min(...ds),width=Math.max(...ss)-Math.min(...ss);
   if(process.env.PAIR_STRICT ? depth>=6&&width<=5 : depth>=3)groups.push(group);}group=[];};
  for(const q of others){if(group.length&&q.s-group.at(-1).s>3)flush();group.push(q);}flush();
  for(let a=0;a<groups.length;a++)for(let b=a+1;b<groups.length;b++){
   const avg=g=>g.reduce((s,q)=>s+q.s,0)/g.length, spacing=Math.abs(avg(groups[a])-avg(groups[b]));
   if(spacing<30||spacing>40)continue;
   const cameraHit=q=>{const f=q.p.f-13,r=Math.hypot(q.p.x,f),ang=Math.atan2(q.p.x,f)*DEG;return f>0&&r>=range*.5-12&&r<=range*1.5+12&&(process.env.PAIR_STRICT ? ang>=left&&ang<=right : Math.abs(ang-bearing)<=7);};
   if(!groups[a].some(cameraHit)&&!groups[b].some(cameraHit))continue;
   const score=wall.count+groups[a].length+groups[b].length;
   if(!best||score>best.score){const repr=g=>({x:g.reduce((s,q)=>s+q.p.x,0)/g.length,f:g.reduce((s,q)=>s+q.p.f,0)/g.length});
    best={a:repr(groups[a]),b:repr(groups[b]),score,spacing};}
  }
 }
 return best;
}
if (require.main === module) (async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 const seed=Number(process.env.BENCH_SEED||1), yawOffsetDeg=Number(process.env.BENCH_YAW||0);
 for(const mode of (process.env.BENCH_MODE ? [process.env.BENCH_MODE] : ['clean','partial_blocker','large_blocker','false_magenta','fake_pair'])){
  const summary={mode,seed,yawOffsetDeg,cases:0,excluded_colliding:0,excluded_target_behind:0,detected_scans:0,candidates:0,chosen_near_limiter:0,chosen_elsewhere:0,actual_confirmed:0,actual_confirmed_near_limiter:0,actual_confirmed_elsewhere:0,multi_candidate:0,pair_candidates:0,pair_both_near_distinct:0,pair_other:0,examples:[]};
  for(const direction of ['ccw','cw'])for(const u of [-80,-65,-50,-35,-20,-5,10])for(const w of [27,35,45,60]){
   const field=W.field.randomLayout(seed,{direction,signs:0,parking:true,startOnTrack:true});
   const sim=new W.sim.Simulation(core,{field,seed,startDelay:1e6,robot:W.robot.PROFILES.largo26});
   const centre=(field.lot.a0+field.lot.a1)/2,sgn=direction==='ccw'?1:-1;
   sim.robot.reset({x:centre+sgn*u,z:field.lot.wallCoord+w,yaw:sgn*Math.PI/2+yawOffsetDeg*RAD});
   let truth=field.lot.limiters;
   const pose=sim.robot.pose();
   if(mode==='partial_blocker'||mode==='large_blocker'){
    const target=truth[direction==='ccw'?0:1],size=mode==='partial_blocker'?5:20;
    const origin={x:pose.x+Math.sin(pose.yaw)*13,z:pose.z+Math.cos(pose.yaw)*13};
    if ((target.x-origin.x)*sgn <= 5) { summary.excluded_target_behind++; continue; }
    const blocker={x:(origin.x+target.x)/2,z:(origin.z+target.z)/2,w:size,d:size,rot:0,h:10,kind:'wall'};
    field.walls.push(blocker);sim._lidarBoxes=null;
   } else if(mode==='false_magenta'||mode==='fake_pair'){
    const targets=mode==='fake_pair' ? truth : [truth[direction==='ccw'?0:1]];
    for(const target of targets)field.walls.push({...target,kind:'magenta'});
    field.lot.limiters=[];truth=[];sim._lidarBoxes=null;
   }
   if(field.walls.concat(field.lot.limiters).some(b=>W.geometry.overlapBoxes(sim.robot.body(),b))) {
    summary.excluded_colliding++; continue;
   }
   let measured=false;
   const original=sim.handle;sim.handle=function(e){original.call(this,e);if(e.type!=='scan'||this.t<.5)return;
    if(process.env.PAIR_ONLY&&measured)return; measured=true;
    const p=predict(e.payload.points,this.snapshot.vision);if(!p.detected)return;summary.detected_scans++;
    if(process.env.PAIR_ONLY){
     const pair=predictPair(e.payload.points,this.snapshot.vision);
     if(pair){summary.pair_candidates++;
      const pos=this.robot.pose();const world=q=>({x:pos.x+Math.cos(pos.yaw)*q.x+Math.sin(pos.yaw)*q.f,z:pos.z-Math.sin(pos.yaw)*q.x+Math.cos(pos.yaw)*q.f});
      const a=world(pair.a),b=world(pair.b);
      const ia=truth.findIndex(box=>distBox(a.x,a.z,box)<=2.5),ib=truth.findIndex(box=>distBox(b.x,b.z,box)<=2.5);
      if(ia>=0&&ib>=0&&ia!==ib)summary.pair_both_near_distinct++;else summary.pair_other++;
     }
    }
    const actual=(this.snapshot.debug.colored||[]).find(o=>o.color===3&&o.confirmed);
    if(actual){
     summary.actual_confirmed++;
     const pose=this.robot.pose(), x=pose.x+Math.cos(pose.yaw)*actual.x+Math.sin(pose.yaw)*actual.forward;
     const z=pose.z-Math.sin(pose.yaw)*actual.x+Math.cos(pose.yaw)*actual.forward;
     if(truth.some(b=>distBox(x,z,b)<=2.5))summary.actual_confirmed_near_limiter++;
     else { summary.actual_confirmed_elsewhere++;
      if(summary.examples.length<2)summary.examples.push({direction,u,w,t:+this.t.toFixed(2),pose:{x:+pose.x.toFixed(2),z:+pose.z.toFixed(2)},anchor:{x:+x.toFixed(2),z:+z.toFixed(2)},nearest_limiter_cm:truth.length?+Math.min(...truth.map(b=>distBox(x,z,b))).toFixed(2):null,nearest_wall_cm:+Math.min(...field.walls.map(b=>distBox(x,z,b))).toFixed(2)});
     }
    }
    if(!p.chosen)return;summary.candidates++;if(p.candidates>1)summary.multi_candidate++;
    const q=p.chosen.point,pose=this.robot.pose(),x=pose.x+Math.cos(pose.yaw)*q.x+Math.sin(pose.yaw)*q.f,z=pose.z-Math.sin(pose.yaw)*q.x+Math.cos(pose.yaw)*q.f;
    if(truth.some(b=>distBox(x,z,b)<=2.5))summary.chosen_near_limiter++;else summary.chosen_elsewhere++;
   };
   while(sim.t<1.3&&!sim.finished)sim.tick();summary.cases++;
  }
  console.log(JSON.stringify(summary));
 }
})().catch(e=>{console.error(e);process.exitCode=1});

module.exports = { predict, predictPair };
