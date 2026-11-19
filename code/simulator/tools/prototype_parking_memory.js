/* Experimento aislado: acumula parejas candidatas entre vistas de una ronda.
 * No escribe mapa del firmware ni influye en comandos. No compensa latencia ni
 * deskew; usa cuadro entregado por la cola y pose de campo publicada. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),{performance}=require('node:perf_hooks'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
process.env.PAIR_STRICT='1';const {predict,predictPair}=require('./prototype_parking_association.js');
function distBox(x,z,b){const a=W.geometry.boxAxes(b),dx=x-b.x,dz=z-b.z;return Math.hypot(Math.max(0,Math.abs(dx*a.right[0]+dz*a.right[1])-b.w/2),Math.max(0,Math.abs(dx*a.forward[0]+dz*a.forward[1])-b.d/2));}
const d=(a,b)=>Math.hypot(a.x-b.x,a.z-b.z);
(async()=>{
 const seed=Number(process.argv[2]||1),seconds=Number(process.argv[3]||120),signs=Number(process.argv[4]||0),direction=seed%2?'ccw':'cw';
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 const field=W.field.randomLayout(seed,{direction,signs,parking:true,feasible:true,allowGates:false});
 const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
 let scans=0,observed=0,pairs=0,pairTruth=0,matching=0;const clusters=[], landmarks=[],pairMs=[];let singles=0,singleTruth=0;
 let deliveredVision=null, deliveredCapture=null, deliveredPose=null;const ages=[];
 const capture=sim.capture;sim.capture=function(){
  const a=this.cmd?.arena;const pose=a?.valid?{x:a.x,z:a.z,theta:a.theta}:null;
  capture.call(this);const frame=this.queue.find(e=>e.type==='frame'&&e.payload.captured===this.t);
  if(frame){frame.payload.researchVision=this.snapshot.vision;frame.payload.researchPose=pose;}
 };
 const original=sim.handle;sim.handle=function(e){original.call(this,e);
  if(e.type==='frame'){deliveredVision=e.payload.researchVision;deliveredCapture=e.payload.captured;deliveredPose=e.payload.researchPose;return;}
  if(e.type!=='scan'||++scans%5!==0||!this.cmd?.arena.valid||this.cmd.phase!=='RUN')return;
  const v=deliveredVision;if(!v?.objects.some(o=>o.color==='magenta'))return;observed++;ages.push((this.t-deliveredCapture)*1000);
  const scanPose=this.cmd.arena,align=process.env.ALIGN_CAMERA==='1';
  if(align&&!deliveredPose)return;
  const p=align?deliveredPose:scanPose,th=p.theta*Math.PI/180,world=q=>({x:p.x+Math.cos(th)*q.x+Math.sin(th)*q.f,z:p.z-Math.sin(th)*q.x+Math.cos(th)*q.f});
  let points=e.payload.points;
  if(align){
   const scanTheta=scanPose.theta*Math.PI/180;
   points=points.map(q=>{
    if(!q.valid||q.distance<=0)return q;const a=q.angle*Math.PI/180,x=Math.sin(a)*q.distance,f=10+Math.cos(a)*q.distance;
    const wx=scanPose.x+Math.cos(scanTheta)*x+Math.sin(scanTheta)*f,wz=scanPose.z-Math.sin(scanTheta)*x+Math.cos(scanTheta)*f;
    const dx=wx-p.x,dz=wz-p.z,lx=Math.cos(th)*dx-Math.sin(th)*dz,lf=Math.sin(th)*dx+Math.cos(th)*dz-10;
    return {...q,angle:Math.atan2(lx,lf)*180/Math.PI,distance:Math.hypot(lx,lf)};
   });
  }
  const single=predict(points,v);if(single.chosen){singles++;const pt=world(single.chosen.point);
   const hit=field.lot.limiters.some(box=>distBox(pt.x,pt.z,box)<=4);if(hit)singleTruth++;
   let best=null,nearest=Infinity;for(const c of landmarks){const distance=d(pt,c.p);if(distance<nearest){nearest=distance;best=c;}}
   if(!best||nearest>6){best={p:{...pt},n:0,first:this.t,last:this.t,corners:[],truthHits:0};landmarks.push(best);}
   best.n++;const w=1/best.n;for(const k of ['x','z'])best.p[k]+=(pt[k]-best.p[k])*w;
   best.last=this.t;if(!best.corners.includes(this.cmd.corners))best.corners.push(this.cmd.corners);if(hit)best.truthHits++;
  }
  const start=performance.now(),pair=predictPair(points,v);pairMs.push(performance.now()-start);if(!pair)return;pairs++;
  const a=world(pair.a),b=world(pair.b);const truthA=field.lot.limiters.findIndex(box=>distBox(a.x,a.z,box)<=4),truthB=field.lot.limiters.findIndex(box=>distBox(b.x,b.z,box)<=4);
  if(truthA>=0&&truthB>=0&&truthA!==truthB)pairTruth++;
  let chosen=null,flip=false,best=Infinity;
  for(const c of clusters){const direct=Math.max(d(a,c.a),d(b,c.b)),reverse=Math.max(d(a,c.b),d(b,c.a));if(Math.min(direct,reverse)<best){best=Math.min(direct,reverse);chosen=c;flip=reverse<direct;}}
  if(!chosen||best>8){chosen={a:{...a},b:{...b},n:0,first:this.t,last:this.t,seenCorners:[],truthHits:0};clusters.push(chosen);flip=false;}else matching++;
  const x=flip?b:a,y=flip?a:b;chosen.n++;const w=1/chosen.n;
  for(const k of ['x','z']){chosen.a[k]+=(x[k]-chosen.a[k])*w;chosen.b[k]+=(y[k]-chosen.b[k])*w;}
  chosen.last=this.t;if(!chosen.seenCorners.includes(this.cmd.corners))chosen.seenCorners.push(this.cmd.corners);
  if(truthA>=0&&truthB>=0&&truthA!==truthB)chosen.truthHits++;
 };
 while(sim.t<seconds&&!sim.finished)sim.tick();
 const ordered=clusters.sort((a,b)=>b.n-a.n).slice(0,8).map(c=>({n:c.n,first:+c.first.toFixed(1),last:+c.last.toFixed(1),corners:c.seenCorners,truthHits:c.truthHits,truthDistanceA: +Math.min(...field.lot.limiters.map(box=>distBox(c.a.x,c.a.z,box))).toFixed(2),truthDistanceB:+Math.min(...field.lot.limiters.map(box=>distBox(c.b.x,c.b.z,box))).toFixed(2)}));
 const eligible=landmarks.filter(c=>c.n>=2&&c.corners.length>=2);
 const inferred=[];for(let i=0;i<eligible.length;i++)for(let j=i+1;j<eligible.length;j++){
  const a=eligible[i],b=eligible[j],spacing=d(a.p,b.p);if(spacing<27||spacing>40)continue;
  const ia=field.lot.limiters.findIndex(box=>distBox(a.p.x,a.p.z,box)<=4),ib=field.lot.limiters.findIndex(box=>distBox(b.p.x,b.p.z,box)<=4);
  inferred.push({spacing:+spacing.toFixed(2),n:[a.n,b.n],corners:[a.corners,b.corners],near_distinct:ia>=0&&ib>=0&&ia!==ib});
 }
 ages.sort((a,b)=>a-b);pairMs.sort((a,b)=>a-b);const pm=q=>pairMs.length?+pairMs[Math.ceil(q*pairMs.length)-1].toFixed(2):null;const age=(q)=>ages.length?+ages[Math.ceil(q*ages.length)-1].toFixed(1):null;
 const landmarkSummary=landmarks.sort((a,b)=>b.n-a.n).slice(0,8).map(c=>({n:c.n,first:+c.first.toFixed(1),last:+c.last.toFixed(1),corners:c.corners,truthHits:c.truthHits,truthDistance:+Math.min(...field.lot.limiters.map(box=>distBox(c.p.x,c.p.z,box))).toFixed(2)}));
 console.log(JSON.stringify({seed,direction,signs,alignCamera:process.env.ALIGN_CAMERA==='1',t:+sim.t.toFixed(2),result:sim.result().endReason,scans,observed,pairCostMs:{n:pairMs.length,p50:pm(.5),p95:pm(.95),max:pm(1)},cameraAgeMs:{p50:age(.5),p95:age(.95),max:age(1)},singles,singleTruth,landmarkCount:landmarks.length,landmarkTop:landmarkSummary,inferredPairs:inferred,pairs,pairTruth,matching,clusters:clusters.length,top:ordered}));
})().catch(e=>{console.error(e);process.exitCode=1});
