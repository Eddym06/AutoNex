/* Experimento aislado: deteccion magenta y union progresiva de barridos al pasar
 * en linea recta junto al cajon. El coche se mueve con mando fijo de prueba;
 * el controlador no recibe el mapa y no se cambia el firmware.
 * node simulator/tools/measure_parking_sweep.js [semillas=2]
 * DIR, OFFSET_CM y SEED limitan casos; LOS datos verdaderos SOLO se usan para evaluar. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const RAD=Math.PI/180;
const quant=(a,q)=>{if(!a.length)return null;const v=[...a].sort((x,y)=>x-y);return +v[Math.ceil(q*v.length)-1].toFixed(3);};
const wrap=a=>((a+180)%360+360)%360-180;
function nearestRect(x,z,b){
 const a=W.geometry.boxAxes(b),dx=x-b.x,dz=z-b.z,lx=dx*a.right[0]+dz*a.right[1],lz=dx*a.forward[0]+dz*a.forward[1],hw=b.w/2,hd=b.d/2;
 const s=[{x:Math.max(-hw,Math.min(hw,lx)),z:-hd,k:Math.max(0,Math.min(b.w,lx+hw))},
 {x:hw,z:Math.max(-hd,Math.min(hd,lz)),k:b.w+Math.max(0,Math.min(b.d,lz+hd))},
 {x:Math.max(-hw,Math.min(hw,lx)),z:hd,k:b.w+b.d+Math.max(0,Math.min(b.w,hw-lx))},
 {x:-hw,z:Math.max(-hd,Math.min(hd,lz)),k:2*b.w+b.d+Math.max(0,Math.min(b.d,hd-lz))}];
 const best=s.reduce((p,q)=>Math.hypot(lx-p.x,lz-p.z)<Math.hypot(lx-q.x,lz-q.z)?p:q);
 return {distance:Math.hypot(lx-best.x,lz-best.z),bin:Math.min(Math.ceil(2*(b.w+b.d))-1,Math.floor(best.k))};
}
function endpoint(p,pose,mount){const a=(p.angle*RAD)+pose.yaw,co=Math.cos(pose.yaw),sn=Math.sin(pose.yaw);return {x:pose.x+sn*mount+Math.sin(a)*p.distance,z:pose.z+co*mount+Math.cos(a)*p.distance};}
function interp(a,b,t){const f=Math.max(0,Math.min(1,(t-a.t)/(b.t-a.t)));return {x:a.x+(b.x-a.x)*f,z:a.z+(b.z-a.z)*f,yaw:(a.theta+wrap(b.theta-a.theta)*f)*RAD};}
function boxArray(n){return {total:n,ideal:new Set(),idealAfter:new Set(),measured:new Set(),truthBins:new Set(),endBins:new Set(),interpBins:new Set(),trueRayBins:new Set(),endRayBins:new Set(),interpRayBins:new Set(),errorsEnd:[],errorsInterp:[],surfaceEnd:[],surfaceInterp:[],accepted:0,ambiguous:0};}
(async()=>{
 const count=Number(process.argv[2]||2),dirs=process.env.DIR?[process.env.DIR]:['ccw','cw'];
 const seeds=process.env.SEED?[Number(process.env.SEED)]:Array.from({length:count},(_,i)=>i+1);
 const offsets=process.env.OFFSET_CM?[Number(process.env.OFFSET_CM)]:[35,45,55];
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 for(const direction of dirs)for(const seed of seeds)for(const offset of offsets){
  const field=W.field.randomLayout(seed,{direction,signs:0,parking:true,startInLot:false,startOnTrack:false});
  const visionConfig=process.env.DISABLE_MAGENTA==='1'?W.sim.DEFAULT_VISION_TEXT+'\nmagenta.enabled off':W.sim.DEFAULT_VISION_TEXT;
  const sim=new W.sim.Simulation(core,{field,seed,visionConfig,robot:W.robot.PROFILES.largo26});
  if(process.env.OCCLUDER==='1'){
   field.walls.push({x:(field.lot.a0+field.lot.a1)/2,z:field.lot.wallCoord+25.5,w:60,d:3,rot:0,h:10,kind:'wall'});
   sim._lidarBoxes=null;
  }
  const lot=field.lot,centre=(lot.a0+lot.a1)/2,sgn=direction==='ccw'?1:-1;
  const yawOffsetDeg=Number(process.env.YAW_OFFSET_DEG||0);
  const startU=Number(process.env.START_DISTANCE_CM||80);
  if(process.env.DISTRACTOR==='1'){
   field.walls.push({x:centre-sgn*(lot.length/2+1+41),z:lot.wallCoord+25,w:2,d:20,rot:0,h:10,kind:'magenta'});
   sim._lidarBoxes=null;
  }
  const initial={x:centre-sgn*startU,z:lot.wallCoord+offset,yaw:sgn*Math.PI/2+yawOffsetDeg*RAD};sim.robot.reset(initial);
  const limits=lot.limiters,wall0=lot.a0-20,wall1=lot.a1+20,wallN=Math.ceil(wall1-wall0);
  const data=[boxArray(Math.ceil(2*(limits[0].w+limits[0].d))),boxArray(Math.ceil(2*(limits[1].w+limits[1].d))),boxArray(wallN)];
  let trigger=null,triggerCaptured=null,triggerCamera=null,scans=0,used=0,noPose=0,lastPose=null,invalidFrame=0;const mapPoints={end:0,interp:0},cloudEnd=[],cloudInterp=[],scanLog=[];
  const initialLidar=sim.lidar.measure;sim.lidar.measure=function(i,pose,boxes){const p=initialLidar.call(this,i,pose,boxes);return {...p,_pose:{...pose},_t:this.revStart+(i+.5)/(this.cfg.hz*this.cfg.rays)};};
  const capture0=sim.capture;sim.capture=function(){
   const a=this.cmd?.arena,pose=a?.valid?{x:a.x,z:a.z,theta:a.theta}:null;
   capture0.call(this);const e=this.queue.find(q=>q.type==='frame'&&q.payload.captured===this.t);
   if(e){e.payload.researchVision=this.snapshot.vision;e.payload.researchPose=pose;}
  };
  const handle0=sim.handle;sim.handle=function(e){
   if(e.type==='cmd')return;
   const before=this.stats.frames;handle0.call(this,e);
   if(e.type==='frame'){
    if(this.stats.frames===before){invalidFrame++;return;}
    const v=e.payload.researchVision;
    if(!trigger&&this.started&&Math.abs(this.robot.v)>2&&v?.objects.some(o=>o.color==='magenta')){
     trigger=this.t;triggerCaptured=e.payload.captured;
     triggerCamera={captured_s:e.payload.captured,delivered_s:this.t,pose:e.payload.researchPose,
      object:v.objects.find(o=>o.color==='magenta'),focal_px:this.camCfg.focal,
      camera_forward_cm:this.camCfg.mountForward,camera_lateral_cm:this.camCfg.mountLateral};
    }
    return;
   }
   if(e.type!=='scan')return;scans++;
   const arena=this.cmd?.arena,cur=arena?.valid?{x:arena.x,z:arena.z,theta:arena.theta,t:e.payload.tEnd}:null;
   const prev=lastPose;if(cur)lastPose=cur;
   const collect=trigger!==null&&cur&&prev&&cur.t>prev.t&&e.payload.tEnd>=trigger;
   if(trigger!==null&&!collect)noPose++;
   if(collect){
    used++;
    if(process.env.EXPORT_DIR)scanLog.push({delivered_s:+this.t.toFixed(6),scan_end_s:e.payload.tEnd,
     estimated_previous:prev,estimated_current:cur,
     rays:e.payload.points.filter(p=>p._t>=trigger).map(p=>[+p._t.toFixed(6),p.angle,p.distance,p.valid?1:0,p.blind?1:0])});
   }
   const boxes=this.renderLidarBoxes(),cfg=this.lidar.cfg;
   for(const p of e.payload.points){
    if(!p._pose||p._t<1.8||cfg.selfMask.some(([a,b])=>p.angle>=a&&p.angle<=b))continue;
    const enabled=collect&&p._t>=trigger;
    // El mapa se construye ANTES de cualquier consulta a la geometria verdadera.
    if(enabled&&p.valid&&p.distance>0){
     cloudEnd.push(endpoint(p,{x:cur.x,z:cur.z,yaw:cur.theta*RAD},cfg.mountForward));
     cloudInterp.push(endpoint(p,interp(prev,cur,p._t),cfg.mountForward));
     mapPoints.end++;mapPoints.interp++;
    }
    const v=p._pose,ang=v.yaw+p.angle*RAD,dx=Math.sin(ang),dz=Math.cos(ang),ox=v.x+Math.sin(v.yaw)*cfg.mountForward+Math.cos(v.yaw)*cfg.mountLateral,oz=v.z+Math.cos(v.yaw)*cfg.mountForward-Math.sin(v.yaw)*cfg.mountLateral;
    let closest=Infinity,target=null;
    for(const box of boxes){if(box.h<cfg.height)continue;const hit=W.geometry.rayBox(ox,oz,dx,dz,box);if(hit&&hit.t<closest){closest=hit.t;target=box;}}
    if(!target||closest<cfg.minCm||closest>cfg.maxCm)continue;
    const ix=ox+dx*closest,iz=oz+dz*closest;
    let k=limits.indexOf(target),idealBin;
    if(k>=0)idealBin=nearestRect(ix,iz,target).bin;
    else if(target.kind==='wall'&&Math.abs(iz-lot.wallCoord)<.05&&ix>=wall0&&ix<wall1){k=2;idealBin=Math.floor(ix-wall0);}
    else continue;
    const d=data[k];d.ideal.add(idealBin);if(enabled)d.idealAfter.add(idealBin);
    if(!enabled||!p.valid||Math.abs(p.distance-closest)>3){if(enabled&&p.valid&&Math.abs(p.distance-closest)>3)d.ambiguous++;continue;}
    d.measured.add(idealBin);d.accepted++;
    const truth=endpoint(p,p._pose,cfg.mountForward), end=endpoint(p,{x:cur.x,z:cur.z,yaw:cur.theta*RAD},cfg.mountForward),mid=endpoint(p,interp(prev,cur,p._t),cfg.mountForward);
    // Evalua cada retorno contra su interseccion ideal: evita atribuir puntos de
    // una pared cercana a una cara oculta de un tope delgado.
    if(Math.hypot(truth.x-ix,truth.z-iz)<=2.5)d.trueRayBins.add(idealBin);
    if(Math.hypot(end.x-ix,end.z-iz)<=2.5)d.endRayBins.add(idealBin);
    if(Math.hypot(mid.x-ix,mid.z-iz)<=2.5)d.interpRayBins.add(idealBin);
    d.errorsEnd.push(Math.hypot(end.x-truth.x,end.z-truth.z));d.errorsInterp.push(Math.hypot(mid.x-truth.x,mid.z-truth.z));
    const locate=q=>k===2?{distance:Math.abs(q.z-lot.wallCoord),bin:Math.floor(q.x-wall0)}:nearestRect(q.x,q.z,limits[k]);
    const real=locate(truth),er=locate(end),ir=locate(mid);
    if(real.distance<=2.5&&real.bin>=0&&real.bin<d.total)d.truthBins.add(real.bin);
    if(er.distance<=2.5&&er.bin>=0&&er.bin<d.total)d.endBins.add(er.bin);
    if(ir.distance<=2.5&&ir.bin>=0&&ir.bin<d.total)d.interpBins.add(ir.bin);
    d.surfaceEnd.push(er.distance);d.surfaceInterp.push(ir.distance);
   }
  };
  while(sim.t<30&&!sim.finished){
   sim.robot.cmd={steer:0,speed:sim.t>=1.8?12:0,yawRate:0};sim.tick();
   const u=sgn*(sim.robot.pose().x-centre);if(u>65)break;
  }
  const mask=(bins,n)=>Array.from({length:n},(_,i)=>bins.has(i)?'#':'.').join('');
  const overlap=(ref,bins)=>{let n=0;for(const i of ref)if(bins.has(i-1)||bins.has(i)||bins.has(i+1))n++;return n;};
  const fromCloud=(cloud,i,d)=>{const bins=new Set();let nearby=0;for(const p of cloud){
   const q=i===2?{distance:Math.abs(p.z-lot.wallCoord),bin:Math.floor(p.x-wall0)}:nearestRect(p.x,p.z,limits[i]);
   if(q.distance<=2.5&&q.bin>=0&&q.bin<d.total){nearby++;bins.add(q.bin);}
  }return {pointsNearSurface:nearby,nearSurfaceBins:bins.size,idealVisibleBinsRecoveredWithin1cm:overlap(d.idealAfter,bins)};};
  const result=data.map((d,i)=>({part:i===0?'tope_izquierda_campo':i===1?'tope_derecha_campo':'pared_segmento',perimeter_or_segment_bins_cm:d.total,
   idealVisibleAll:d.ideal.size,idealVisibleAfterTrigger:d.idealAfter.size,measuredIdealBins:d.measured.size,
   idealVisibilityMask:mask(d.idealAfter,d.total),registeredRayMaskEnd:mask(d.endRayBins,d.total),
   truePoseNearSurfaceBins:d.truthBins.size,endPoseNearSurfaceBins:d.endBins.size,interpolatedPoseNearSurfaceBins:d.interpBins.size,
   idealBinsWithMatchedRayTruth:d.trueRayBins.size,
   idealBinsWithMatchedRayEndPose:d.endRayBins.size,
   idealBinsWithMatchedRayInterpolation:d.interpRayBins.size,
   acceptedReturns:d.accepted,ambiguousReturns:d.ambiguous,
   allCloudEnd:fromCloud(cloudEnd,i,d),allCloudInterpolated:fromCloud(cloudInterp,i,d),
   idealVisibleBinsRecoveredByLabeledTruthWithin1cm:overlap(d.idealAfter,d.truthBins),
   idealVisibleBinsRecoveredByLabeledEndWithin1cm:overlap(d.idealAfter,d.endBins),
   idealVisibleBinsRecoveredByLabeledInterpolationWithin1cm:overlap(d.idealAfter,d.interpBins),
   endpointErrorEndCm:{p50:quant(d.errorsEnd,.5),p95:quant(d.errorsEnd,.95),max:quant(d.errorsEnd,1)},
   endpointErrorInterpCm:{p50:quant(d.errorsInterp,.5),p95:quant(d.errorsInterp,.95),max:quant(d.errorsInterp,1)},
   surfaceDistanceEndCm:{p50:quant(d.surfaceEnd,.5),p95:quant(d.surfaceEnd,.95)},
   surfaceDistanceInterpCm:{p50:quant(d.surfaceInterp,.5),p95:quant(d.surfaceInterp,.95)}}));
  const contacts=sim.result().contacts;
  if(process.env.EXPORT_DIR){
   fs.mkdirSync(process.env.EXPORT_DIR,{recursive:true});
   const name=`barridos_${direction}_semilla${seed}_distancia${offset}${yawOffsetDeg?'_yaw'+yawOffsetDeg:''}${process.env.OCCLUDER==='1'?'_occlusion':''}${process.env.DISTRACTOR==='1'?'_distractor':''}.json`;
   fs.writeFileSync(path.join(process.env.EXPORT_DIR,name),JSON.stringify({description:'Barridos y poses estimadas; sin geometria verdadera ni decisiones de asociacion',direction,seed,offset_cm:offset,yaw_offset_deg:yawOffsetDeg,distractor:process.env.DISTRACTOR==='1',trigger_s:trigger,camera:triggerCamera,lidar_forward_cm:sim.lidar.cfg.mountForward,ray_columns:['t_s','angle_deg','distance_cm','valid','blind'],scans:scanLog}));
  }
  console.log(JSON.stringify({direction,seed,offset_cm:offset,yaw_offset_deg:yawOffsetDeg,occluder:process.env.OCCLUDER==='1',distractor:process.env.DISTRACTOR==='1',magentaDisabled:process.env.DISABLE_MAGENTA==='1',trigger_t:trigger===null?null:+trigger.toFixed(2),trigger_captured_t:triggerCaptured===null?null:+triggerCaptured.toFixed(2),scans,used_scans:used,no_pose_scans_after_trigger:noPose,final_t:+sim.t.toFixed(2),contacts,mapPoints,result}));
 }
})().catch(e=>{console.error(e);process.exitCode=1});
