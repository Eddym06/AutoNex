/* Compara el A* C usando geometria segmentada y geometria verdadera, desde la
 * misma pose estimada. Simula su ejecucion fisica fuera del controlador. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),{spawnSync}=require('node:child_process');
const root=path.resolve(__dirname,'../..'),W=require(path.join(root,'simulator/web/sim.js'));
const main=path.join(root,'LIDAR System/main'),native=path.join(os.tmpdir(),`autonex-plan-experiment-${process.pid}.exe`);
const compiler=spawnSync('gcc',['-O2','-std=c11','-I',main,path.join(main,'park_ackermann.c'),path.join(main,'park_planner.c'),path.join(__dirname,'plan_parking_experiment.c'),'-o',native,'-lm'],{encoding:'utf8'});
if(compiler.status!==0)throw new Error(compiler.stderr||'fallo compilacion');
const wrap=x=>((x+180)%360+360)%360-180,dot=(a,b)=>a[0]*b[0]+a[1]*b[1];
function frame(center,u,w){return {center,u,w};}
function poseLocal(p,f){const delta=[p.x-f.center[0],p.z-f.center[1]],yaw=p.theta*Math.PI/180,dir=[Math.sin(yaw),Math.cos(yaw)];return {u:dot(delta,f.u),w:dot(delta,f.w),theta:Math.atan2(dot(dir,f.w),dot(dir,f.u))*180/Math.PI};}
function worldPose(p,f){const theta=p.theta*Math.PI/180,fx=f.u[0]*Math.cos(theta)+f.w[0]*Math.sin(theta),fz=f.u[1]*Math.cos(theta)+f.w[1]*Math.sin(theta);return {x:f.center[0]+f.u[0]*p.u+f.w[0]*p.w,z:f.center[1]+f.u[1]*p.u+f.w[1]*p.w,yaw:Math.atan2(fx,fz)};}
function plan(p,length){const r=spawnSync(native,[p.u,p.w,p.theta,length,1].map(String),{encoding:'utf8'});if(r.status!==0)throw new Error(r.stderr);return JSON.parse(r.stdout);}
function countRev(arcs){let prev=1,n=0;for(const [d] of arcs){if(d!==prev)n++;prev=d;}return n;}
const {observedGrid,observedCollision,boxDistance}=require('./parking_cloud_collision.js');
function ideal(planResult,start,f,field,grid){let p={...start},hits=0,mapHits=0,minClear=Infinity;const car=new W.robot.Robot(W.robot.PROFILES.largo26);const obstacles=field.walls.concat(field.lot.limiters,field.signs);
 for(const [direction,steer] of planResult.arcs)for(let j=0;j<6;j++){const ds=direction*3.5/6,k=Math.tan(steer*40*Math.PI/180)/18.45,dt=ds*k,mid=p.theta*Math.PI/180+dt/2;p.u+=ds*Math.cos(mid);p.w+=ds*Math.sin(mid);p.theta=wrap(p.theta+dt*180/Math.PI);const box=car.body(worldPose(p,f));const clearance=Math.min(...obstacles.map(b=>boxDistance(box,b)));minClear=Math.min(minClear,clearance);if(clearance<=0)hits++;if(observedCollision(box,grid))mapHits++;}
 return {overlapSamples:hits,observedObstacleSamples:mapHits,minTrueClearanceCm:+minClear.toFixed(3),final:worldPose(p,f)};}
function goal(p,field){const car=new W.robot.Robot(W.robot.PROFILES.largo26),box=car.body(p),corners=W.geometry.corners(box),lot=field.lot;const heading=field.direction==='ccw'?90:-90;return corners.every(([x,z])=>x>lot.a0+.5&&x<lot.a1-.5&&z>lot.wallCoord+.5&&z<lot.innerCoord-.5)&&Math.abs(wrap(p.yaw*180/Math.PI-heading))<=10;}
function dynamic(planResult,start,f,field,feedback=false){const car=new W.robot.Robot(W.robot.PROFILES.largo26);car.reset(worldPose(start,f));const det=f.u[0]*f.w[1]-f.u[1]*f.w[0],solids=field.walls.concat(field.lot.limiters,field.signs),speeds=[];let ticks=0,collision=null;
 function step(steer,speed){car.cmd={steer,speed};const events=car.step(.01,{solids,signs:[]});speeds.push(car.v);ticks++;const hit=events.find(e=>e.kind==='wall'||e.kind==='limiter');if(hit)collision=hit.kind;}
 const segments=[];for(const [dir,s] of planResult.arcs){const last=segments.at(-1);if(last&&last.dir===dir&&last.s===s)last.length+=3.5;else segments.push({dir,s,length:3.5});}
 let reference={...start};
 function advance(p,dir,s,length){const ds=dir*length,k=Math.tan(s*40*Math.PI/180)/18.45,dt=ds*k,mid=p.theta*Math.PI/180+dt/2;return {u:p.u+ds*Math.cos(mid),w:p.w+ds*Math.sin(mid),theta:wrap(p.theta+dt*180/Math.PI)};}
 for(const {dir,s,length} of segments){const target=-det*s*40;
  let wait=0;while((Math.abs(car.steer-target)>1||Math.abs(car.v)>.2)&&wait++<150&&!collision)step(target,0);
  const startDistance=car.travelled;let run=0;
  while(car.travelled-startDistance<Math.max(0,length-Math.abs(car.v)*car.cfg.speedTauS)&&run++<1000&&!collision){
   let steer=target;
   if(feedback){
    const progress=Math.min(length,car.travelled-startDistance),desired=advance(reference,dir,s,progress);
    const pose=car.pose(),actual=poseLocal({x:pose.x,z:pose.z,theta:pose.yaw*180/Math.PI},f);
    const theta=desired.theta*Math.PI/180,lateral=-(actual.u-desired.u)*Math.sin(theta)+(actual.w-desired.w)*Math.cos(theta);
    const canonical=Math.max(-40,Math.min(40,s*40+dir*(1.2*wrap(desired.theta-actual.theta)-1.5*lateral)));
    steer=-det*canonical;
   }
   step(steer,dir*8);
  }
  let brake=0;while(Math.abs(car.v)>.2&&brake++<150&&!collision)step(target,0);
  reference=advance(reference,dir,s,length);
  if(collision||run>=1000)break;
 }
 let settle=0;while(Math.abs(car.v)>1&&settle++<150&&!collision)step(car.steer,0);
 let rev=0,last=0;for(const x of speeds){if(Math.abs(x)<2)continue;const s=Math.sign(x);if(last&&s!==last)rev++;last=s;}
 return {collision,parkingFull:goal(car.pose(),field),movingReversals:rev,time_s:+(ticks*.01).toFixed(2),final:car.pose()};}
const files=process.argv.slice(2);if(!files.length)throw new Error('Uso: node compare_parking_routes.js barridos.json [...]');
for(const file of files){const d=JSON.parse(fs.readFileSync(file,'utf8'));
 if(!d.camera?.pose||!d.scans.length){console.log(JSON.stringify({file:path.basename(file),status:'sin_deteccion_magenta_o_pose',skipped:'sin referencia o barridos'}));continue;}
 /* Elegir el instante sin mirar el futuro ni la geometria verdadera: 105 cm de
  * avance estimado desde la pose disponible al capturar la referencia. */
 const cameraPose=d.camera.pose,initialHeading=cameraPose.theta*Math.PI/180,forward=[Math.sin(initialHeading),Math.cos(initialHeading)];
 const displacement=p=>dot([p.x-cameraPose.x,p.z-cameraPose.z],forward);
 const choices=d.scans.map(s=>s.estimated_current).filter(Boolean);
 if(!choices.length||Math.max(...choices.map(displacement))<100){console.log(JSON.stringify({file:path.basename(file),status:'avance_insuficiente',skipped:'no alcanzo la zona de prueba'}));continue;}
 const selected=choices.reduce((a,b)=>Math.abs(displacement(a)-105)<Math.abs(displacement(b)-105)?a:b);
 const prefix=d.scans.filter(s=>s.scan_end_s<=selected.t+1e-7);
 const prefixPath=path.join(os.tmpdir(),`autonex-route-prefix-${process.pid}.json`);fs.writeFileSync(prefixPath,JSON.stringify({...d,scans:prefix}));
 const r=spawnSync('python',[path.join(__dirname,'segment_parking_cloud.py'),prefixPath],{encoding:'utf8'});if(r.status!==0)throw new Error(r.stderr);const h=JSON.parse(r.stdout);
 if(h.status!=='hipotesis_no_confirmada'){console.log(JSON.stringify({file:path.basename(file),status:h.status,selected_scan_s:+selected.t.toFixed(2),prefix_scans:prefix.length,skipped:'hipotesis no apta para experimento'}));continue;}
 const field=W.field.randomLayout(d.seed,{direction:d.direction,signs:0,parking:true,startInLot:false,startOnTrack:false});W.field.fitToRobot(field,{length:26,rear:3.775});
 if(d.distractor)field.walls.push({x:(field.lot.a0+field.lot.a1)/2-(d.direction==='ccw'?1:-1)*(field.lot.length/2+1+41),z:field.lot.wallCoord+25,w:2,d:20,rot:0,h:10,kind:'magenta'});
 const center=[(h.markers[0].wall_end[0]+h.markers[1].wall_end[0])/2,(h.markers[0].wall_end[1]+h.markers[1].wall_end[1])/2],n=h.wall.normal;
 let u=[-n[1],n[0]],nominal=d.direction==='ccw'?[1,0]:[-1,0];if(dot(u,nominal)<0)u=u.map(x=>-x);
 const estimated=frame(center,u,n),truth=frame([(field.lot.a0+field.lot.a1)/2,field.lot.wallCoord],nominal,[0,1]);
 const grid=observedGrid(prefix,d.lidar_forward_cm||10);
 const worlds={x:selected.x,z:selected.z,theta:selected.theta},pe=poseLocal(worlds,estimated),pt=poseLocal(worlds,truth);
 const results={};for(const [name,p,f,len] of [['mapa',pe,estimated,h.spacing_cm-2],['verdad',pt,truth,field.lot.length]]){
  const a=plan(p,len);const k=a.found&&!a.atGoal?ideal(a,p,f,field,grid):null;
  results[name]={found:a.found,atGoal:a.atGoal,expanded:a.expanded,clock_ms:a.clock_ms,plannedReversals:countRev(a.arcs),arcs:a.arcs.length,idealOverlapSamples:k?.overlapSamples??null,observedObstacleSamples:k?.observedObstacleSamples??null,idealMinTrueClearanceCm:k?.minTrueClearanceCm??null,idealParkingFull:k?goal(k.final,field):null,dynamic:a.found&&!a.atGoal?dynamic(a,p,f,field):null,oracleFeedback:a.found&&!a.atGoal?dynamic(a,p,f,field,true):null};
 }
 console.log(JSON.stringify({file:path.basename(file),direction:d.direction,seed:d.seed,offset_cm:d.offset_cm,status:h.status,selected_scan_s:+selected.t.toFixed(2),prefix_scans:prefix.length,observedGridCells:grid.size,estimated_pose:pe,truth_frame_pose:pt,results}));
}
