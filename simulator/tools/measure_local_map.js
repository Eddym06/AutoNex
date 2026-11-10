/* Mide cercania de celdas ocupadas del mapa local a solidos verdaderos con coche inmovil.
 * Mide cobertura de rayos ideales visibles: no mide superficies ocultas ni mapa global. */
'use strict';
const fs=require('node:fs'), path=require('node:path');
const web=path.join(__dirname,'../web'), W=require(path.join(web,'sim.js')); require(path.join(web,'core.js'));
function distanceToBox(x,z,b) { const a=W.geometry.boxAxes(b), dx=x-b.x,dz=z-b.z; const qx=dx*a.right[0]+dz*a.right[1], qz=dx*a.forward[0]+dz*a.forward[1]; return Math.hypot(Math.max(0,Math.abs(qx)-b.w/2),Math.max(0,Math.abs(qz)-b.d/2)); }
const stats=a=>{a.sort((x,y)=>x-y);const v=q=>+a[Math.ceil(a.length*q)-1].toFixed(3);return {n:a.length,p50:v(.5),p95:v(.95),max:v(1),above_2_5_cm:a.filter(x=>x>2.5).length,above_5_cm:a.filter(x=>x>5).length};};
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm')));
 for(const direction of ['ccw','cw']) {
  const seed=1, field=W.field.randomLayout(seed,{direction,signs:0,parking:true,startOnTrack:true});
  const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
  const pose=sim.robot.pose(), boxes=sim.renderLidarBoxes(), distances=[]; let occupied=0, valid=0; const coverage={eligible:0,validReturn:0,occupiedExact:0,occupiedNeighbor:0};
  for(let scan=0;scan<50;scan++) {
   const pts=[];for(let i=0;i<sim.lidar.cfg.rays;i++)pts.push(sim.lidar.measure(i,pose,boxes));
   valid+=pts.filter(p=>p.valid).length;core.scan(pts,(scan+1)*100000);
   const map=core.map(), n=core.mapSize(), cell=core.mapCellCm();
   const cfg=sim.lidar.cfg, ox=pose.x+Math.sin(pose.yaw)*cfg.mountForward+Math.cos(pose.yaw)*cfg.mountLateral;
   const oz=pose.z+Math.cos(pose.yaw)*cfg.mountForward-Math.sin(pose.yaw)*cfg.mountLateral;
   for(let i=0;i<cfg.rays;i++) {
    const deg=-180+i*360/cfg.rays;if(cfg.selfMask.some(([a,b])=>deg>=a&&deg<=b))continue;
    const angle=deg*Math.PI/180,worldAngle=pose.yaw+angle,dx=Math.sin(worldAngle),dz=Math.cos(worldAngle);
    let best=Infinity;for(const b of boxes){if(b.h<cfg.height)continue;const hit=W.geometry.rayBox(ox,oz,dx,dz,b);if(hit&&hit.t<best)best=hit.t;}
    if(!Number.isFinite(best)||best<cfg.minCm||best>cfg.maxCm)continue;
    const lx=Math.sin(angle)*best,lf=cfg.mountForward+Math.cos(angle)*best;
    const col=Math.round(n/2+lx/cell),row=Math.round(n/2-lf/cell);
    if(row<1||row>=n-1||col<1||col>=n-1)continue;
    coverage.eligible++;if(pts[i].valid)coverage.validReturn++;
    if(map[row*n+col]&2)coverage.occupiedExact++;
    let neighbor=false;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++)if(map[(row+dy)*n+col+dx]&2)neighbor=true;
    if(neighbor)coverage.occupiedNeighbor++;
   }
   for(let row=0;row<n;row++)for(let col=0;col<n;col++) {
    if(!(map[row*n+col]&2))continue; occupied++;
    const x=(col-n/2)*cell, f=(n/2-row)*cell;
    const wx=pose.x+Math.cos(pose.yaw)*x+Math.sin(pose.yaw)*f;
    const wz=pose.z-Math.sin(pose.yaw)*x+Math.cos(pose.yaw)*f;
    distances.push(Math.min(...boxes.map(b=>distanceToBox(wx,wz,b))));
   }
  }
  console.log(JSON.stringify({direction,scans:50,pose,cell_cm:core.mapCellCm(),valid_returns:valid,visible_ideal_rays:coverage,occupied_samples:occupied,distance_from_cell_center_to_solid_cm:stats(distances)}));
 }
})().catch(e=>{console.error(e);process.exitCode=1});
