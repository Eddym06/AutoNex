/* Evaluación externa: reconstruye el primer impacto real de cada rayo emitido.
 * No entrega geometría verdadera ni datos nuevos al controlador. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const web=path.join(__dirname,'../web'),W=require(path.join(web,'sim.js'));
require(path.join(web,'core.js'));
const G=W.geometry, round=x=>+x.toFixed(2);
function firstHit(ox,oz,dx,dz,boxes,height,min,max) {
 let best=Infinity,hit=null,object=-1;
 for(let i=0;i<boxes.length;i++) {
  const b=boxes[i]; if(b.h<height)continue;
  const h=G.rayBox(ox,oz,dx,dz,b);
  if(h&&h.t<best){best=h.t;hit=h;object=i;}
 }
 return hit&&best>=min&&best<=max?{object,hit,x:ox+dx*best,z:oz+dz*best}:null;
}
function faceBin(b,h,x,z) {
 const a=G.boxAxes(b),rx=x-b.x,rz=z-b.z;
 const side=h.nx*a.right[0]+h.nz*a.right[1],front=h.nx*a.forward[0]+h.nz*a.forward[1];
 const right=Math.abs(side)>Math.abs(front), face=right?(side>0?'right':'left'):(front>0?'front':'back');
 const along=right?rx*a.forward[0]+rz*a.forward[1]:rx*a.right[0]+rz*a.right[1];
 const length=right?b.d:b.w,n=Math.max(1,Math.ceil(length));
 const bin=Math.min(n-1,Math.max(0,Math.floor((along+length/2)/length*n)));
 return {face,bin};
}
function allBins(boxes) {
 return boxes.map(b=>{
  const a=G.boxAxes(b), result={};
  for(const face of ['left','right','front','back']) {
   const right=face==='left'||face==='right',length=right?b.d:b.w,n=Math.max(1,Math.ceil(length));
   const fixed=(face==='right'||face==='front'?1:-1)*(right?b.w:b.d)/2;
   result[face]=Array.from({length:n},(_,i)=>{
    const along=-length/2+(i+.5)*length/n,x=right?fixed:along,z=right?along:fixed;
    return {x:b.x+x*a.right[0]+z*a.forward[0],z:b.z+x*a.right[1]+z*a.forward[1],visible:false,valid:false};
   });
  }
  return result;
 });
}
function stats(array) {
 if(!array.length)return null; array.sort((a,b)=>a-b);
 const q=p=>round(array[Math.ceil(array.length*p)-1]);
 return {n:array.length,p50_cm:q(.5),p95_cm:q(.95),max_cm:q(1)};
}
function nearby(x,z,map,radius) {
 const col=Math.floor((x-map.originCm)/map.cellCm),row=Math.floor((z-map.originCm)/map.cellCm),rad=Math.ceil(radius/map.cellCm)+1;
 for(let r=Math.max(0,row-rad);r<=Math.min(map.size-1,row+rad);r++)for(let c=Math.max(0,col-rad);c<=Math.min(map.size-1,col+rad);c++)
  if((map.cells[r*map.size+c]&1)&&Math.hypot(map.originCm+(c+.5)*map.cellCm-x,map.originCm+(r+.5)*map.cellCm-z)<=radius)return true;
 return false;
}
function summarize(bins,map) {
 const out={};
 for(const [face,all] of Object.entries(bins)) {
  const vis=all.filter(b=>b.visible),valid=vis.filter(b=>b.valid),near=valid.filter(b=>nearby(b.x,b.z,map,3.75));
  out[face]={total_bins:all.length,visible_bins:vis.length,valid_return_bins:valid.length,
   valid_bins_with_map_near:near.length,unobserved_bins:all.length-vis.length};
 }
 return out;
}
function nearestMap(x,z,map) {
 let best=Infinity;
 for(let row=0;row<map.size;row++)for(let col=0;col<map.size;col++)
  if(map.cells[row*map.size+col]&1)best=Math.min(best,Math.hypot(map.originCm+(col+.5)*map.cellCm-x,map.originCm+(row+.5)*map.cellCm-z));
 return best;
}
function nearestVisibleStats(bins,map) {
 const values=[];for(const all of Object.values(bins))for(const b of all)if(b.valid)values.push(nearestMap(b.x,b.z,map));
 return stats(values);
}
/* Prueba mínima: el primer obstáculo tapa al segundo y se clasifica su cara. */
{
 const boxes=[{x:0,z:10,w:4,d:2,h:10},{x:0,z:20,w:4,d:2,h:10}];
 const h=firstHit(0,0,0,1,boxes,6,5,100); assert.equal(h.object,0);
 assert.equal(faceBin(boxes[0],h.hit,h.x,h.z).face,'back');
 assert.equal(firstHit(0,0,0,-1,boxes,6,5,100),null);
}
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm'))),results=[];
 for(const [direction,seed] of [['cw',1],['ccw',3]]) {
  const field=W.field.randomLayout(seed,{direction,signs:3,parking:true,startInLot:true});
  const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26});
  const boxes=sim.renderLidarBoxes(),bins=allBins(boxes),lidar=sim.lidar,original=lidar.measure;
  const residuals={walls:[],limiter0:[],limiter1:[],signs:[]};
  let total=0,eligible=0,valid=0;
  lidar.measure=function(index,pose,bs) {
   /* Captura la primera gaussiana ya consumida para jitter sin alterar el RNG. */
   const rng=this.rng,gauss=rng.gauss;let jitter=null;
   rng.gauss=function(){const v=gauss();if(jitter===null)jitter=v;return v;};
   let point;try{point=original.call(this,index,pose,bs);}finally{rng.gauss=gauss;}
   total++;
   if(!sim.started || !sim.cmd || !sim.cmd.arena || !sim.cmd.arena.valid)return point;
   const c=this.cfg,angle=-180+index*360/c.rays;
   if(c.selfMask.some(([a,b])=>angle>=a&&angle<=b))return point;
   const ox=pose.x+Math.sin(pose.yaw)*c.mountForward+Math.cos(pose.yaw)*c.mountLateral;
   const oz=pose.z+Math.cos(pose.yaw)*c.mountForward-Math.sin(pose.yaw)*c.mountLateral;
   const a=pose.yaw+(angle+(jitter===null?0:jitter*c.angleJitterDeg))*Math.PI/180;
   const h=firstHit(ox,oz,Math.sin(a),Math.cos(a),bs,c.height,c.minCm,c.maxCm);if(!h)return point;
   eligible++;const bin=faceBin(bs[h.object],h.hit,h.x,h.z),entry=bins[h.object][bin.face][bin.bin];
   entry.visible=true;
   if(point.valid){
    valid++;entry.valid=true;
    const b=bs[h.object], li=sim.field.lot.limiters.indexOf(b);
    const group=li>=0?'limiter'+li:b.kind==='wall'?'walls':'signs';
    residuals[group].push(Math.abs(point.distance-h.hit.t));
   }
   return point;
  };
  sim.run(180); const map=core.traceMap(),walls=[],limiters=[];
  for(let i=0;i<boxes.length;i++) {
   const entry={index:i,faces:summarize(bins[i],map),valid_surface_bin_to_nearest_map:nearestVisibleStats(bins[i],map)};
   if(sim.field.lot.limiters.includes(boxes[i])) {
    const b=boxes[i],axis=G.boxAxes(b),v=b.w>=b.d?axis.right:axis.forward,len=Math.max(b.w,b.d);
    entry.true_long_axis_ends_to_nearest_map_cm=[-1,1].map(sign=>round(nearestMap(b.x+sign*v[0]*len/2,b.z+sign*v[1]*len/2,map)));
   }
   if(boxes[i].kind==='wall')walls.push(entry);
   if(sim.field.lot.limiters.includes(boxes[i]))limiters.push(entry);
  }
  const counts={};for(const [k,v] of Object.entries(residuals))counts[k]=stats(v);
  const out={direction,seed,seconds:round(sim.t),end_reason:sim.result().endReason,scans:map.scans,
   rays:{total,eligible_after_global_pose:eligible,valid_after_global_pose:valid},
   range_noise_with_true_ray_pose:counts,walls,limiters};
  results.push(out);console.log(JSON.stringify(out));
 }
 if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(results,null,2)+'\n');
})().catch(e=>{console.error(e);process.exitCode=1;});
