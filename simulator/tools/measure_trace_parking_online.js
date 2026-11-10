/* Diagnóstico en vivo con el controlador C; la verdad solo se usa para evaluar. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),web=path.join(__dirname,'../web');
const W=require(path.join(web,'sim.js'));require(path.join(web,'core.js'));
const dist=(p,q)=>Math.hypot(p.x-q.x,p.z-q.z);
(async()=>{
 const core=await W.core.load(fs.readFileSync(path.join(web,'wro_core.wasm'))),out=[];
 for(const direction of (process.env.TRACE_DIR?[process.env.TRACE_DIR]:['ccw','cw'])) {
  const seed=Number(process.env.TRACE_SEED||1),signs=Number(process.env.TRACE_SIGNS||0);
  const field=W.field.randomLayout(seed,{direction,signs,parking:true,feasible:true});
  const sim=new W.sim.Simulation(core,{field,seed,robot:W.robot.PROFILES.largo26}),events=[],errors=[];
  let previous='';
  while(sim.t<180&&!sim.finished){
   sim.run(.5);const r=core.traceParking(),key=`${r.seen}/${r.status}`;
   let err=null;
   if(r.status===1){
      const truth=sim.field.lot.limiters.map(b=>({x:b.x,z:sim.field.lot.wallCoord}));
      const d1=[dist(r.points[0],truth[0]),dist(r.points[1],truth[1])],d2=[dist(r.points[0],truth[1]),dist(r.points[1],truth[0])];
      err=(Math.max(...d1)<Math.max(...d2)?d1:d2).map(n=>+n.toFixed(2));errors.push(Math.max(...err));
   }
   if(key!==previous){
    const e={t:+sim.t.toFixed(2),seen:r.seen,status:r.status,wallCells:r.wallCells,alternatives:r.alternatives,
      cameraPose:r.seen?r.cameraPose:null,bearing:r.seen?r.bearing:null};
    if(err)e.errors_cm=err;
    events.push(e);previous=key;
   }
  }
  errors.sort((a,b)=>a-b);
  const candidate_errors=errors.length?{samples:errors.length,p95_cm:errors[Math.ceil(.95*errors.length)-1],max_cm:errors.at(-1)}:null;
  const memory=core.traceParking().memory,truth=sim.field.lot.limiters.map(b=>({x:b.x,z:sim.field.lot.wallCoord}));
  let memory_errors=null;
  if(memory){const a=[dist(memory[0],truth[0]),dist(memory[1],truth[1])],b=[dist(memory[0],truth[1]),dist(memory[1],truth[0])];
    memory_errors=(Math.max(...a)<Math.max(...b)?a:b).map(n=>+n.toFixed(2));}
  const result={direction,seed,signs,t:+sim.t.toFixed(2),end:sim.result().endReason,events,candidate_errors,memory_errors};out.push(result);console.log(JSON.stringify(result));
  if(process.env.TRACE_MAX_ERROR_CM){
    const limit=Number(process.env.TRACE_MAX_ERROR_CM);
    if(!candidate_errors||candidate_errors.max_cm>limit||!memory_errors||Math.max(...memory_errors)>limit)
      throw new Error(`Candidato o recuerdo incorrecto: ${JSON.stringify(result)}`);
  }
 }
 if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(out,null,2)+'\n');
})().catch(e=>{console.error(e);process.exitCode=1;});
