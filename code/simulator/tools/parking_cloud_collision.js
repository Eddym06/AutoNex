/* Herramienta experimental: celdas con retornos repetidos; no representa espacio libre. */
'use strict';
const path=require('node:path'),W=require(path.join(__dirname,'../web/sim.js'));
function observedGrid(prefix,mount){const counts=new Map();for(const scan of prefix){const p=scan.estimated_current,theta=p.theta*Math.PI/180;
 for(const r of scan.rays){if(!r[3]||!(r[2]>0)||r[2]>200)continue;const a=theta+r[1]*Math.PI/180;
  const x=Math.round(p.x+Math.sin(theta)*mount+Math.sin(a)*r[2]),z=Math.round(p.z+Math.cos(theta)*mount+Math.cos(a)*r[2]);
  const k=`${x},${z}`;counts.set(k,(counts.get(k)||0)+1);
 }}return new Set([...counts].filter(([,n])=>n>=2).map(([k])=>k));}
function observedCollision(body,grid){
 const corners=W.geometry.corners(body),minX=Math.floor(Math.min(...corners.map(p=>p[0]))-1),maxX=Math.ceil(Math.max(...corners.map(p=>p[0]))+1),minZ=Math.floor(Math.min(...corners.map(p=>p[1]))-1),maxZ=Math.ceil(Math.max(...corners.map(p=>p[1]))+1);
 const axes=W.geometry.boxAxes(body);
 for(let x=minX;x<=maxX;x++)for(let z=minZ;z<=maxZ;z++){if(!grid.has(`${x},${z}`))continue;const dx=x-body.x,dz=z-body.z;
  if(Math.abs(dx*axes.right[0]+dz*axes.right[1])<=body.w/2+.5&&Math.abs(dx*axes.forward[0]+dz*axes.forward[1])<=body.d/2+.5)return true;
 }return false;
}
function boxDistance(a,b){
 if(W.geometry.overlapBoxes(a,b))return 0;
 const x=W.geometry.corners(a),y=W.geometry.corners(b);let min=Infinity;
 const segment=(p,a,b)=>{const dx=b[0]-a[0],dz=b[1]-a[1],t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dz)/(dx*dx+dz*dz)));return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dz);};
 for(let i=0;i<4;i++)for(let j=0;j<4;j++)min=Math.min(min,segment(x[i],y[j],y[(j+1)%4]),segment(y[j],x[i],x[(i+1)%4]));
 return min;
}
module.exports={observedGrid,observedCollision,boxDistance};
