// Geometry/path conventions adapted from ANGELOREYES-afk/arm_3dof_sim at 3719fc5.

const RAD=Math.PI/180, TAU=2*Math.PI;
export const READY=[0,-20,302].map(v=>v*RAD);
export const CALIBRATION=[0,0,360].map(v=>v*RAD);
const A={h:137.2005,L2:350.5292012616316,L3:268.9043008695935,d:-58.10240538849397,sign:[-1,1,1],offset:[-1.6292967086251973,0.9239930941086216,-3.9331327806566088]};
const near=(a,ref)=>a+TAU*Math.round((ref-a)/TAU);
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
export function fk(q) {
  const [yaw,shoulder,elbow]=q.map((v,i)=>A.sign[i]*v+A.offset[i]);
  const radius=A.L2*Math.sin(shoulder)+A.L3*Math.sin(shoulder+elbow);
  return [radius*Math.cos(yaw)-A.d*Math.sin(yaw),radius*Math.sin(yaw)+A.d*Math.cos(yaw),A.h+A.L2*Math.cos(shoulder)+A.L3*Math.cos(shoulder+elbow)];
}
export function ik([x,y,z],reference) {
  const rho2=x*x+y*y;if(rho2<A.d*A.d)return null;
  const rr=Math.sqrt(rho2-A.d*A.d),bearing=Math.atan2(y,x),out=[];
  for(const radius of [rr,-rr]) {
    const yaw=bearing-Math.atan2(A.d,radius),height=z-A.h;
    const cosine=(radius*radius+height*height-A.L2*A.L2-A.L3*A.L3)/(2*A.L2*A.L3);
    if(cosine < -1 || cosine > 1)continue;
    for(const sign of [1,-1]) {
      const elbow=sign*Math.acos(cosine);
      const shoulder=Math.atan2(radius,height)-Math.atan2(A.L3*Math.sin(elbow),A.L2+A.L3*Math.cos(elbow));
      const q=[yaw,shoulder,elbow].map((v,i)=>near(A.sign[i]*(v-A.offset[i]),reference[i]));
      if(Math.abs(q[0])<=Math.PI/2)out.push(q);
    }
  }
  out.sort((a,b)=>a.reduce((s,v,i)=>s+(v-reference[i])**2,0)-b.reduce((s,v,i)=>s+(v-reference[i])**2,0));
  return out[0]||null;
}
const readyTool=fk(READY),radius=Math.hypot(readyTool[0],readyTool[1]);
const centre=[readyTool[0]*(radius-170)/radius,readyTool[1]*(radius-170)/radius,readyTool[2]-60];
const across=[-readyTool[1]/radius,readyTool[0]/radius];
export function figureEight(seconds,period=10) {
  if(!Number.isFinite(seconds)||!Number.isFinite(period)||period<=0)throw new Error('Path time and period must be finite; period must be positive');
  const phase=TAU*(seconds%period)/period,a=150*Math.sin(phase);
  return [centre[0]+across[0]*a,centre[1]+across[1]*a,centre[2]+70*Math.sin(2*phase)];
}
export function validateModel(model) {
  const expected=[{origin:[0,0,64.3002],axis:[.000003,.000001,-1]},{origin:[26.856,-1.5733,137.2005],axis:[.998289,-.058467,.000001]},{origin:[-35.5252,-278.1261,348.4428],axis:[.998289,-.058468,.000014]}];
  if(!model?.joints||model.joints.length<3)throw new Error('Load the CAD model before selecting a path');
  expected.forEach((joint,i)=>{
    for(const [field,tolerance] of [['origin',.001],['axis',.00002]]) {
      if(!Array.isArray(model.joints[i][field])||model.joints[i][field].length!==3||model.joints[i][field].some((v,k)=>!Number.isFinite(v)||Math.abs(v-joint[field][k])>tolerance))throw new Error('Preset geometry does not match this model; rebuild its kinematics first');
    }
  });
}
export class PreviewPlayer {
  constructor(){this.q=[0,0,0];this.selected=false;this.running=false;this.kind=null;this.elapsed=0;this.goal=null;this.period=10;}
  select(kind,current=[0,0,0]) {
    if(!['ready','calibration','figure8'].includes(kind)||!Array.isArray(current)||current.length!==3||!current.every(Number.isFinite))throw new Error('Invalid preview selection');
    this.q=[...current];this.selected=true;this.running=true;this.kind=kind;this.elapsed=0;
    const pose=kind==='ready'?READY:CALIBRATION;
    this.goal=kind==='figure8'?ik(figureEight(0),current):pose.map((v,i)=>near(v,current[i]));
    if(!this.goal)throw new Error('Figure-8 start is unreachable');
    this.approaching=kind==='figure8';
  }
  stop(){this.running=false;}
  clear(){this.stop();this.selected=false;}
  step(dt){
    if(!this.running)return this.q;
    if(!Number.isFinite(dt)||dt<0)throw new Error('Invalid preview timestep');
    dt=Math.min(dt,.05);
    if(this.kind==='figure8'&&!this.approaching){
      this.elapsed+=dt;
      this.goal=ik(figureEight(this.elapsed,this.period),this.q);
      if(!this.goal){this.stop();throw new Error('Figure-8 target is outside this model’s reachable preview');}
    }
    const difference=this.goal.map((v,i)=>v-this.q[i]);
    const duration=Math.max(...difference.map((v,i)=>Math.abs(v)/([45,30,45][i]*RAD)));
    this.q=this.q.map((v,i)=>v+difference[i]*(duration?Math.min(1,dt/duration):1));
    if(this.kind==='figure8'&&this.approaching&&duration<=dt)this.approaching=false;
    if(this.kind!=='figure8'&&duration<=dt)this.stop();
    return this.q;
  }
}
export function followAxis(target,measured,lowerDeg,upperDeg) {
  if(![target,measured,lowerDeg,upperDeg].every(Number.isFinite)||target<lowerDeg*RAD||target>upperDeg*RAD)return {axis:0,blocked:true};
  return {axis:clamp((target-measured)/(2*RAD),-1,1),blocked:false};
}
