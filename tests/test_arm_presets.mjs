import assert from 'node:assert/strict';
import fs from 'node:fs';
const root=new URL('..',import.meta.url).pathname.replace(/\/$/,'');
const {READY,CALIBRATION,fk,ik,figureEight,validateModel,PreviewPlayer,followAxis}=await import(root+'/arm_web/presets.js');
validateModel(JSON.parse(fs.readFileSync(root+'/arm_assets/model.json')));
let q=READY, maxError=0;
for(let i=0;i<=400;i++) {const p=figureEight(i/40);const next=ik(p,q);assert.ok(next,'reachable point '+i);maxError=Math.max(maxError,Math.hypot(...fk(next).map((v,k)=>v-p[k])));q=next;}
assert.ok(maxError<1e-7);assert.deepEqual(figureEight(0),figureEight(10));
const player=new PreviewPlayer();player.select('ready');for(let i=0;i<300;i++)player.step(1/60);assert.ok(player.q.every((v,i)=>Math.abs(Math.sin(v-READY[i]))<1e-9));
player.select('calibration',player.q);for(let i=0;i<300;i++)player.step(1/60);assert.ok(player.q.every((v,i)=>Math.abs(Math.sin(v-CALIBRATION[i]))<1e-9));
player.select('figure8',player.q);for(let i=0;i<600;i++){const previous=[...player.q];player.step(1/60);assert.ok(player.q.every((v,k)=>Math.abs(v-previous[k])<=Math.PI/180*[45,30,45][k]/60+1e-12));}
player.stop();const stopped=[...player.q];player.step(1);assert.deepEqual(player.q,stopped);
assert.equal(followAxis(.2,0,-5,5).blocked,true);assert.equal(followAxis(.01,0,-5,5).blocked,false);assert.equal(followAxis(.01,null,-5,5).blocked,true);
const bad=JSON.parse(fs.readFileSync(root+'/arm_assets/model.json'));bad.joints[1].origin[0]+=1;assert.throws(()=>validateModel(bad));
assert.throws(()=>figureEight(1,0));
console.log(JSON.stringify({points:401,maxErrorMm:maxError,poses:true,rateBound:true,stop:true,physicalBounds:true,modelMismatchBlocked:true}));
