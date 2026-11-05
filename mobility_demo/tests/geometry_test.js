'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),G=require('../viewer/geometry.js');
const samples=JSON.parse(fs.readFileSync(0,'utf8'));let maxError=0;
for(const sample of samples){const body=G.body(sample.frame);for(let i=0;i<2;i++){const d=Math.hypot(...body.legs[i].ankle.map((x,k)=>x-sample.feet[i][k]));maxError=Math.max(maxError,d);assert(d<.005,'Rendered ankle differs from actual simulator state by '+d+' m');}}
assert(samples.length>=30);console.log('PASS actual-state geometry samples='+samples.length+' max_error_m='+maxError);
