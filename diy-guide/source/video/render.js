const {chromium}=require('playwright');
(async()=>{
 const [url,outdir,fps,only]=process.argv.slice(2);
 const b=await chromium.launch({args:['--use-angle=swiftshader','--enable-unsafe-swiftshader','--ignore-gpu-blocklist']});
 const p=await b.newPage({viewport:{width:1280,height:720}});
 p.on('console',m=>console.log('console:',m.text())); p.on('pageerror',e=>console.log('err:',e.message));
 await p.goto(url); await p.waitForFunction('window.ready===true',{timeout:60000});
 const total=await p.evaluate('window.TOTAL');
 const times = only? only.split(',').map(Number) : Array.from({length:Math.round(total*fps)},(_,i)=>i/fps);
 let i=0; const t0=Date.now();
 for(const t of times){ await p.evaluate(t=>window.renderAt(t),t);
   await p.screenshot({path:`${outdir}/f${String(i).padStart(5,'0')}.jpg`,type:'jpeg',quality:92}); i++;
   if(i%100===0) console.log(i,'frames',((Date.now()-t0)/1000).toFixed(0)+'s'); }
 await b.close();
})();
