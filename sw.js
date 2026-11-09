const C='sb-re-9',A=['./','index.html','manifest.webmanifest','icon-192.png','icon-512.png','icon-maskable-512.png','apple-touch-icon.png','badge-96.png','logo-mark.svg'];
self.oninstall=e=>{self.skipWaiting();e.waitUntil(caches.open(C).then(c=>c.addAll(A)))};
self.onactivate=e=>e.waitUntil(caches.keys().then(k=>Promise.all(k.filter(x=>x!==C).map(x=>caches.delete(x)))).then(()=>self.clients.claim()));
// Network first so a new deploy shows up, cache as the offline fallback.
self.onfetch=e=>{if(e.request.method!=='GET'||new URL(e.request.url).origin!==location.origin)return;
 e.respondWith(fetch(e.request).then(r=>{if(r.ok){const c=r.clone();caches.open(C).then(x=>x.put(e.request,c))}return r}).catch(()=>caches.match(e.request).then(r=>r||caches.match('index.html'))))};
// Tapping a phone notification, or one of its buttons, talks back to the open page.
self.onnotificationclick=e=>{e.notification.close();const a=e.action;
 e.waitUntil(self.clients.matchAll({type:'window',includeUncontrolled:true}).then(list=>{
  const msg=a==='snooze'?{type:'snooze'}:a==='stop'?{type:'cmd',cmd:'pump off'}:null;
  const w=list.find(c=>'focus' in c)||null;
  if(w){if(msg)w.postMessage(msg);return a?undefined:w.focus()}
  if(!a)return self.clients.openWindow('./')}))};
