// Service worker – praca offline. Wszystkie zasoby lokalne (bez CDN).
// Zmiana dowolnego pliku aplikacji = podbicie CACHE.
const CACHE = 'x4f-drift-v1.9.1';
const ASSETS = [
  './', './index.html', './manifest.webmanifest', './css/app.css',
  './vendor/uPlot.iife.min.js', './vendor/uPlot.min.css',
  './js/ui.js', './js/transport.js', './js/protocol.js', './js/format.js', './js/analysis.js',
  './js/estimator.js', './js/storage.js', './js/charts.js', './js/sync.js', './js/drift.js', './js/axes.js', './js/sound.js',
  './icons/icon-192.png', './icons/icon-512.png', './icons/icon-maskable-512.png',
  './demo/index.json', './demo/mode1_grip.bin', './demo/mode1_slip.bin', './demo/mode3_grip.bin',
  './demo/mode3_slip.bin', './demo/mode1_slip_noedges.bin', './demo/mode1_lifted.bin',
  './demo/races.json', './demo/race_oval.bin', './demo/race_recovered.bin',
  './demo/drift.json', './demo/drift_eights.bin', './demo/drift_eights_rev.bin',
];

self.addEventListener('install', (e) => {
  // cache: 'reload' – z pominięciem pamięci HTTP przeglądarki (GitHub Pages: max-age 600),
  // inaczej nowa wersja mogła się zainstalować ze starymi plikami.
  e.waitUntil(caches.open(CACHE)
    .then((c) => c.addAll(ASSETS.map((u) => new Request(u, { cache: 'reload' }))))
    .then(() => self.skipWaiting()));
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

// Najpierw sieć (z internetem zawsze najnowsza wersja), po 3 s albo bez sieci –
// pamięć podręczna (działanie na torze bez zasięgu).
self.addEventListener('fetch', (e) => {
  if (e.request.method !== 'GET' || new URL(e.request.url).origin !== location.origin) return;
  e.respondWith((async () => {
    const cache = await caches.open(CACHE);
    const hit = await cache.match(e.request, { ignoreSearch: true });
    const net = fetch(e.request, { cache: 'no-cache' }).then((res) => {
      if (res.ok) cache.put(e.request, res.clone());
      return res;
    });
    if (!hit) return net;
    const timeout = new Promise((r) => setTimeout(() => r(hit), 3000));
    try { return await Promise.race([net, timeout]); } catch (err) { return hit; }
  })());
});
