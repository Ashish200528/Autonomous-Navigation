// ArUco in its own worker thread (adaptive threshold + contours on 640x480 is too slow for the
// main loop on a Pi). Receives {t, pan, Y, W, H, intr}, replies {t, pan, markers, ms, Y}.
import { parentPort, workerData } from 'node:worker_threads';
import { createAruco } from './aruco.js';

const detect = createAruco(workerData.cfg);
parentPort.postMessage({ ready: true });
parentPort.on('message', ({ t, pan, Y, W, H, intr }) => {
  const t0 = performance.now();
  const markers = detect(new Uint8Array(Y), W, H, intr);
  parentPort.postMessage({ t, pan, markers, ms: performance.now() - t0, Y }, [Y]);
});
