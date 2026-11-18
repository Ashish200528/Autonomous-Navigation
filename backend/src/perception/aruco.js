// ArUco detection on the camera's luma plane (js-aruco2, DICT_7X7_1000 from OpenCV) + pose by
// POSIT. Same outputs as object_detection_new_babu.py: dist = |t|, angle_deg = atan2(tx, tz).
import { createRequire } from 'node:module';
import { horizontalZone } from './mono_distance.js';

const require = createRequire(import.meta.url);
const { AR } = require('js-aruco2');
const { CV } = require('js-aruco2/src/cv');
const { POS } = require('js-aruco2/src/posit1');
require('js-aruco2/src/dictionaries/aruco_7x7_1000.js');

export function createAruco(cfg) {
  // js-aruco2's default tolerance for this dictionary accepts ~13 wrong bits, which reads
  // marker-like texture inside a real marker as another id; accept < max_hamming bit errors only.
  const det = new AR.Detector({ dictionaryName: cfg.aruco.dictionary, maxHammingDistance: cfg.aruco.max_hamming });
  const size = cfg.aruco.marker_size_m;

  // Y: Uint8Array luma (W*H). intr: {f, cx, cy}. Skips js-aruco2's RGBA->grey step: the Y plane is grey already.
  return function detect(Y, W, H, intr) {
    det.grey = new CV.Image(W, H, Y);
    CV.adaptiveThreshold(det.grey, det.thres, 2, 7);
    det.contours = CV.findContours(det.thres, det.binary);
    let cand = det.findCandidates(det.contours, W * 0.01, 0.05, 10);
    cand = det.notTooNear(det.clockwiseCorners(cand), 10);
    const posit = new POS.Posit(size, intr.f);
    return det.findMarkers(det.grey, cand, 49).map((m) => {
      const pts = m.corners.map((c) => ({ x: c.x - intr.cx, y: intr.cy - c.y })); // POSIT: centred, y up
      const [tx, ty, tz] = posit.pose(pts).bestTranslation;
      const u = m.corners.reduce((s, c) => s + c.x, 0) / 4, v = m.corners.reduce((s, c) => s + c.y, 0) / 4;
      return {
        id: m.id, dist: Math.hypot(tx, ty, tz), angle_deg: tz ? (Math.atan2(tx, tz) * 180) / Math.PI : 0,
        center: [u, v], corners: m.corners.map((c) => [c.x, c.y]), zone: horizontalZone(u, W), hamming: m.hammingDistance,
      };
    });
  };
}

// Bit pattern of a marker id (for tests / printing): '0'/'1' string, row-major, 1 = white.
export const markerBits = (cfg, id) => new AR.Dictionary(cfg.aruco.dictionary).codeList[id];
