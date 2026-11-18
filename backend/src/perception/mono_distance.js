// 1:1 port of the distance logic in object_detection_new_babu.py (same names, same numbers).

export const focalFromFov = (frameWidthPx, hfovDeg) => frameWidthPx / 2 / Math.tan((hfovDeg * Math.PI) / 360);

// estimate_obstacle_distance: Z = H_class * f / h_px
export function knownHeightDistance(bboxHeightPx, className, focalPx, knownHeights, defaultHeight) {
  const realHeight = knownHeights[className] ?? defaultHeight;
  if (realHeight == null || bboxHeightPx <= 0 || !focalPx) return null;
  return (realHeight * focalPx) / bboxHeightPx;
}

// --calibrate CLASS DISTANCE_M: implied focal length from a known placement.
export const calibrateFocal = (bboxHeightPx, distanceM, realHeightM) => (bboxHeightPx * distanceM) / realHeightM;

export const isTruncated = (y1, y2, frameHeight, margin = 2) => y1 <= margin || y2 >= frameHeight - margin;

export function horizontalZone(xCenter, frameWidth) {
  if (xCenter < frameWidth / 3) return 'Left';
  if (xCenter > (2 * frameWidth) / 3) return 'Right';
  return 'Center';
}

export function obstacleStatus(distanceM, flagM, warnM) {
  if (distanceM == null) return 'UNKNOWN';
  if (distanceM <= flagM) return 'FLAGGED';
  if (distanceM <= warnM) return 'WARNING';
  return 'SAFE';
}

export function median(xs) {
  const s = [...xs].sort((a, b) => a - b), m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}
