// YUV420p (ffmpeg output) -> RGB in one pass. Callers pass preallocated outputs so the
// hot path allocates nothing.

const clamp8 = (v) => (v < 0 ? 0 : v > 255 ? 255 : v);

// BT.601 limited range (what USB webcams deliver through ffmpeg).
function yuvPixel(Y, U, V, out, o) {
  const c = 1.164 * (Y - 16), d = U - 128, e = V - 128;
  out[o] = clamp8(c + 1.596 * e);
  out[o + 1] = clamp8(c - 0.392 * d - 0.813 * e);
  out[o + 2] = clamp8(c + 2.017 * d);
}

// Downscaled RGB image {w, h, data} for lanes, HUD and JPEG. Nearest sampling.
export function yuvToRgb(yuv, W, H, w, h, out = new Uint8Array(w * h * 3)) {
  const uOff = W * H, vOff = uOff + (W * H) / 4, sx = W / w, sy = H / h;
  for (let y = 0, o = 0; y < h; y++) {
    const Y0 = Math.floor(y * sy);
    for (let x = 0; x < w; x++, o += 3) {
      const X0 = Math.floor(x * sx), c = (Y0 >> 1) * (W >> 1) + (X0 >> 1);
      yuvPixel(yuv[Y0 * W + X0], yuv[uOff + c], yuv[vOff + c], out, o);
    }
  }
  return { w, h, data: out };
}

// Ultralytics-style letterbox into a float32 CHW tensor (RGB / 255, pad 114),
// bilinear on luma like cv2.INTER_LINEAR. Returns the mapping back to frame pixels.
export function letterbox(yuv, W, H, inW, inH, out, rgb = new Uint8Array(3)) {
  const scale = Math.min(inW / W, inH / H);
  const nw = Math.round(W * scale), nh = Math.round(H * scale);
  const padX = Math.floor((inW - nw) / 2), padY = Math.floor((inH - nh) / 2);
  const plane = inW * inH, pad = 114 / 255;
  out.fill(pad);
  const uOff = W * H, vOff = uOff + (W * H) / 4;
  for (let y = 0; y < nh; y++) {
    const fy = Math.min(H - 1.001, Math.max(0, (y + 0.5) / scale - 0.5));
    const y0 = Math.floor(fy), ay = fy - y0;
    for (let x = 0; x < nw; x++) {
      const fx = Math.min(W - 1.001, Math.max(0, (x + 0.5) / scale - 0.5));
      const x0 = Math.floor(fx), ax = fx - x0, i = y0 * W + x0;
      const Y = (yuv[i] * (1 - ax) + yuv[i + 1] * ax) * (1 - ay) + (yuv[i + W] * (1 - ax) + yuv[i + W + 1] * ax) * ay;
      const c = (y0 >> 1) * (W >> 1) + (x0 >> 1);
      yuvPixel(Y, yuv[uOff + c], yuv[vOff + c], rgb, 0);
      const o = (y + padY) * inW + x + padX;
      out[o] = rgb[0] / 255;
      out[o + plane] = rgb[1] / 255;
      out[o + 2 * plane] = rgb[2] / 255;
    }
  }
  return { scale, padX, padY };
}
