// config/default.json, overridden by config/local.json (written by the dashboard).
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { randomBytes } from 'node:crypto';

const dir = new URL('../config/', import.meta.url);
const read = (name) => JSON.parse(readFileSync(new URL(name, dir), 'utf8'));

export function merge(base, over) {
  for (const [k, v] of Object.entries(over ?? {})) {
    base[k] = v && typeof v === 'object' && !Array.isArray(v) && base[k] && typeof base[k] === 'object'
      ? merge(base[k], v) : v;
  }
  return base;
}

// Tests pass { local: false } so a machine's own overrides cannot change their results.
export function loadConfig({ local = true } = {}) {
  const cfg = read('default.json');
  if (local && existsSync(new URL('local.json', dir))) merge(cfg, read('local.json'));
  return cfg;
}

// Saves only what the dashboard changed, not the whole config.
export function saveLocal(patch) {
  const file = new URL('local.json', dir);
  const local = existsSync(file) ? read('local.json') : {};
  writeFileSync(file, JSON.stringify(merge(local, patch), null, 2));
}

// Token needed for anything that moves the robot. Generated on first start.
export function loadSecret() {
  const file = new URL('secret', dir);
  if (!existsSync(file)) writeFileSync(file, randomBytes(9).toString('base64url') + '\n', { mode: 0o600 });
  return readFileSync(file, 'utf8').trim();
}
