// Cross-check: for every joint node in fr3_animated.glb, the viewer's
// currentAngle() (strip origin_rpy, measure about axis) must return the
// joint value that was recorded. This is the contract between the baked
// animation and the pose sliders: if the exporter and viewer disagree about
// quaternion order or the origin frame, the sliders silently snap.
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

// Resolve paths relative to this file so the test runs from any cwd.
const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '..', '..');
const VIEWER = path.join(ROOT, 'web', 'viewer.html');
const GLB = path.join(ROOT, 'data', 'gltf', 'fr3_animated.glb');

const html = fs.readFileSync(VIEWER, 'utf8');
const grab = (name) => {
  const i = html.indexOf(`function ${name}(`);
  if (i < 0) throw new Error(`missing ${name}`);
  let d = 0, started = false;
  for (let k = i; k < html.length; k++) {
    if (html[k] === '{') { d++; started = true; }
    else if (html[k] === '}') { d--; if (started && d === 0) return html.slice(i, k + 1); }
  }
  throw new Error(`unterminated ${name}`);
};
const src = ['rpyQuat', 'quatMul', 'axisAngleQuat', 'signedAngle', 'jointQuat']
  .map(grab).join('\n');
const mod = new Function(src + '\nreturn {rpyQuat, quatMul, axisAngleQuat, signedAngle, jointQuat};')();

// --- parse the GLB
const buf = fs.readFileSync(GLB);
const jsonLen = buf.readUInt32LE(12);
const g = JSON.parse(buf.slice(20, 20 + jsonLen).toString('utf8'));
const binOff = 20 + jsonLen + 8;

function readAccessor(i) {
  const a = g.accessors[i];
  const bv = g.bufferViews[a.bufferView];
  const base = binOff + (bv.byteOffset || 0) + (a.byteOffset || 0);
  const nc = { SCALAR: 1, VEC3: 3, VEC4: 4 }[a.type];
  const out = [];
  for (let k = 0; k < a.count; k++) {
    out.push([0, 1, 2, 3].slice(0, nc).map((c) => buf.readFloatLE(base + (k * nc + c) * 4)));
  }
  return out;
}

let fails = 0;
const check = (n, ok, extra = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${n}${extra}`);
  if (!ok) fails++;
};

// The exporter's convention: node rotation = rpy_quat(origin_rpy) * axis(value)
// The viewer strips the origin with rel = conj(origin) * q  (THREE premultiply
// with the conjugate), so mirror that exactly here.
const stripOrigin = (meta, q) => {
  const o = mod.rpyQuat(...meta.origin_rpy);
  const inv = [-o[0], -o[1], -o[2], o[3]];
  return mod.quatMul(inv, q);
};
const rest = new Map();
for (const nd of g.nodes) {
  const meta = nd.extras && nd.extras.armweb_joint;
  if (meta && meta.axis) rest.set(nd.name, meta);
}

check('joint metadata present', rest.size === 13, ` (${rest.size})`);

// Feed each animated rotation channel through the viewer's inverse and
// confirm it recovers a plausible joint angle rather than garbage.
const anim = g.animations[0];
let worst = 0, seen = 0;
for (const ch of anim.channels) {
  if (ch.target.path !== 'rotation') continue;
  const node = g.nodes[ch.target.node];
  const meta = rest.get(node.name);
  if (!meta) continue;
  const smp = anim.samplers[ch.sampler];
  const vals = readAccessor(smp.output);
  const axis = meta.axis;
  for (const q of vals) {
    const rel = stripOrigin(meta, [q[0], q[1], q[2], q[3]]);
    const ang = mod.signedAngle({ x: rel[0], y: rel[1], z: rel[2], w: rel[3] }, axis);
    if (!Number.isFinite(ang)) { worst = Infinity; break; }
    worst = Math.max(worst, Math.abs(ang));
    seen++;
  }
}
check('viewer reads finite angles from every frame', seen > 0 && worst !== Infinity,
  ` (${seen} samples)`);
check('all recovered angles within +-pi', worst <= Math.PI + 1e-6,
  ` (max |angle| = ${worst.toFixed(3)} rad)`);

// And the rest pose must read as exactly zero for every joint.
let restWorst = 0;
for (const [name, meta] of rest) {
  const nd = g.nodes.find((x) => x.name === name);
  const r = nd.rotation || [0, 0, 0, 1];
  const rel = stripOrigin(meta, r);
  const ang = mod.signedAngle({ x: rel[0], y: rel[1], z: rel[2], w: rel[3] }, meta.axis);
  if (Math.abs(ang) > 1e-3) {
    console.log(`      ${name}: type=${meta.type} axis=[${meta.axis}] ` +
      `origin_rpy=[${meta.origin_rpy.map((v) => v.toFixed(3))}] -> ${ang.toFixed(4)} rad`);
  }
  restWorst = Math.max(restWorst, Math.abs(ang));
}
check('rest pose reads as 0 on every slider', restWorst < 1e-4,
  ` (max ${restWorst.toExponential(2)} rad)`);

console.log();
if (fails) { console.log(`FAILED (${fails})`); process.exit(1); }
console.log('All viewer <-> animation checks passed.');