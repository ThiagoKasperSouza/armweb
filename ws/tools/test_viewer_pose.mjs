// Verifies viewer.html's quaternion helpers against reference values.
//
// The maths is duplicated from web/viewer.html on purpose: this checks the
// browser code's formulas (rpy -> quaternion, axis-angle, signed angle
// recovery) without needing a GPU or a DOM. If viewer.html is edited, copy
// the functions here too -- that is the point of the file.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const viewerPath = path.join(here, '..', '..', 'web', 'viewer.html');
const html = fs.readFileSync(viewerPath, 'utf8');

const fails = [];
function check(name, cond, extra = '') {
  console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${name}${extra}`);
  if (!cond) fails.push(name);
}

// Pull the real functions out of the HTML so we test what actually ships.
function grab(sig) {
  // Anchor on "function name(" so the extracted text is valid JS.
  const i = html.indexOf(`function ${sig}(`);
  if (i < 0) throw new Error(`function not found in viewer.html: ${sig}`);
  const start = html.indexOf('{', i);
  let depth = 0, end = start;
  for (let k = start; k < html.length; k++) {
    if (html[k] === '{') depth++;
    else if (html[k] === '}') { depth--; if (depth === 0) { end = k; break; } }
  }
  return html.slice(i, end + 1);
}

const names = ['rpyQuat', 'quatMul', 'axisAngleQuat', 'signedAngle', 'jointQuat'];
const src = names.map(grab).join('\n');
const mod = await import('data:text/javascript,' +
  encodeURIComponent(src + '\nexport { ' + names.join(', ') + ' };'));
const { rpyQuat, quatMul, axisAngleQuat, signedAngle, jointQuat } = mod;

// Minimal stand-in for THREE.Quaternion, which currentAngle uses.
class Q {
  constructor(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; }
}

console.log('=== identity: zero rotation ===');
let q = rpyQuat(0, 0, 0);
check('rpyQuat(0,0,0) is identity',
  Math.abs(q[0]) < 1e-15 && Math.abs(q[1]) < 1e-15 &&
  Math.abs(q[2]) < 1e-15 && Math.abs(1 - q[3]) < 1e-15, ` ${q}`);

console.log('=== 90 deg about Z maps X -> Y ===');
q = rpyQuat(0, 0, Math.PI / 2);
const rotY = quatMul(q, [1, 0, 0, 0]);
const conj = [-q[0], -q[1], -q[2], q[3]];
let r = quatMul(rotY, conj);
check('X -> Y', Math.abs(r[1] - 1) < 1e-9 &&
  Math.abs(r[0]) < 1e-9 && Math.abs(r[2]) < 1e-9, ` (${r[0]},${r[1]},${r[2]})`);

console.log('=== signedAngle recovers + and - rotations ===');
for (const ang of [0, 0.8, 1.5, 2.9, -0.8, -1.5, -2.9]) {
  const axis = [0, 0, 1];
  const total = quatMul(rpyQuat(0, 0, 0), axisAngleQuat(axis, ang));
  const rel = new Q(total[0], total[1], total[2], total[3]);
  const got = signedAngle(rel, axis);
  check(`${ang >= 0 ? '+' : ''}${ang} rad`, Math.abs(got - ang) < 1e-9,
    ` -> ${got.toFixed(9)}`);
}

console.log('=== signedAngle strips the origin rotation first ===');
// A joint whose URDF origin has its own rpy: the measured joint angle must
// still come back as the pure axis-angle value.
for (const ang of [1.1, -0.6]) {
  const meta = { axis: [0, 1, 0], origin_rpy: [0, 0.3, 0] };
  const total = jointQuat(meta, ang);
  const origin = rpyQuat(...meta.origin_rpy);
  const inv = new Q(-origin[0], -origin[1], -origin[2], origin[3]);
  // viewer does: rel = node.quaternion.clone().premultiply(inv)
  const rel = new Q(total[0], total[1], total[2], total[3]);
  const px = rel.x * inv.w + rel.w * inv.x + rel.y * inv.z - rel.z * inv.y;
  const py = rel.w * inv.y - rel.x * inv.z + rel.y * inv.w + rel.z * inv.x;
  const pz = rel.w * inv.z + rel.x * inv.y - rel.y * inv.x + rel.z * inv.w;
  const pw = rel.w * inv.w - rel.x * inv.x - rel.y * inv.y - rel.z * inv.z;
  const got = signedAngle(new Q(px, py, pz, pw), meta.axis);
  check(`origin_rpy 0.3 @ ${ang}`, Math.abs(got - ang) < 1e-9,
    ` -> ${got.toFixed(9)}`);
}

console.log('=== jointQuat(0) equals the origin rotation ===');
const meta = { axis: [1, 0, 0], origin_rpy: [0.2, 0, 0.4] };
const rest = jointQuat(meta, 0);
const want = rpyQuat(0.2, 0, 0.4);
const same = want.every((v, i) => Math.abs(v - rest[i]) < 1e-12) ||
             want.every((v, i) => Math.abs(v + rest[i]) < 1e-12);
check('rest pose matches origin_rpy', same, ` ${rest}`);

console.log('=== quaternions stay unit length ===');
for (const ang of [1.7, -2.9, 0.1]) {
  const qq = jointQuat({ axis: [0, 1, 0], origin_rpy: [0.3, 0, 0] }, ang);
  const n = Math.hypot(...qq);
  check(`|q| = 1 @ ${ang}`, Math.abs(n - 1) < 1e-12, ` ${n.toFixed(15)}`);
}

console.log('=== non-unit axes are normalised ===');
const qn = axisAngleQuat([0, 0, 5], 1.0);
const qn1 = axisAngleQuat([0, 0, 1], 1.0);
check('axis scale ignored',
  qn.every((v, i) => Math.abs(v - qn1[i]) < 1e-15), ` ${qn}`);

console.log();
if (fails.length) {
  console.log(`FAILED (${fails.length}): ` + fails.join(', '));
  process.exit(1);
}
console.log('All viewer maths checks passed.');