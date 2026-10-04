// Parse-checks viewer.html's module script by stripping the bare `import`
// lines (which new Function cannot parse) and compiling the rest.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(
  path.join(here, '..', '..', 'web', 'viewer.html'), 'utf8');

const blocks = [...html.matchAll(/<script type="module">([\s\S]*?)<\/script>/g)];
console.log(`  found ${blocks.length} module script block(s)`);
if (blocks.length !== 1) {
  console.log('  [FAIL] expected exactly one module script');
  process.exit(1);
}

const js = blocks[0][1].replace(/^\s*import .*?;\s*$/gm, '');
try {
  // eslint-disable-next-line no-new-func
  new Function(js);
  console.log('  [PASS] module script parses as valid JS');
} catch (e) {
  console.log(`  [FAIL] JS syntax error: ${e.message}`);
  process.exit(1);
}

const needs = ['rpyQuat', 'quatMul', 'axisAngleQuat', 'signedAngle',
               'jointQuat', 'applyAngle', 'currentAngle', 'collectJoints',
               'buildPosePanel', 'GLTFExporter'];
const missing = needs.filter((n) => !js.includes(n));
console.log(missing.length
  ? `  [FAIL] missing symbols: ${missing.join(', ')}`
  : `  [PASS] all ${needs.length} required symbols present`);

// The exporter must be asked for binary output, or the download is not a .glb.
if (/binary:\s*true/.test(js)) console.log('  [PASS] exporter uses binary:true');
else { console.log('  [FAIL] exporter is not in binary mode'); process.exit(1); }

// userData is where three.js puts glTF `extras`; the collector must read it.
if (/userData\?\.armweb_joint/.test(js))
  console.log('  [PASS] joint metadata read from userData.armweb_joint');
else { console.log('  [FAIL] joint metadata is not read'); process.exit(1); }

console.log('Viewer structure checks passed.');