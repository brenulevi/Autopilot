import { existsSync, readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
let root=dirname(fileURLToPath(import.meta.url));
while (true) {
  const pkg=join(root,'package.json');
  if (existsSync(pkg) && JSON.parse(readFileSync(pkg,'utf8')).name==='autopilot-mission-planner') break;
  const parent=dirname(root); if(parent===root) throw Error('Cannot locate webapp root.'); root=parent;
}
export const appRoot=root;
export const repoRoot=dirname(root);
export const nativePath=join(root,'.native-build','bin',process.platform==='win32'?'mission_preview.exe':'mission_preview');
