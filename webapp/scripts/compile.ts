import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { dirname } from 'node:path';
import { compileMission } from '../server/mission.js';
import { native } from '../server/planner.js';
const [source,output]=process.argv.slice(2);
if(!source||!output){console.error('Usage: npm run mission:compile -- source.json output.apm');process.exit(1);}
try{
  const bytes=compileMission(JSON.parse(await readFile(source,'utf8')));
  await native(['--validate'],bytes);
  await mkdir(dirname(output),{recursive:true});await writeFile(output,bytes);
  console.log(`Compiled ${bytes.length} bytes: ${output}`);
}catch(error){console.error((error as Error).message);process.exitCode=1;}
