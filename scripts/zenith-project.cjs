const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const { createHash } = require('node:crypto');
const ASSETS = 'TeamCode/src/main/assets/autos/';
const GENERATED = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/zenith/generated/';
const MANIFEST = ASSETS + 'deployment-manifest.json';
const JAVA = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/';
function inside(root, relative) {
  if (typeof relative !== 'string' || path.isAbsolute(relative) || relative.includes('\\'))
    throw Error('Expected a project-relative slash path: ' + relative);
  const full = path.resolve(root, relative);
  const rel = path.relative(root, full);
  if (!rel || rel.startsWith('..') || path.isAbsolute(rel)) throw Error('Path escapes project: ' + relative);
  let cursor = full;
  while (cursor !== path.resolve(root)) {
    if (fs.existsSync(cursor) && fs.lstatSync(cursor).isSymbolicLink()) throw Error('Symlink output/source refused: ' + relative);
    cursor = path.dirname(cursor);
  }
  return full;
}
const read = (root, file) => fs.readFileSync(inside(root,file));
const json = (root,file) => JSON.parse(read(root,file).toString('utf8'));
const digest = value => createHash('sha256').update(value).digest('hex');
function unique(values,label) {
  if (new Set(values).size !== values.length) throw Error('Duplicate ' + label);
}
function contracts(root) {
  const source = read(root,JAVA+'AutonomousCommands.java').toString();
  const entries = [...source.matchAll(/command\("([^"]+)", "([^"]+)", "([^"]*)",/g)];
  const conditions = [...source.matchAll(/condition\("([^"]+)",/g)].map(m=>m[1]);
  unique(entries.map(m=>m[1]),'Java commands'); unique(conditions,'Java conditions');
  return { commands: new Map(entries.map(m=>[m[1],{requires:m[2].split(','),params:m[3] ? m[3].split(',') : []}])), conditions };
}
function walk(value, visit, location='steps') {
  if (!value || typeof value !== 'object') return;
  visit(value,location);
  for (const [key,child] of Object.entries(value)) {
    if (Array.isArray(child)) child.forEach((v,i)=>walk(v,visit,location+'.'+key+'['+i+']'));
    else if (child && typeof child === 'object') walk(child,visit,location+'.'+key);
  }
}
function validateTeam(root) {
  const link=json(root,'zenith.json'), robot=json(root,'autos/robot.json'), routines=json(root,'autos/routines.json');
  if (link.autosDir !== 'autos' || link.deploy.dir+'/' !== ASSETS || link.codegen.package !== 'org.firstinspires.ftc.teamcode.zenith'
      || link.codegen.dir !== 'TeamCode/src/main/java' || link.deploy.opModeBase !== 'org.firstinspires.ftc.teamcode.BioBuzzAutoFromFile')
    throw Error('Team deployment layout/base differs from supported zenith.json contract');
  if (routines.formatVersion !== 1 || routines.defaultProfile !== 'FULL_ROBOT') throw Error('Unsupported routine metadata');
  const profiles = {
    FULL_ROBOT: ['drive','intake','transfer','gate','depollinator','turretYaw','hood','flywheel','pollenCamera','limelight'],
    POLLEN_TEST: ['drive','intake','transfer','gate','pollenCamera']
  };
  const profileSource=read(root,JAVA+'AutonomousProfile.java').toString();
  for (const [name,capabilities] of Object.entries(profiles)) {
    const declaration=profileSource.match(new RegExp(name+'\\(([^)]+)\\)'));
    if (!declaration || JSON.stringify([...declaration[1].matchAll(/"([^"]+)"/g)].map(m=>m[1])) !== JSON.stringify(capabilities))
      throw Error('Java hardware profile differs: '+name);
  }
  const registry=contracts(root);
  unique(robot.commands.map(c=>c.name),'metadata command names');
  unique(robot.conditions.map(c=>c.name),'metadata condition names');
  if (JSON.stringify([...registry.commands.keys()].sort()) !== JSON.stringify(robot.commands.map(c=>c.name).sort())
      || JSON.stringify(registry.conditions.sort()) !== JSON.stringify(robot.conditions.map(c=>c.name).sort()))
    throw Error('Java/robot.json registry names disagree');
  const byName=new Map(robot.commands.map(c=>[c.name,c]));
  for (const command of robot.commands) {
    const actual=registry.commands.get(command.name);
    const required=actual.requires[0] === '*' ? profiles.FULL_ROBOT : actual.requires;
    if (JSON.stringify([...required].sort()) !== JSON.stringify([...(command.requires || [])].sort())
        || JSON.stringify(actual.params.sort()) !== JSON.stringify(Object.keys(command.params || {}).sort()))
      throw Error('Java/robot.json arguments or requirements disagree: '+command.name);
  }
  const pkg=json(root,'package.json');
  const gradle=read(root,'TeamCode/build.gradle').toString();
  const lock=json(root,'package-lock.json');
  if (pkg.devDependencies['@horizon36596/zenith-cli'] !== '0.1.2'
      || robot.kinematics.follower.version !== '3.0.0-20260828.185437-17'
      || !gradle.includes("zenith:v0.1.2") || !gradle.includes("com.pedropathing.ivy:pedro:1.1.1")
      || !gradle.includes("strictly '3.0.0-20260828.185437-17'")
      || lock.packages['node_modules/@horizon36596/zenith-cli'].version !== '0.1.2'
      || lock.packages['node_modules/@horizon36596/zenith-schema'].version !== '0.1.2')
    throw Error('Pinned CLI/Pedro/runtime contract changed');
  const files=fs.readdirSync(inside(root,'autos')).filter(f=>f.endsWith('.auto.json')).sort();
  const names=[], titles=[], classes=[], alliances={};
  for (const file of files) {
    const auto=json(root,'autos/'+file), name=file.slice(0,-10);
    if (auto.name !== name || !/^[a-z][a-z0-9]*(?:-[a-z0-9]+)*$/.test(name)) throw Error(file+': name must equal its lowercase kebab-case file stem');
    if (auto.robot !== undefined && auto.robot !== 'autos/robot.json') throw Error(file+': use the canonical autos/robot.json command contract');
    if (auto.field !== undefined) {
      if (!auto.field.startsWith('autos/field/') || !auto.field.endsWith('.field.json')) throw Error(file+': field must be inside autos/field');
      read(root,auto.field);
    }
    names.push(name); titles.push(auto.title || name); alliances[name]=auto.alliance;
    classes.push(name.split('-').map(s=>s[0].toUpperCase()+s.slice(1)).join('')+'Auto');
    const meta=routines.reserved[name], profile=meta ? meta.profile : routines.defaultProfile;
    if (!profiles[profile]) throw Error(file+': unknown hardware profile '+profile);
    const ids=[];
    walk(auto.steps,(step,where)=>{
      if (step.id) ids.push(step.id);
      if (step.kind === 'wait' && step.until !== undefined && step.timeoutS === undefined)
        throw Error(file+': '+where+': condition waits require timeoutS');
      if (['sequence','parallel','branch'].includes(step.kind) && step.timeoutS !== undefined)
        throw Error(file+': '+where+': group timeoutS is ignored by Zenith v0.1.2; bound its leaf steps');
      if (step.kind !== 'command' && !where.endsWith('.command')) return;
      const command=byName.get(step.name);
      if (!command) throw Error(file+': '+where+': unknown command '+step.name);
      if (step.name !== 'stopAll') for (const required of command.requires || [])
        if (!profiles[profile].includes(required)) throw Error(file+': '+where+': '+step.name+' requires '+required+' outside '+profile);
      const args=step.args || {};
      for (const key of Object.keys(args)) if (!Object.hasOwn(command.params || {},key))
        throw Error(file+': '+where+': '+step.name+': unknown argument '+key);
      for (const [key,param] of Object.entries(command.params || {})) {
        const value=Object.hasOwn(args,key) ? args[key] : param.default;
        const label=file+': '+where+': '+step.name+'.'+key;
        if (value === undefined) throw Error(label+' is required');
        if (param.type === 'number' || param.type === 'integer') {
          if (typeof value !== 'number' || !Number.isFinite(value) || (param.type === 'integer' && !Number.isInteger(value))
              || (param.min !== undefined && value < param.min) || (param.max !== undefined && value > param.max))
            throw Error(label+' outside declared numeric bounds');
        } else if (param.type === 'enum' ? !param.values.includes(value) : typeof value !== param.type)
          throw Error(label+' has invalid type/value');
        if (key === 'preset' && value !== 'diagnostic-example') throw Error(label+': unknown shot preset '+value);
      }
    });
    unique(ids,file+' step IDs');
  }
  unique(names,'routine names'); unique(titles,'Driver Station titles'); unique(classes,'generated Java classes');
  const reserved=routines.reserved;
  if (Object.keys(reserved).length !== 1 || reserved['pollen-navigation-test']?.profile !== 'POLLEN_TEST'
      || reserved['pollen-navigation-test'].wrapper !== JAVA+'PollenAuto.java' || reserved['pollen-navigation-test'].group !== 'Test'
      || !names.includes('pollen-navigation-test')) throw Error('Reserved pollen test wrapper/profile must be retained');
  const wrapper=read(root,JAVA+'PollenAuto.java').toString();
  if (!wrapper.includes('BioBuzzRobot.pollenTest()') || !wrapper.includes('group = "Test"')) throw Error('Pollen wrapper hardware/group changed');
  // Legacy placeholders are retained for code compatibility, but must never duplicate generated match entries.
  for (const side of ['Red','Blue']) if (!read(root,JAVA+side+'CompetitionAutonomous.java').toString().includes('@Disabled'))
    throw Error('Legacy '+side+' placeholder must be disabled');
  // A handwritten wrapper may run a routine only on the OTHER alliance (Ivy mirrors it once), at most once per
  // routine; same-alliance wrappers would duplicate the generated Driver Station entry.
  const mirrors=new Map();
  function checkWrappers(dir) {
    for (const entry of fs.readdirSync(inside(root,dir),{withFileTypes:true})) {
      const file=dir+entry.name;
      if (entry.isDirectory()) { if (file+'/' !== GENERATED) checkWrappers(file+'/'); continue; }
      if (!file.endsWith('.java')) continue;
      const text=read(root,file).toString();
      if (!text.includes('@Autonomous') || /@Disabled\b/.test(text)) continue;
      const auto=text.match(/autoName\s*\(\s*\)\s*\{\s*return\s+"([^"]+)"/)?.[1];
      if (!names.includes(auto) || file === reserved[auto]?.wrapper) continue;
      const running=text.match(/new\s+BioBuzzRobot\s*\(\s*TurretSubsystem\.Alliance\.(RED|BLUE)\s*\)/)?.[1];
      if (!running || running === alliances[auto])
        throw Error('Duplicate selectable handwritten wrapper for '+auto+': '+file);
      if (mirrors.has(auto)) throw Error('Duplicate mirrored wrapper for '+auto+': '+mirrors.get(auto)+' and '+file);
      mirrors.set(auto,file);
    }
  }
  checkWrappers(JAVA);
  return {link,robot,routines,files};
}
async function planTeamDeploy(root) {
  root=path.resolve(root);
  const team=validateTeam(root);
  const cli=path.resolve(__dirname,'../node_modules/@horizon36596/zenith-cli/dist/');
  const {loadProject}=await import(pathToFileURL(path.join(cli,'project.js')).href);
  const {planDeploy}=await import(pathToFileURL(path.join(cli,'commands/deploy.js')).href);
  const preview=planDeploy(loadProject(root));
  if (preview.errors) throw Error('Pinned Zenith deployment preview has validation errors; run npm run zenith:validate');
  const expected=new Map(), sources=new Map();
  let reservedStub;
  function copy(from,to) { const data=read(root,from); expected.set(to,data); sources.set(from,digest(data)); }
  for (const action of preview.actions) {
    if (action.kind === 'copy') copy(action.from,action.path);
    if ((action.kind === 'write' || action.kind === 'keep') && action.path.startsWith(GENERATED)) {
      if (action.path === GENERATED+'PollenNavigationTestAuto.java') { reservedStub=action.content; continue; }
      const content=action.content ?? read(root,action.path).toString();
      expected.set(action.path,Buffer.from(content));
    }
  }
  for (const file of fs.readdirSync(inside(root,'autos/field')).filter(f=>f.endsWith('.field.json')).sort())
    copy('autos/field/'+file,ASSETS+file);
  copy('autos/routines.json',ASSETS+'routines.json');
  for (const file of ['zenith.json','package-lock.json',JAVA+'PollenAuto.java']) sources.set(file,digest(read(root,file)));
  const previous=fs.existsSync(inside(root,MANIFEST)) ? json(root,MANIFEST) : {outputs:{}};
  function managed(file) {
    if (!(file.startsWith(ASSETS) && /^[a-z0-9-]+(?:\.auto|\.field)?\.json$/.test(file.slice(ASSETS.length)))
        && !(file.startsWith(GENERATED) && /^[A-Za-z0-9]+Auto\.java$/.test(file.slice(GENERATED.length))))
      throw Error('Manifest declares a path outside managed outputs: '+file);
    inside(root,file);
  }
  const removals=[];
  for (const file of Object.keys(previous.outputs || {})) {
    managed(file);
    if (!expected.has(file) && fs.existsSync(inside(root,file))) {
      if (digest(read(root,file)) !== previous.outputs[file]) throw Error('Obsolete managed file was edited; preserve/reconcile before deletion: '+file);
      removals.push(file);
    }
  }
  // The reserved duplicate can be left by the editor's raw Deploy button. Only remove an actual CLI stub.
  const duplicate=GENERATED+'PollenNavigationTestAuto.java';
  if (fs.existsSync(inside(root,duplicate))) {
    if (read(root,duplicate).toString() !== reservedStub)
      throw Error('Unmanaged reserved wrapper collision: '+duplicate);
    removals.push(duplicate);
  }
  for (const dir of [ASSETS.slice(0,-1),GENERATED.slice(0,-1)]) {
    if (!fs.existsSync(inside(root,dir))) continue;
    for (const file of fs.readdirSync(inside(root,dir))) {
      const rel=dir+'/'+file;
      if (rel === MANIFEST || expected.has(rel) || removals.includes(rel)) continue;
      if (file.endsWith('.auto.json') || file.endsWith('.java')) throw Error('Unmanaged/stale selectable output: '+rel);
    }
  }
  const manifest={formatVersion:1,cliVersion:'0.1.2',runtimeVersion:'v0.1.2',pedroVersion:team.robot.kinematics.follower.version,
    sources:Object.fromEntries([...sources].sort()),outputs:Object.fromEntries([...expected].sort().map(([file,data])=>[file,digest(data)]))};
  expected.set(MANIFEST,Buffer.from(JSON.stringify(manifest,null,2)+'\n'));
  const writes=[...expected].filter(([file,data])=>!fs.existsSync(inside(root,file)) || !read(root,file).equals(data));
  return {root,writes,removals:[...new Set(removals)].sort(),expected,preview};
}
function applyTeamDeploy(plan) {
  // All inputs, collisions and deletion paths were checked before the first mutation.
  for (const [file,data] of plan.writes) {
    const full=inside(plan.root,file); fs.mkdirSync(path.dirname(full),{recursive:true}); fs.writeFileSync(full,data);
  }
  for (const file of plan.removals) fs.unlinkSync(inside(plan.root,file));
}
module.exports={inside,validateTeam,planTeamDeploy,applyTeamDeploy,MANIFEST};
