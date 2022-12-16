import * as fsp from 'node:fs/promises';

const OUTPUT_FILE_PATH = 'Blacksmith/src/main/kotlin/ftc/rouge/blacksmith/adapters/_TrajectorySequenceBuilder.kt';
const INPUT_FILE_PATH = 'scripts/traj_seq_builder_methods_to_adapt.txt';

type Method = {
  name: string;
  params: { name: string, type: string }[];
}

(async () => {
  const code = (await fsp.readFile(INPUT_FILE_PATH, 'utf-8'))
    .replace('\n', '')
    .match(/public\s+[\w\s<>,]+\s+(\w+)\(.*\)/g)!
    .map(match => (
      match.split(' ').slice(2).join(' ')
    ))
    .map(codeToMethod)
    .filter(method => (
      method.name != 'build'
    ))
    .map(methodToActualMethod)
    .join('\n\n');

  const template = await fsp.readFile(OUTPUT_FILE_PATH, 'utf-8');

  await fsp.writeFile(OUTPUT_FILE_PATH, addCodeIntoTemplate(template, code));
})();

function codeToMethod(code: string) {
  const name = code.split('(')[0];

  const params = code.split('(')[1].slice(0, -1).split(', ').filter(s => s).map(param => {
    const [type, name] = param.split(' ');
    return { name, type };
  });

  return { name, params };
}

function methodToActualMethod(method: Method): string {
  const params = method.params.map(param => {
    const name = param.name;
    const type = param.type;

    const pascalCaseType = type[0].toUpperCase() + type.slice(1);

    return `${name}: ${pascalCaseType}`;
  }).join(', ');

  const invokeParams = method.params.map(param => param.name).join(', ');

  const methodClasses = method.params.map(param => {
    const type = param.type;

    const pascalCaseType = type[0].toUpperCase() + type.slice(1);

    return `${pascalCaseType}::class.java`;
  }).join(', ');

  const generatedCode = [
    `fun ${method.name}(${params}) = returnThisAndRethrow {`,
    `    getMethod("${method.name}", ${methodClasses}).invoke(builder, ${invokeParams})`,
    `}`
  ];

  return generatedCode.map(line => `    ${line}`).join('\n');
}

function addCodeIntoTemplate(template: string, code: string): string {
  const template_start_index = template.indexOf('// -- START MACHINE GENERATED CODE --') + '// -- START MACHINE GENERATED CODE --'.length;
  const template_end_index = template.indexOf('    // -- END MACHINE GENERATED CODE --');

  return (
    template.substring(0, template_start_index) +
    '\n' +
    '\n' +
    code +
    '\n' +
    '\n' +
    template.substring(template_end_index)
  );
}
