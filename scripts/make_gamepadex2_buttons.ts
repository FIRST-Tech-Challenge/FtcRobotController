import * as fsp from 'node:fs/promises';

const GAMEPAD_EX2_PATH = 'TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcodekt/blacksmith/listeners/GamepadEx2.kt';

(async () => {
  let generated_code = '\n';

  getButtons().forEach(button => {
    if (button.type == 'boolean') {
      generated_code += generateBooleanButton(button.name) + '\n\n';
    } else {
      generated_code += generateFloatButton(button.name) + '\n\n';
    }
  });

  generated_code = generated_code.slice(0, -1); // Remove extra newline

  let template = await fsp.readFile(GAMEPAD_EX2_PATH, 'utf-8');

  let generated_file = addCodeIntoTemplate(template, generated_code);

  await fsp.writeFile(GAMEPAD_EX2_PATH, generated_file);
})();

function generateBooleanButton(name: string): string {
  const generated_code = [
    `/**`,
    ` * Allows client to perform an action when the gamepad's '${name}' button's state is mutated.`,
    ` * \`\`\`java`,
    ` * //e.g:`,
    ` * gamepad_x1.${name}.${randomUsageFunction()}(this::doSomething);`,
    ` */`,
    `@JvmField`,
    `val ${name} = Listener(gamepad::${name})`,
  ]

  return generated_code.map(line => `    ${line}`).join('\n');
}

function generateFloatButton(name: string): string {
  const random_usage_function = randomUsageFunction();

  const generated_code = [
    `/**`,
    ` * Allows client to perform an action when the gamepad's '${name}' button's state is mutated.`,
    ` * \`\`\`java`,
    ` * //e.g: (Triggers when abs(${name}) > .5)`,
    ` * gamepad_x1.${name}.${random_usage_function}(this::doSomething);`,
    ` */`,
    `@JvmField`,
    `val ${name} = ${name}(deadzone = .5)`,
    ``,
    `/**`,
    ` * Allows client to perform an action when the gamepad's '${name}' button's state is mutated.`,
    ` * \`\`\`java`,
    ` * //e.g: (Triggers when abs(${name}) > the_given_deadzone)`,
    ` * gamepad_x1.${name}(.1).${random_usage_function}(this::doSomething);`,
    ` * \`\`\``,
    ` * @param deadzone The minimum value that the ${name} must be above to trigger the event.`,
    ` */`,
    `fun ${name}(deadzone: Double): Listener {`,
    `    return Listener { abs(gamepad.${name}) > deadzone }`,
    `}`,
  ];

  return generated_code.map(line => `    ${line}`).join('\n');
}

function addCodeIntoTemplate(template: string, code: string): string {
  const template_start_index = template.indexOf('// -- START MACHINE GENERATED CODE --') + '// -- START MACHINE GENERATED CODE --'.length;
  const template_end_index = template.indexOf('    // -- END MACHINE GENERATED CODE --');

  return (
    template.substring(0, template_start_index) +
    '\n' +
    code +
    '\n' +
    template.substring(template_end_index)
  );
}

function randomUsageFunction() {
  return [
    'onRise',
    'onFall',
    'whileHigh',
    'whileLow',
  ][~~(Math.random() * 4)];
}

interface GamepadButton {
  type: 'float' | 'boolean',
  name: string
}

function getButtons(): GamepadButton[] {
  return [
    { type: 'boolean', name: 'a' },
    { type: 'boolean', name: 'b' },
    { type: 'boolean', name: 'x' },
    { type: 'boolean', name: 'y' },
    { type: 'boolean', name: 'dpad_up' },
    { type: 'boolean', name: 'dpad_down' },
    { type: 'boolean', name: 'dpad_left' },
    { type: 'boolean', name: 'dpad_right' },
    { type: 'boolean', name: 'left_bumper' },
    { type: 'boolean', name: 'right_bumper' },
    { type: 'float',   name: 'left_stick_x' },
    { type: 'float',   name: 'left_stick_y' },
    { type: 'float',   name: 'right_stick_x' },
    { type: 'float',   name: 'right_stick_y' },
    { type: 'float',   name: 'left_trigger' },
    { type: 'float',   name: 'right_trigger' },
    { type: 'boolean', name: 'left_stick_button' },
    { type: 'boolean', name: 'right_stick_button' },
    { type: 'boolean', name: 'circle' },
    { type: 'boolean', name: 'cross' },
    { type: 'boolean', name: 'triangle' },
    { type: 'boolean', name: 'square' },
    { type: 'boolean', name: 'share' },
    { type: 'boolean', name: 'options' },
    { type: 'boolean', name: 'guide' },
    { type: 'boolean', name: 'start' },
    { type: 'boolean', name: 'back' },
    { type: 'boolean', name: 'touchpad' },
    { type: 'boolean', name: 'touchpad_finger_1' },
    { type: 'boolean', name: 'touchpad_finger_2' },
    { type: 'float',   name: 'touchpad_finger_1_x' },
    { type: 'float',   name: 'touchpad_finger_1_y' },
    { type: 'float',   name: 'touchpad_finger_2_x' },
    { type: 'float',   name: 'touchpad_finger_2_y' },
    { type: 'boolean', name: 'ps' },
  ];
}
