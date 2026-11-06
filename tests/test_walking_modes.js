// Run with: node tests/test_walking_modes.js
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

class Element {
  constructor(tag) { this.tag = tag; this.children = []; this.open = false; }
  append(...children) { this.children.push(...children); }
  replaceChildren() { this.children = []; }
  querySelector(tag) { return this.children.find((child) => child.tag === tag); }
}
const host = new Element('div');
let language = 'zh';
const buttons = [];
const context = vm.createContext({
  $: () => host,
  document: { createElement: (tag) => new Element(tag) },
  manifestText: (zh, en) => language === 'zh' ? zh : en,
  createProfileControlButton: (command, label, target, dangerous) => {
    const button = { command, label, target, dangerous, classes: [] };
    button.classList = { add: (name) => button.classes.push(name) };
    buttons.push(button);
    return button;
  },
});
const app = fs.readFileSync('web/app.js', 'utf8');
vm.runInContext(app.slice(app.indexOf('function renderWalkingModeGroups('),
  app.indexOf('function applyLocomotionPolicy(')), context);
const registry = fs.readFileSync('src/robot_registry.cpp', 'utf8');
const go2 = registry.slice(registry.indexOf('"damp,balance_stand,stop_move,stand_up'));
const declaration = go2.match(/\{"web_mode_commands",([\s\S]*?)\}/)[1];
const commands = new Set([...declaration.matchAll(/"([^"]*)"/g)]
  .map((match) => match[1]).join('').split(','));
const render = () => context.renderWalkingModeGroups(commands, new Map(), new Map(), new Set(['free_walk']));
assert.equal(render(), true);
assert.deepEqual(buttons.slice(0, 4).map((button) => button.command),
  ['stand_up', 'free_walk', 'stop_move', 'stand_down']);
assert.equal(buttons.length, commands.size);
assert(!buttons.some((button) => button.command === 'balance_stand'));
assert(!host.children.some((child) => child.tag === 'p'));
assert(!host.children[1].children.some((child) => child.tag === 'h4'));
assert.equal(new Set(buttons.map((button) => button.command)).size, commands.size);
assert.equal(buttons.find((button) => button.command === 'free_walk').dangerous, true);
assert(buttons.find((button) => button.command === 'stop_move').classes.includes('stop-command'));
assert(buttons.find((button) => button.command === 'damp').classes.includes('critical'));
const details = host.querySelector('details');
assert.equal(details.open, false);
for (const button of buttons.filter((button) => button.command.endsWith('_on'))) {
  const index = buttons.indexOf(button);
  assert.equal(buttons[index + 1].command, button.command.replace(/_on$/, '_off'));
}
details.open = true;
language = 'en';
buttons.length = 0;
render();
assert.equal(host.querySelector('details').open, true);
assert.equal(host.children[0].textContent, 'Common modes');
assert.equal(buttons.length, commands.size);
assert.equal(context.renderWalkingModeGroups(new Set(['start', 'zero_torque']), new Map(), new Map(), new Set()), false);
assert.equal(host.hidden, true);
assert.equal(host.children.length, 0);
const a2Commands = new Set([
  'damp', 'stand_up', 'stand_down', 'balance_stand', 'stop_move', 'recovery_stand',
  'walk', 'run', 'climb', 'left_side_gait_on', 'left_side_gait_off',
  'right_side_gait_on', 'right_side_gait_off', 'hand_stand_on', 'hand_stand_off',
  'biped_stand_on', 'biped_stand_off',
]);
buttons.length = 0;
assert.equal(context.renderWalkingModeGroups(a2Commands, new Map(), new Map(), new Set(['run'])), true);
assert.deepEqual(buttons.slice(0, 4).map((button) => button.command),
  ['stand_up', 'balance_stand', 'stop_move', 'stand_down']);
assert.equal(buttons.length, a2Commands.size);
assert.equal(buttons.find((button) => button.command === 'run').dangerous, true);
assert(!buttons.some((button) => button.command === 'front_flip'));
for (const button of buttons.filter((button) => button.command.endsWith('_on'))) {
  assert.equal(buttons[buttons.indexOf(button) + 1].command, button.command.replace(/_on$/, '_off'));
}
console.log('Walking mode grouping: ordering, coverage, paired exits, warnings, language and profile reset passed.');
