const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

const source = fs.readFileSync(path.join(__dirname, '../tools/smartledz-export-spa/app.js'), 'utf8');
const start = source.indexOf('function appendExtraYaml(');
const end = source.indexOf('\nfunction appendDuvNumberYaml(', start);
assert(start >= 0 && end > start);
const context = vm.createContext({});
vm.runInContext(source.slice(start, end), context);
function render(text) {
  const lines = [];
  context.appendExtraYaml(lines, text, 4);
  return lines.join('\n');
}
assert.equal(render('on_turn_on:\n  then:\n    - logger.log: "on"'),
  '    on_turn_on:\n      then:\n        - logger.log: "on"');
assert.equal(render('  lambda: |-\r\n    first\r\n\r\n    second'),
  '    lambda: |-\n      first\n    \n      second');
assert.equal(render('  \n\t'), '');
console.log('Extra YAML indentation tests passed');
