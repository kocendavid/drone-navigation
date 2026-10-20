// Exercise the real chart receiver with in-memory samples. No browser or flight.
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const assert = require("node:assert/strict");

const html = fs.readFileSync(path.join(__dirname, "../config/flight_profile.html"), "utf8");
const receiver = html.slice(
  html.indexOf("function receive(data)"),
  html.indexOf("new ResizeObserver"),
);
const elements = new Map();
const element = (id) => {
  if (!elements.has(id)) elements.set(id, { setAttribute() {} });
  return elements.get(id);
};
let requests = 0;
const context = vm.createContext({
  samples: [],
  run: null,
  version: null,
  zoom: [1, 2],
  eventHighlight: 42,
  status: { dataset: {} },
  view: { value: "full", options: [{}] },
  userSelected: false,
  plots: [true],
  byId: element,
  wallTimestamp: String,
  draw() {},
  makePlot() {},
  fetch: () => {
    requests++;
    return new Promise(() => {});
  },
});
vm.runInContext(receiver, context);
const snapshot = (values) => ({
  run: "first",
  version: "1:1",
  reset: false,
  offset: 0,
  samples: [],
  live: true,
  csv_available: true,
  navigation_available: true,
  simulation_s: 1,
  timestamp_unix_s: 1000,
  ...values,
});

context.receive(snapshot({ reset: true, samples: [{ simulation_s: 1 }] }));
assert.equal(context.samples.length, 1);
context.zoom = [0, 2];
context.eventHighlight = 10;
context.receive(snapshot({ version: "1:2", offset: 1, samples: [{ simulation_s: 2 }] }));
assert.deepEqual(
  Array.from(context.samples, (row) => row.simulation_s),
  [1, 2],
);
assert.deepEqual(context.zoom, [0, 2]);
context.receive(snapshot({ version: "1:2", offset: 2, live: false }));
assert.equal(context.samples.length, 2);
assert.equal(context.status.dataset.live, "false");
assert.match(context.status.textContent, /1000/);
context.receive(snapshot({ version: "1:3", offset: 4, samples: [{ simulation_s: 3 }] }));
assert.equal(requests, 1);
assert.equal(context.samples.length, 2);
context.receive(
  snapshot({ run: "second", version: "2:1", reset: true, samples: [{ simulation_s: 0 }] }),
);
assert.equal(context.samples.length, 1);
assert.equal(context.samples[0].simulation_s, 0);
assert.equal(context.zoom, null);
assert.equal(context.eventHighlight, null);
console.log("Flight profile UI: full snapshot, append, heartbeat, resync and run reset passed");
