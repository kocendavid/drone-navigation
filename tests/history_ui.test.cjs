// Exercise the actual dashboard script with an in-memory DOM. No browser or flight commands.
const fs = require("node:fs");
const vm = require("node:vm");
const assert = require("node:assert/strict");

function dashboard() {
  const html = fs.readFileSync(
    require("node:path").join(__dirname, "../config/navigation.html"),
    "utf8",
  );
  const elements = new Map();
  const drawn = [];
  const drawing = new Proxy(
    {},
    {
      get: (_, key) => (key === "fillText" ? (text) => drawn.push(text) : () => {}),
      set: () => true,
    },
  );
  function element(id) {
    if (!elements.has(id))
      elements.set(id, {
        id,
        textContent: "",
        disabled: false,
        checked: true,
        value: "current",
        dataset: {},
        style: {},
        children: [],
        width: 900,
        height: 900,
        getContext: () => drawing,
        addEventListener() {},
        replaceChildren() {
          this.children = [];
        },
        append(node) {
          this.children.push(node);
        },
      });
    return elements.get(id);
  }
  const sandbox = {
    document: {
      getElementById: element,
      createElement: () => element("generated-" + elements.size),
    },
    fetch: () => new Promise(() => {}),
    setTimeout: () => 1,
    clearTimeout() {},
    AbortController,
    AbortSignal,
    crypto: require("node:crypto").webcrypto,
    console,
  };
  vm.createContext(sandbox);
  vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], sandbox);
  vm.runInContext(
    "globalThis.api={setState(s){state=s;update();},sendAction,refreshReceipt,reviewFrame,pending(){return pendingRequest;},replay(){return replayFrame;}}",
    sandbox,
  );
  const state = {
    control_enabled: true,
    ready: true,
    map_fresh: true,
    phase: "HOLD",
    reason: "Waiting",
    width: 20,
    height: 20,
    origin: [-2, -2],
    resolution: 0.2,
    grid: ".".repeat(400),
    safe: "1".repeat(400),
    pose: [0, 0, 0, 1],
    observed_cells: 400,
    history: { run_id: "test", frames: [], frame_ids: [], events: [], targets: [], trail: [] },
    commands: [],
    mission: { phase: "IDLE" },
    exploration: {},
    experiment_reset: { phase: "IDLE", busy: false, reason: "Ready" },
  };
  sandbox.api.setState(state);
  return { sandbox, state, element, drawn };
}

(async () => {
  {
    const d = dashboard();
    d.state.ready = false;
    d.state.map_fresh = false;
    d.sandbox.api.setState(d.state);
    for (const id of ["go", "explore", "mission", "resume", "hold", "restart"])
      assert.equal(d.element(id).disabled, false, id + " must not flicker with telemetry");
  }
  {
    const d = dashboard();
    let respond;
    d.sandbox.fetch = () =>
      new Promise((resolve) => {
        respond = resolve;
      });
    const promise = d.sandbox.api.sendAction("restart");
    assert.equal(d.sandbox.api.pending().status, "sending");
    assert.match(d.element("action-status").textContent, /sending/);
    assert(d.drawn.some((text) => String(text).includes("Reset in progress")));
    assert.equal(d.element("go").disabled, true);
    const pending = d.sandbox.api.pending();
    respond({
      ok: true,
      json: async () => ({
        command: { ...pending, status: "running", reason: "Handing control to Hold" },
      }),
    });
    await promise;
    assert.equal(
      d.element("restart").disabled,
      true,
      "reset must remain locked until backend completion",
    );
    d.state.commands = [{ ...pending, status: "completed", reason: "Disarmed on ground" }];
    d.state.experiment_reset = {
      phase: "READY",
      busy: false,
      reason: "Reset complete",
      archive: "archive-1",
    };
    d.state.observed_cells = 0;
    d.state.map_fresh = false;
    d.sandbox.api.setState(d.state);
    assert.match(d.element("action-status").textContent, /completed/);
    assert.equal(d.element("restart").disabled, false);
  }
  {
    const d = dashboard();
    let request;
    d.sandbox.fetch = async (url, args) => {
      request = JSON.parse(args.body);
      return {
        ok: true,
        json: async () => ({
          command: {
            id: request.request_id,
            action: request.action,
            status: "queued",
            reason: "Awaiting controller",
          },
        }),
      };
    };
    await d.sandbox.api.sendAction("explore");
    assert.match(d.element("action-status").textContent, /queued/);
    d.state.commands = [
      {
        id: request.request_id,
        action: "explore",
        status: "rejected",
        reason: "Fresh position unavailable",
      },
    ];
    d.sandbox.api.setState(d.state);
    assert.match(d.element("action-status").textContent, /rejected.*Fresh position unavailable/);
    assert.equal(d.element("explore").disabled, false);
  }
  {
    const d = dashboard();
    d.sandbox.fetch = async () => ({
      ok: true,
      json: async () => ({
        ...d.state,
        id: 12,
        utc: new Date().toISOString(),
        phase: "BLOCKED",
        reason: "Route changed",
      }),
    });
    await d.sandbox.api.reviewFrame(12);
    assert.equal(d.sandbox.api.replay().phase, "BLOCKED");
    assert.equal(d.state.phase, "HOLD", "reviewing must not overwrite live telemetry");
    assert.equal(d.element("go").disabled, true);
    assert.equal(d.element("hold").disabled, false);
    d.element("live-view").onclick();
    assert.equal(d.sandbox.api.replay(), null);
    assert.equal(d.element("go").disabled, false);
  }
  {
    const d = dashboard();
    let command;
    d.sandbox.fetch = async (url, args) => {
      if (args?.method === "POST") {
        const request = JSON.parse(args.body);
        command = {
          id: request.request_id,
          action: request.action,
          status: "queued",
          reason: "Waiting",
          utc: "2026-10-16T20:00:00.000+00:00",
        };
        return { ok: true, json: async () => ({ command }) };
      }
      return {
        ok: true,
        json: async () => ({
          command: {
            ...command,
            status: "accepted",
            reason: "Controller accepted",
            utc: "2026-10-16T20:00:00.100+00:00",
          },
        }),
      };
    };
    await d.sandbox.api.sendAction("hold");
    d.state.commands = [command];
    await d.sandbox.api.refreshReceipt(command.id, 0);
    assert.equal(
      d.sandbox.api.pending().status,
      "accepted",
      "old full-map state must not overwrite a newer receipt",
    );
    assert.match(d.element("action-status").textContent, /accepted/);
  }
  {
    const d = dashboard();
    d.state.wall_policy = { confirmation_scans: 3, confirmed_cells: 1, tentative_cells: 2 };
    d.sandbox.api.setState(d.state);
    assert.match(
      d.element("map-quality").textContent,
      /1 confirmed wall cells retained until Reset/,
    );
    d.sandbox.fetch = async () => ({
      ok: true,
      json: async () => ({
        ...d.state,
        id: 3,
        utc: new Date().toISOString(),
        wall_policy: { confirmation_scans: 3, confirmed_cells: 5, tentative_cells: 3 },
      }),
    });
    await d.sandbox.api.reviewFrame(3);
    assert.match(
      d.element("map-quality").textContent,
      /5 confirmed wall cells retained until Reset/,
    );
    d.element("live-view").onclick();
    assert.match(
      d.element("map-quality").textContent,
      /1 confirmed wall cells retained until Reset/,
    );
  }
  {
    const d = dashboard();
    const attempts = [];
    d.sandbox.fetch = async (url, args) => {
      if (url === "/") return { ok: true, text: async () => 'let token = "renewed-test-token";' };
      attempts.push({ token: args.headers["X-Navigation-Token"], body: args.body });
      if (attempts.length === 1)
        return {
          ok: false,
          status: 403,
          json: async () => ({ code: "session_expired", error: "Page session expired" }),
        };
      const request = JSON.parse(args.body);
      return {
        ok: true,
        json: async () => ({
          command: {
            id: request.request_id,
            action: request.action,
            status: "queued",
            reason: "Awaiting controller",
          },
        }),
      };
    };
    await d.sandbox.api.sendAction("explore");
    assert.equal(attempts.length, 2);
    assert.equal(attempts[1].token, "renewed-test-token");
    assert.equal(
      attempts[0].body,
      attempts[1].body,
      "renewal must retain the same request ID and intent",
    );
    assert.equal(d.sandbox.api.pending().status, "queued");
  }
  {
    const d = dashboard();
    let attempts = 0;
    d.sandbox.fetch = async () => {
      attempts++;
      throw Error("Connection lost");
    };
    await d.sandbox.api.sendAction("explore");
    assert.equal(attempts, 1, "uncertain network failures must never retry a flight request");
    assert.equal(d.sandbox.api.pending().status, "unconfirmed");
  }
  {
    const d = dashboard();
    let attempts = 0;
    d.sandbox.fetch = async () => {
      attempts++;
      return {
        ok: false,
        status: 403,
        json: async () => ({ code: "origin_mismatch", error: "Another page origin" }),
      };
    };
    await d.sandbox.api.sendAction("explore");
    assert.equal(attempts, 1, "origin rejection must not be bypassed by renewal");
    assert.match(d.element("action-status").textContent, /rejected.*Another page origin/);
  }
  {
    const d = dashboard();
    const actions = [];
    d.state.flight = { mode: "Hold", armed: false, next_step: "1. Click Take off to 1 m." };
    d.state.flight_controls = { busy: false, phase: "IDLE", reason: "Waiting for a click" };
    d.sandbox.api.setState(d.state);
    assert.match(d.element("flight-next").textContent, /Take off to 1 m/);
    assert.match(d.element("flight-mode").textContent, /Disarmed/);
    assert.equal(d.element("takeoff").disabled, false);
    assert.deepEqual(actions, [], "displaying readiness must not initiate takeoff");
    d.sandbox.fetch = async (url, args) => {
      const command = JSON.parse(args.body);
      actions.push(command.action);
      return {
        ok: true,
        json: async () => ({
          command: {
            id: command.request_id,
            action: command.action,
            status: "accepted",
            reason: "Takeoff requested",
          },
        }),
      };
    };
    await d.element("takeoff").onclick();
    assert.deepEqual(actions, ["takeoff"]);
    assert.match(d.element("action-status").textContent, /takeoff.*accepted/);
  }
  {
    const d = dashboard();
    d.state.flight_controls = { busy: true, phase: "OBSERVING", reason: "Waiting for hover" };
    d.sandbox.api.setState(d.state);
    for (const id of ["takeoff", "offboard", "go", "explore", "mission"])
      assert.equal(d.element(id).disabled, true);
    for (const id of ["hold", "land", "restart"])
      assert.equal(d.element(id).disabled, false, id + " must allow interruption");
  }
  {
    const d = dashboard();
    d.state.yaw_mode = "face_route_smoothly";
    d.state.yaw_measured_ned_rad = -Math.PI / 2;
    d.state.yaw_reference_ned_rad = Math.PI / 2;
    d.state.settings = {
      yaw_rate_deg_s: 45,
      cruise_m_s: 1.0,
      slow_distance_m: 1.0,
      stop_distance_m: 0.65,
      obstacle_margin_m: 0.8,
      braking_reaction_s: 0.5,
      acceleration_m_s2: 0.25,
    };
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("yaw-measured").textContent, "270.0°");
    assert.equal(d.element("yaw-reference").textContent, "90.0°");
    assert.match(d.element("yaw-policy").textContent, /45°\/s/);
    assert.match(d.element("bands").textContent, /up to 1 m\/s/);
    assert.match(d.element("bands").textContent, /3\.15 m/);
  }
  {
    const d = dashboard();
    d.state.exploration = {
      policy: "confirmed_wall_endings_then_farthest_frontier",
      corner_xy: [0.5, 0.5],
      wall_endings_xy: [[0.5, 0.5]],
      hidden_area_m2: 2.88,
    };
    d.sandbox.api.setState(d.state);
    assert.match(d.element("corner-status").textContent, /2.88 m²/);
    assert(d.drawn.includes("Look around"));
    assert.match(d.element("corner-status").textContent, /not a flight destination/);
  }
  console.log("13 dashboard interaction checks passed");
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
