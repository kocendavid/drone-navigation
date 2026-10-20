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
        querySelector(selector) {
          return this.querySelectorAll(selector)[0] || null;
        },
        querySelectorAll(selector) {
          return this.children.flatMap((child) =>
            typeof child === "object"
              ? [
                  ...(selector === "input" && child.type === "checkbox" ? [child] : []),
                  ...child.querySelectorAll(selector),
                ]
              : [],
          );
        },
        replaceChildren(...nodes) {
          this.children = [...nodes];
        },
        append(node) {
          this.children.push(node);
        },
      });
    return elements.get(id);
  }
  const sandbox = {
    window: { addEventListener() {} },
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
    "globalThis.api={setState(s){state=s;update();},sendAction,refreshReceipt,reviewFrame,poll,pending(){return pendingRequest;},replay(){return replayFrame;}}",
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

function logLayout(d) {
  const root = d.element("event-log");
  const height = (item) => (item.children[2]?.textContent.includes("wrapped") ? 90 : 40);
  let scroll = 0;
  Object.defineProperties(root, {
    scrollHeight: { get: () => root.children.reduce((sum, item) => sum + height(item), 0) },
    scrollTop: {
      get: () => scroll,
      set: (value) => {
        scroll = Math.max(0, Math.min(value, root.scrollHeight - 120));
      },
    },
  });
  root.getBoundingClientRect = () => ({ top: 200, bottom: 320 });
  const append = root.append;
  root.append = function (item) {
    append.call(this, item);
    item.getBoundingClientRect = () => {
      const above = root.children.slice(0, root.children.indexOf(item));
      const top = 200 + above.reduce((sum, row) => sum + height(row), 0) - scroll;
      return { top, bottom: top + height(item) };
    };
  };
  return root;
}

function logEvent(id, reason = "New entry") {
  return { id, utc: new Date(1792396800000 + id * 1000).toISOString(), kind: "decision", reason };
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
    assert.equal(d.element("live-view").hidden, false);
    assert.equal(d.element("log-live-view").hidden, false);
    d.element("live-view").onclick();
    assert.equal(d.sandbox.api.replay(), null);
    assert.equal(d.element("go").disabled, false);
    assert.equal(d.element("live-view").hidden, true);
    assert.equal(d.element("log-live-view").hidden, true);
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
      cruise_m_s: 4.0,
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
    assert.match(d.element("bands").textContent, /up to 4 m\/s/);
    assert.match(d.element("bands").textContent, /34\.65 m/);
    d.state.control_profile = "position_with_route_velocity_feedforward";
    d.state.target_stream_hz = 20;
    d.sandbox.api.setState(d.state);
    assert.match(d.element("control-profile").textContent, /Position \+ route velocity.*20 Hz/);
    d.state.acceleration_reference_ned_xy = [0.25, 0];
    d.state.control_backend = "cpp";
    d.state.motion_policy = "certified_stopping_space_then_replan";
    d.state.tracking = { error_m: 0.08, reference_time_aligned: true };
    d.sandbox.api.setState(d.state);
    assert.match(d.element("control-profile").textContent, /acceleration feedforward/);
    assert.match(d.element("control-profile").textContent, /C\+\+ publisher/);
    assert.match(d.element("tracking-status").textContent, /Sample-time error.*brakes above 0.25/);
    d.state.tracking.reference_time_aligned = false;
    d.sandbox.api.setState(d.state);
    assert.match(d.element("tracking-status").textContent, /Predicted tracking error/);
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
  {
    const d = dashboard();
    assert.equal(d.element("home").disabled, true);
    d.state.route_planner = "optimizedRoutePlanner";
    d.sandbox.api.setState(d.state);
    assert.match(d.element("home-status").textContent, /not captured/);
    d.state.home_xy = [0.5, -0.75];
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("home").disabled, false);
    assert.match(d.element("home-status").textContent, /0.50, -0.75/);
    assert(d.drawn.includes("H"));
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
            reason: "Waiting",
          },
        }),
      };
    };
    await d.element("home").onclick();
    assert.equal(request.action, "home");
    assert.equal(request.point, null, "home request carries no destination; controller owns home");
    assert.match(d.element("action-status").textContent, /home.*queued/);
    d.state.experiment_reset.busy = true;
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("home").disabled, true);
  }
  {
    const d = dashboard();
    d.state.home_xy = [0.5, -0.75];
    d.state.route_planner = "optimizedRoutePlanner";
    d.sandbox.api.setState(d.state);
    const requests = [];
    d.sandbox.fetch = async (url, options) => {
      requests.push([url, options]);
      return {
        ok: true,
        json: async () => ({
          run_id: "test",
          goal: [0.5, -0.75],
          path: [
            [0, 0],
            [0.5, -0.75],
          ],
          route_length_m: 0.91,
        }),
      };
    };
    await d.element("home-preview").onclick();
    assert.equal(requests.length, 1);
    assert.equal(requests[0][0], "/api/home-route");
    assert.equal(requests[0][1].method, undefined, "preview must only read a route");
    assert.match(d.element("home-preview-status").textContent, /0.91 m/);
    assert.equal(d.sandbox.api.pending(), null, "preview must not create a flight command");
    d.state.home_xy = null;
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("home-preview").disabled, true);
    assert.match(d.element("home-preview-status").textContent, /cleared/);
  }
  {
    const d = dashboard();
    d.state.home_xy = [0.5, -0.75];
    d.state.route_planner = "optimizedRoutePlanner";
    d.sandbox.api.setState(d.state);
    let respond;
    d.sandbox.fetch = () =>
      new Promise((resolve) => {
        respond = resolve;
      });
    const pending = d.element("home-preview").onclick();
    assert.match(d.element("home-preview-status").textContent, /Planning return/);
    d.state.experiment_reset.busy = true;
    d.sandbox.api.setState(d.state);
    respond({
      ok: true,
      json: async () => ({
        run_id: "test",
        goal: [0.5, -0.75],
        path: [
          [0, 0],
          [0.5, -0.75],
        ],
        route_length_m: 0.91,
      }),
    });
    await pending;
    assert.match(d.element("home-preview-status").textContent, /cleared/);
    assert.equal(d.element("home-preview").disabled, true);
  }
  {
    const d = dashboard();
    d.state.route_planner = "optimizedRoutePlanner";
    d.state.home_xy = [0.5, -0.75];
    d.sandbox.api.setState(d.state);
    d.sandbox.fetch = async () => ({
      ok: true,
      json: async () => ({ ...d.state, id: 7, home_xy: [-0.5, 0.75] }),
    });
    await d.sandbox.api.reviewFrame(7);
    assert.equal(d.element("home").disabled, true, "history review must not dispatch a return");
    assert.deepEqual(d.state.home_xy, [0.5, -0.75]);
    assert.deepEqual(Array.from(d.sandbox.api.replay().home_xy), [-0.5, 0.75]);
  }
  {
    const d = dashboard();
    assert.equal(d.element("state-preflight").dataset.tone, "unknown");
    assert.equal(d.element("value-vehicle").textContent, "Unknown");
    d.state.flight = {
      mode: "Hold",
      armed: false,
      next_step: "Take off when ready",
      checks: {
        telemetry_fresh: true,
        preflight_ok: false,
        landed: true,
        stationary: true,
        ground_origin: true,
        command_connected: true,
      },
    };
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("value-preflight").textContent, "Blocked");
    assert.equal(d.element("state-preflight").dataset.tone, "warn");
    assert.equal(d.element("value-vehicle").textContent, "Ground · disarmed");
    assert.equal(d.element("value-motion").textContent, "Ground stable");
    assert.match(d.element("tip-motion").textContent, /ground origin/);
    d.state.flight.checks.telemetry_fresh = false;
    d.sandbox.api.setState(d.state);
    assert.equal(
      d.element("value-vehicle").textContent,
      "Unknown",
      "stale disarmed state must not look confirmed",
    );
    assert.equal(d.element("state-preflight").dataset.tone, "unknown");
    assert.equal(d.element("telemetry").dataset.stale, "true");
    assert.match(d.element("flight-mode").textContent, /Unconfirmed/);
    assert.equal(d.element("state-navigation").dataset.tone, "warn");
  }
  {
    const d = dashboard();
    d.state.flight = {
      next_step: "Restart experiment to recover the origin change",
      checks: { telemetry_fresh: false, origin_fault: true },
    };
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("state-telemetry").dataset.tone, "bad");
    assert.equal(d.element("value-experiment").textContent, "Recovery needed");
    assert.match(d.element("tip-experiment").textContent, /Restart experiment/);
    assert.equal(d.element("map-loading").dataset.loading, "false");
    assert.match(d.element("map-loading").textContent, /restart required/);
    assert.equal(d.element("value-map").textContent, "Paused");
    assert.equal(d.element("go").disabled, true);
    assert.equal(d.element("restart").disabled, false);
    assert.equal(d.element("hold").disabled, false);
    assert.equal(
      d.element("state-telemetry").dataset.loading,
      "false",
      "origin faults require recovery, not an endless loading state",
    );
  }
  {
    const d = dashboard();
    let respond;
    d.sandbox.fetch = () =>
      new Promise((resolve) => {
        respond = resolve;
      });
    const request = d.sandbox.api.sendAction("takeoff");
    assert.equal(d.element("takeoff").dataset.loading, "true");
    assert.equal(d.element("takeoff").ariaBusy, "true");
    const pending = d.sandbox.api.pending();
    respond({
      ok: true,
      json: async () => ({
        command: { ...pending, status: "running", reason: "Waiting for hover" },
      }),
    });
    await request;
    assert.equal(d.element("takeoff").dataset.loading, "true");
    d.state.commands = [{ ...pending, status: "completed", reason: "Hover confirmed" }];
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("takeoff").dataset.loading, "false");
    assert.equal(d.element("action-status").dataset.loading, "false");
  }
  {
    const d = dashboard();
    let respond;
    d.sandbox.fetch = () =>
      new Promise((resolve) => {
        respond = resolve;
      });
    const request = d.sandbox.api.reviewFrame(2);
    assert.equal(d.element("map-loading").hidden, false);
    assert.match(d.element("map-loading").textContent, /Loading recorded map/);
    assert.equal(d.element("go").disabled, true, "controls lock as soon as replay loading begins");
    respond({ ok: true, json: async () => ({ ...d.state, id: 2, utc: "2026-10-19T08:00:00Z" }) });
    await request;
    assert.equal(d.element("map-loading").hidden, true);
    assert.equal(d.element("value-map").textContent, "Recorded");
    assert.equal(d.element("value-navigation").textContent, "Map replay");
  }
  {
    const d = dashboard();
    d.state.flight = {
      mode: "Offboard",
      armed: true,
      checks: { telemetry_fresh: true, preflight_ok: true },
    };
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("state-navigation").dataset.tone, "good");
    d.sandbox.fetch = async () => {
      throw Error("Connection lost");
    };
    await d.sandbox.api.poll();
    assert.equal(d.element("phase").textContent, "DISCONNECTED");
    assert.equal(d.element("value-vehicle").textContent, "Unknown");
    assert.equal(d.element("state-navigation").dataset.tone, "unknown");
    assert.equal(d.element("telemetry").dataset.stale, "true");
    // A late route/receipt response can call update after a failed state poll.
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("go").disabled, true);
    assert.equal(d.element("phase").textContent, "DISCONNECTED");
    d.sandbox.fetch = async () => ({ ok: true, json: async () => d.state });
    await d.sandbox.api.poll();
    assert.equal(d.element("state-navigation").dataset.tone, "good");
    assert.equal(d.element("telemetry").dataset.stale, "false");
    assert.equal(d.element("go").disabled, false);
  }
  {
    const d = dashboard();
    const target = {
      id: 1,
      utc: "2026-10-19T08:00:00Z",
      xy: [1, 2],
      status: "selected",
      reason: "Target selected",
      frame_id: 1,
    };
    d.state.history.targets = [target];
    d.state.history.events = [
      { id: 1, utc: target.utc, kind: "target_selected", reason: "Target selected", frame_id: 1 },
      {
        id: 2,
        utc: "2026-10-19T08:01:00Z",
        kind: "command",
        command: { action: "explore", status: "rejected" },
        reason: "Waiting for checks",
      },
    ];
    d.sandbox.api.setState(d.state);
    const rows = d.element("event-log").children;
    assert.equal(rows.length, 2, "target summaries merge with their selection event");
    assert.equal(rows[0].dataset.tone, "bad");
    assert.equal(rows[0].children[1].textContent, "explore / rejected");
    assert.match(rows[0].children[0].textContent, /^2026-10-19 \d{2}:\d{2}:\d{2}$/);
    assert.equal(rows[1].children[3].textContent, "View map");
    target.status = "superseded";
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("event-log").children[1].dataset.tone, "warn");
    assert.equal(
      d.element("event-log").children[1].children[1].textContent,
      "Target / superseded",
      "target changes render even without a new event",
    );
  }
  {
    const d = dashboard();
    d.state.runtime = { reload_pending: true };
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("runtime-status").hidden, false);
    assert.match(d.element("runtime-status").textContent, /Updates waiting/);
    d.state.runtime.reload_pending = false;
    d.sandbox.api.setState(d.state);
    assert.equal(d.element("runtime-status").hidden, true);
  }
  {
    const d = dashboard();
    d.state.exploration = {
      frontier_catalog: {
        pending: 2,
        resolved: 1,
        frontiers: [
          {
            id: 2,
            status: "active",
            span_m: 2.6,
            midpoint_xy: [1, 1],
            boundary_points_xy: [
              [0, 1],
              [1, 1],
              [2, 1],
            ],
            route_length_m: 4.25,
            reason: "Reachable route",
          },
          { id: 3, status: "unreachable", route_length_m: null, reason: "Unknown space remains" },
        ],
      },
    };
    d.sandbox.api.setState(d.state);
    assert.match(d.element("frontier-summary").textContent, /2 unresolved · 1 resolved/);
    const rows = d.element("frontier-list").children;
    assert.equal(rows.length, 2);
    assert.match(rows[0].textContent, /F2 · active · 2.6 m span · 4.3 m route/);
    assert.ok(d.drawn.includes("F2 · 2.6 m"), "frontier extent is labeled on the map");
    assert.match(rows[1].textContent, /F3 · unreachable · route not available/);
  }
  {
    const d = dashboard();
    d.state.phase = "PAUSED";
    d.state.reason = "Simulation clock unavailable or discontinuous; hold until Resume";
    d.state.trajectory_clock = { available: true };
    d.state.timing_diagnostics = {
      recording: true,
      stream_counts: { isaac_truth: 20, px4_position: 15, reference: 10 },
      latest: { estimation_error_m: { samples: 0 } },
      last_incident: {
        reason: "Clock delayed",
        summary: {
          stop: { clock: { consume_reason: "clock_update_stale", source_age_wall_s: 0.4 } },
        },
      },
    };
    d.sandbox.api.setState(d.state);
    assert.match(d.element("reason").textContent, /Clock recovered/);
    assert.match(
      d.element("timing-recorder").textContent,
      /Recording · Isaac 20 · native PX4 15 · references 10/,
    );
    assert.match(d.element("timing-errors").textContent, /Insufficient aligned evidence/);
    assert.match(d.element("timing-pause").textContent, /clock_update_stale/);
    d.state.timing_diagnostics.recording = false;
    d.sandbox.api.setState(d.state);
    assert.match(d.element("timing-recorder").textContent, /unavailable or stale/);
  }
  {
    const d = dashboard();
    const root = logLayout(d);
    d.state.history.events = Array.from({ length: 10 }, (_, i) => logEvent(i + 1));
    d.sandbox.api.setState(d.state);
    root.scrollTop = 55;
    const reading = root.children[1];
    const top = reading.getBoundingClientRect().top;
    d.state.history.events.push(logEvent(11, "A wrapped new entry"), logEvent(12));
    d.sandbox.api.setState(d.state);
    const retained = root.children.find((row) => row.dataset.logKey === reading.dataset.logKey);
    assert.equal(retained.getBoundingClientRect().top, top, "prepend preserves the partial row");
    assert.equal(root.scrollTop, 185);
    d.state.history.events.find((event) => event.id === 10).reason = "Updated wrapped entry";
    d.sandbox.api.setState(d.state);
    const updated = root.children.find((row) => row.dataset.logKey === reading.dataset.logKey);
    assert.equal(updated.getBoundingClientRect().top, top, "changed row heights preserve reading");
    assert.equal(root.scrollTop, 235);
  }
  {
    const d = dashboard();
    const root = logLayout(d);
    d.state.history.events = Array.from({ length: 10 }, (_, i) => logEvent(i + 1));
    d.sandbox.api.setState(d.state);
    d.state.history.events.push(logEvent(11));
    d.sandbox.api.setState(d.state);
    assert.equal(root.scrollTop, 0, "at the top, continue showing the newest entries");
    assert.equal(root.children[0].dataset.logKey, "11");
  }
  {
    const d = dashboard();
    const root = logLayout(d);
    d.state.history.events = Array.from({ length: 160 }, (_, i) => logEvent(i + 1));
    d.sandbox.api.setState(d.state);
    root.scrollTop = 500;
    const reading = root.children[12];
    const top = reading.getBoundingClientRect().top;
    d.state.history.events.push(logEvent(161));
    d.sandbox.api.setState(d.state);
    assert.equal(root.children.length, 160);
    assert.equal(root.scrollTop, 540, "row cap must not defeat scroll anchoring");
    assert.equal(
      root.children
        .find((row) => row.dataset.logKey === reading.dataset.logKey)
        .getBoundingClientRect().top,
      top,
    );
    root.scrollTop = root.scrollHeight;
    d.state.history.events.push(logEvent(162), logEvent(163), logEvent(164));
    d.sandbox.api.setState(d.state);
    assert.equal(root.scrollTop, root.scrollHeight - 120, "evicted rows stay at oldest retained");
  }
  {
    const d = dashboard();
    const root = logLayout(d);
    d.state.history.events = Array.from({ length: 10 }, (_, i) => logEvent(i + 1));
    d.sandbox.api.setState(d.state);
    root.scrollTop = 100;
    d.state.history.run_id = "new-run";
    d.state.history.events.push(logEvent(11));
    d.sandbox.api.setState(d.state);
    assert.equal(root.scrollTop, 0, "a new run starts at its newest entry");
  }
  {
    const d = dashboard();
    d.state.history.events = [
      {
        id: 1,
        utc: "2026-10-19T08:00:00Z",
        kind: "state",
        phase: "NAVIGATING",
        reason: "Slowing for tracking lag",
        speed_decision: {
          expected_speed_m_s: 0.485,
          current_speed_m_s: 0.35,
          action: "accelerating",
        },
      },
    ];
    d.sandbox.api.setState(d.state);
    const message = d.element("event-log").children[0].children[2].textContent;
    assert.match(message, /Slowing for tracking lag/);
    assert.match(message, /expected 0\.48.*m\/s/);
    assert.match(message, /current 0\.35.*m\/s/);
    assert.match(message, /command accelerating/);
    d.state.history.events[0].speed_decision.current_speed_m_s = null;
    d.sandbox.api.setState(d.state);
    assert.match(d.element("event-log").children[0].children[2].textContent, /current —/);
  }
  {
    for (const failed of [false, true]) {
      const d = dashboard();
      assert.equal(d.element("live-view").hidden, true);
      assert.equal(d.element("log-live-view").hidden, true);
      let respond, reject;
      d.sandbox.fetch = () =>
        new Promise((resolve, fail) => {
          respond = resolve;
          reject = fail;
        });
      const request = d.sandbox.api.reviewFrame(2);
      assert.equal(d.element("log-live-view").hidden, false);
      d.element("log-live-view").onclick();
      assert.equal(d.sandbox.api.replay(), null);
      assert.equal(d.element("go").disabled, false);
      assert.equal(d.element("live-view").hidden, true);
      if (failed) reject(new Error("Late replay error"));
      else
        respond({
          ok: true,
          json: async () => ({ ...d.state, id: 2, utc: new Date().toISOString() }),
        });
      await request;
      assert.equal(d.sandbox.api.replay(), null, "late replay response cannot leave live view");
      assert.equal(d.element("review-status").textContent, "Live map");
      assert.equal(d.element("log-live-view").hidden, true);
      assert.match(d.element("map-context").textContent, /LIVE MAP/);
    }
  }
  {
    const d = dashboard();
    vm.runInContext(
      "globalThis.routeCalls=[]; const renderRoute=route; route=(points,...args)=>{routeCalls.push(points);renderRoute(points,...args);};",
      d.sandbox,
    );
    d.state.path = [
      [0, 0],
      [1, 1],
    ];
    d.state.history.trail = [
      [0, 0],
      [0.2, 0],
    ];
    for (const phase of ["NAVIGATING", "COMPLETE", "REACHED"]) {
      d.sandbox.routeCalls.length = 0;
      d.state.phase = phase;
      d.sandbox.api.setState(d.state);
      assert.equal(d.sandbox.routeCalls.includes(d.state.path), phase === "NAVIGATING");
      assert.ok(
        d.sandbox.routeCalls.includes(d.state.history.trail),
        "flown trail remains visible",
      );
    }
    d.sandbox.routeCalls.length = 0;
    d.sandbox.fetch = async () => ({
      ok: true,
      json: async () => ({ ...d.state, id: 1, phase: "NAVIGATING", utc: new Date().toISOString() }),
    });
    await d.sandbox.api.reviewFrame(1);
    assert.ok(d.sandbox.routeCalls.includes(d.state.path), "replay retains earlier active routes");
  }
  console.log("33 dashboard interaction checks passed");
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
