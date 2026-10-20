// FPV Drone toolbar panel. Talks to the FpvDrone WASM module over the CommBus.
//   panel -> module : "FpvDrone.Set" ("key=value;..."), "FpvDrone.Command" ("toggle", "getSettings", ...)
//   module -> panel : "FpvDrone.Settings" (JSON), "FpvDrone.State" (JSON, ~15 Hz)
// Written for Coherent GT: plain ES6, no optional chaining.

const FPV_CAMERA_STATES = ["Not acquired", "Acquired", "Used by another add-on", "Disabled in MSFS camera options"];

const FPV_RATE_TYPES = ["Betaflight", "Actual", "KISS"];

const FPV_TABS = [
    {
        name: "Drone",
        items: [
            { t: "section", label: "Power" },
            { t: "slider", key: "batteryCells", label: "Battery", min: 1, max: 8, step: 1, unit: "S", dec: 0 },
            { t: "slider", key: "motorKv", label: "Motor KV", min: 1000, max: 3500, step: 10, unit: "KV", dec: 0 },
            { t: "slider", key: "propDiameter", label: "Prop diameter", min: 2, max: 8, step: 0.1, unit: "in", dec: 1 },
            { t: "slider", key: "propPitch", label: "Prop pitch", min: 2, max: 6, step: 0.1, unit: "in", dec: 1 },
            { t: "check", key: "linearAcceleration", label: "Linear acceleration (thrust = throttle x KV)" },
            { t: "section", label: "Body" },
            { t: "slider", key: "droneMass", label: "Mass", min: 0.05, max: 2, step: 0.001, mult: 1000, unit: "g", dec: 0 },
            { t: "slider", key: "droneSurfaceArea", label: "Surface area", min: 0, max: 0.05, step: 0.0001, mult: 10000, unit: "cm²", dec: 0 },
            { t: "slider", key: "minimalSurfaceAreaCoefficient", label: "Min surface coeff", min: 0, max: 1, step: 0.05, dec: 2 },
            { t: "section", label: "Camera" },
            { t: "slider", key: "cameraAngle", label: "Camera angle", min: 0, max: 90, step: 1, unit: "°", dec: 0 },
            { t: "slider", key: "cameraFov", label: "Field of view", min: 30, max: 150, step: 1, unit: "°", dec: 0 },
            { t: "section", label: "Other" },
            { t: "check", key: "collision", label: "Floor collision" },
            { t: "reset" },
        ],
    },
    { name: "Rates", items: [{ t: "rates" }] },
    {
        name: "Physics",
        items: [
            { t: "slider", key: "airDensity", label: "Air density", min: 0, max: 3, step: 0.1, dec: 1 },
            { t: "slider", key: "airDrag", label: "Air drag", min: 0, max: 3, step: 0.1, dec: 1 },
            { t: "slider", key: "gravity", label: "Gravity", min: -1, max: 3, step: 0.1, dec: 1 },
            { t: "slider", key: "timeScale", label: "Time multiplier", min: 0.05, max: 2, step: 0.05, unit: "x", dec: 2 },
            { t: "section", label: "Wind" },
            { t: "check", key: "windEnabled", label: "Use the sim wind (off = calm air)" },
            {
                t: "note",
                text: "The drone, the Inspire and the wing fly through the moving air: the drone drifts, the Inspire leans into " +
                    "the wind to hold its position, the wing crabs and its airspeed changes. The wind is the one at the " +
                    "aircraft's position. It can be turned off at any time.",
            },
            { t: "slider", key: "windStrength", label: "Wind strength", min: 0, max: 200, step: 5, unit: "%", dec: 0, showIf: (s) => s.windEnabled >= 0.5 },
            { t: "windStatus" },
            { t: "section", label: "Floor" },
            { t: "note", text: "MSFS cannot test the drone against the scenery, so it hits a flat floor instead. Buildings and trees are not solid." },
            { t: "choice", key: "floorMode", label: "Floor", options: ["Terrain at launch", "Fixed altitude"] },
            { t: "slider", key: "floorOffset", label: "Floor offset", min: -100, max: 100, step: 0.5, unit: "m", dec: 1, showIf: (s) => s.floorMode < 0.5 },
            { t: "slider", key: "floorAltitude", label: "Floor altitude MSL", min: -500, max: 9000, step: 1, unit: "m", dec: 0, showIf: (s) => s.floorMode >= 0.5 },
            { t: "slider", key: "groundFriction", label: "Friction", min: 0, max: 1, step: 0.01, dec: 2 },
            { t: "slider", key: "bounciness", label: "Bounciness", min: 0, max: 1, step: 0.01, dec: 2 },
        ],
    },
    {
        name: "Controls",
        items: [
            { t: "section", label: "Assign in MSFS" },
            {
                t: "note",
                text: "In Options > Controls, make a new profile for your controller and assign: THROTTLE AXIS, AILERONS AXIS, " +
                    "ELEVATOR AXIS and RUDDER AXIS (the normal aircraft axes). Set sensitivity to 0 (linear) and the deadzone " +
                    "to its minimum, the drone has its own rates.",
            },
            { t: "section", label: "Input source" },
            {
                t: "choice", key: "inputSource", label: "Read sticks from",
                options: ["Axis events", "Aircraft controls"],
            },
            {
                t: "note",
                text: "Axis events: the axis values sent by your bindings (default). Aircraft controls: reads the yoke, pedal and " +
                    "throttle lever positions instead, use it if the bars below stay at zero with your aircraft.",
            },
            { t: "section", label: "Live axes (as the drone sees them)" },
            {
                t: "note",
                text: "Throttle up, roll right, pitch forward and yaw right must all move the bar to the right. If not, tick Invert.",
            },
            { t: "axis", key: "invertThrottle", label: "Throttle", state: "thr" },
            { t: "axis", key: "invertRoll", label: "Roll", state: "roll" },
            { t: "axis", key: "invertPitch", label: "Pitch", state: "pitch" },
            { t: "axis", key: "invertYaw", label: "Yaw", state: "yaw" },
            { t: "slider", key: "stickDeadzone", label: "Deadzone (roll, pitch, yaw)", min: 0, max: 50, step: 1, unit: "%", dec: 0 },
            { t: "check", key: "mode3d", label: "3D mode (throttle centre = no thrust, bottom = reverse)" },
            { t: "section", label: "Toggle the drone from a button" },
            {
                t: "note",
                text: "Pick SMOKE_TOGGLE, then bind \"Toggle smoke\" to a button in your MSFS controls profile: each press turns the drone on or off.",
            },
            { t: "choice", key: "toggleEvent", label: "Toggle event", options: ["Panel only", "SMOKE_TOGGLE", "TOGGLE_TAIL_HOOK_HANDLE", "TOGGLE_WATER_RUDDER"] },
            { t: "section", label: "Aircraft" },
            { t: "check", key: "freezeAircraft", label: "Freeze the aircraft while the drone flies" },
            {
                t: "note",
                text: "Your sticks still move the aircraft controls while the drone flies. Keep the aircraft frozen so it does not fly off.",
            },
        ],
    },
    {
        name: "Camera",
        items: [
            {
                t: "note",
                text: "The MSFS Camera API is still in beta. Turn the drone on, then fix the view here if it looks wrong. These settings are saved.",
            },
            { t: "check", key: "camInvertPitch", label: "Invert pitch (sky shows when diving)" },
            { t: "check", key: "camInvertBank", label: "Invert bank (horizon rolls the wrong way)" },
            { t: "check", key: "camInvertHeading", label: "Invert heading (yaw turns the wrong way)" },
            { t: "check", key: "camSwapLatLon", label: "Swap latitude / longitude (drone is far away)" },
            { t: "check", key: "camAltitudeFeet", label: "Altitude in feet (drone too high)" },
            { t: "section", label: "Spawn" },
            { t: "slider", key: "spawnBehind", label: "Behind aircraft", min: -50, max: 200, step: 1, unit: "m", dec: 0 },
            { t: "slider", key: "spawnAbove", label: "Above aircraft", min: -10, max: 100, step: 0.5, unit: "m", dec: 1 },
            { t: "section", label: "Lens" },
            { t: "check", key: "camFovHorizontal", label: "Horizontal FOV (tick if the focal length looks too zoomed in)" },
        ],
    },
    {
        name: "Inspire",
        items: [
            { t: "check", key: "inspireEnabled", label: "Inspire mode (DJI Inspire 3 style flight)" },
            {
                t: "note",
                text: "The sticks command speeds: pitch = forward, roll = sideways, throttle = climb / descent (centered = hold " +
                    "altitude), yaw = turn rate. Release the sticks and the drone brakes and holds its position. " +
                    "Use a self-centering throttle stick.",
            },
            { t: "choice", key: "inspireMode", label: "Flight mode", options: ["Tripod", "Normal", "Sport"] },
            { t: "inspire" },
            { t: "button", label: "Reset this mode", cmd: "resetInspire" },
            { t: "section", label: "Camera (all modes)" },
            { t: "slider", key: "inspireCamTilt", label: "Camera tilt", min: -110, max: 60, step: 1, unit: "\u00B0", dec: 0 },
            { t: "slider", key: "inspireCamFov", label: "Camera FOV", min: 20, max: 150, step: 1, unit: "\u00B0", dec: 0 },
        ],
    },
    {
        name: "Fixed wing",
        items: [
            { t: "check", key: "wingEnabled", label: "Fixed wing (TBS Mojito style flying wing)" },
            {
                t: "note",
                text: "Same sticks and rates as the drone: roll and pitch work like acro, scaled below. No yaw: a flying " +
                    "wing has no rudder. Hand launch: raise the throttle above 25% and the wing is thrown forward. " +
                    "Centered sticks: it glides along its flight path. Too slow, and the elevons lose authority and it stalls. " +
                    "The camera is level (0\u00B0). Dual OP is not available with the wing.",
            },
            { t: "wingStatus" },
            { t: "slider", key: "wingRollScale", label: "Roll rate (x drone rates)", min: 0.1, max: 1.5, step: 0.05, unit: "x", dec: 2 },
            { t: "slider", key: "wingPitchScale", label: "Pitch rate (x drone rates)", min: 0.1, max: 1.5, step: 0.05, unit: "x", dec: 2 },
            { t: "slider", key: "wingThrust", label: "Thrust to weight", min: 0.3, max: 3, step: 0.05, dec: 2 },
            { t: "slider", key: "wingGlide", label: "Trim angle of attack (higher = slower, floatier)", min: 1, max: 10, step: 0.5, unit: "\u00B0", dec: 1 },
            { t: "button", label: "Reset wing settings", cmd: "resetWing" },
        ],
    },
    {
        name: "Dual OP",
        items: [
            { t: "note", text: "Dual OP is off while the fixed wing is selected.", showIf: (s) => s.wingEnabled >= 0.5 },
            { t: "check", key: "dualOpEnabled", label: "Enable Dual OP (the flights are recorded)" },
            {
                t: "note",
                text: "Fly a first flight, then Replay gimbal: the drone flies it again by itself and your sticks operate the " +
                    "gimbal (roll = pan, pitch = tilt, yaw = gimbal roll). Replay FPV shows the flight from the pilot camera. " +
                    "After a replay the take is kept until you delete it.",
            },
            { t: "dualStatus" },
            {
                t: "buttons", buttons: [
                    { label: "Replay gimbal", cmd: "replayGimbal" },
                    { label: "Replay FPV", cmd: "replayFpv" },
                    { label: "Stop", cmd: "stopReplay" },
                ],
            },
            {
                t: "buttons", buttons: [
                    { label: "Record gimbal", cmd: "recordGimbal", live: (st) => (st.grec ? "Stop gimbal recording" : "Record gimbal") },
                    { label: "Clear gimbal track", cmd: "clearGimbal" },
                    { label: "Delete flight", cmd: "deleteFlight" },
                ],
            },
            { t: "slider", key: "replaySpeed", label: "Replay speed", min: 0.1, max: 1, step: 0.05, unit: "x", dec: 2 },
            { t: "check", key: "replayLoop", label: "Loop the replay" },
            { t: "slider", key: "focalLength", label: "Focal length", min: 6, max: 200, step: 1, unit: "mm", dec: 0 },
            { t: "section", label: "Ronin" },
            { t: "choice", key: "roninMode", label: "Gimbal mode", options: ["Lock", "Pan follow", "Pan/Tilt follow", "FPV"] },
            { t: "button", label: "Recenter", cmd: "recenter" },
            { t: "check", key: "roninDrift", label: "Ronin roll drift (the horizon drifts under lateral G)" },
            { t: "slider", key: "driftStrength", label: "Drift strength", min: 0, max: 20, step: 0.5, unit: "\u00B0/g", dec: 1 },
            { t: "slider", key: "driftRecovery", label: "Drift recovery", min: 0.05, max: 3, step: 0.05, unit: "s", dec: 2 },
        ],
    },
    {
        name: "Gimbal",
        items: [
            { t: "section", label: "Pan" },
            { t: "slider", key: "gimbalPanSpeed", label: "Max speed", min: 5, max: 360, step: 1, unit: "\u00B0/s", dec: 0 },
            { t: "slider", key: "gimbalPanSmooth", label: "Smoothing", min: 0, max: 2, step: 0.05, unit: "s", dec: 2 },
            { t: "slider", key: "gimbalPanExpo", label: "Expo", min: 0, max: 1, step: 0.05, dec: 2 },
            { t: "section", label: "Tilt" },
            { t: "slider", key: "gimbalTiltSpeed", label: "Max speed", min: 5, max: 360, step: 1, unit: "\u00B0/s", dec: 0 },
            { t: "slider", key: "gimbalTiltSmooth", label: "Smoothing", min: 0, max: 2, step: 0.05, unit: "s", dec: 2 },
            { t: "slider", key: "gimbalTiltExpo", label: "Expo", min: 0, max: 1, step: 0.05, dec: 2 },
            { t: "check", key: "gimbalInvertTilt", label: "Invert tilt" },
            { t: "section", label: "Roll" },
            { t: "slider", key: "gimbalRollSpeed", label: "Max speed", min: 5, max: 360, step: 1, unit: "\u00B0/s", dec: 0 },
            { t: "slider", key: "gimbalRollSmooth", label: "Smoothing", min: 0, max: 2, step: 0.05, unit: "s", dec: 2 },
            { t: "slider", key: "gimbalRollExpo", label: "Expo", min: 0, max: 1, step: 0.05, dec: 2 },
            { t: "section", label: "Follow" },
            { t: "slider", key: "followSpeed", label: "Follow speed", min: 0, max: 100, step: 1, dec: 0 },
            { t: "note", text: "How quickly the follow modes track the drone: 0 = immediately, 100 = very slowly." },
            { t: "button", label: "Reset gimbal", cmd: "resetGimbal" },
        ],
    },
];

const FPV_INSPIRE_PREFIX = ["tripod", "normal", "sport"];

// Same formulas as the module (drone.cpp), used to preview the rates.
function fpvRate(type, input, rate, superRate, expo, center, max, curve) {
    const inputAbs = Math.abs(input);
    if (type === 1) {
        const e = inputAbs * (Math.pow(input, 5) * expo + input * (1 - expo));
        return input * center + Math.max(0, max - center) * e;
    }
    if (type === 2) {
        const useRates = 1 / Math.min(Math.max(1 - inputAbs * superRate, 0.01), 1);
        const rcCommand = (Math.pow(input, 3) * curve + input * (1 - curve)) * (rate / 10);
        return Math.min(Math.max(2000 * useRates * rcCommand, -1998), 1998);
    }
    const rcCommand = input * Math.pow(inputAbs, 3) * expo + input * (1 - expo);
    let rcRate = rate;
    if (rcRate > 2) rcRate += 14.54 * (rcRate - 2);
    return 200 * rcRate * rcCommand / Math.min(Math.max(1 - inputAbs * superRate, 0.01), 1);
}

// Show script errors in the panel itself (the Coherent debugger is not always at hand).
function fpvReportError(message) {
    const el = document.getElementById("FpvError");
    if (el) el.textContent = "Panel error: " + message;
    console.error("[FPV Drone] " + message);
}

window.addEventListener("error", (e) => fpvReportError(e.message + " (" + (e.filename || "") + ":" + (e.lineno || "") + ")"));

function fpvEl(tag, className, text) {
    const e = document.createElement(tag);
    if (className) e.className = className;
    if (text !== undefined) e.textContent = text;
    return e;
}

// classList.toggle(name, force) is not reliable in every Coherent GT build.
function fpvSetClass(el, name, on) {
    if (on) el.classList.add(name);
    else el.classList.remove(name);
}

// Only touch the DOM when the text really changes: rewriting a button's text 15 times a second
// replaces its text node between mouse down and mouse up, and Coherent GT then drops the click.
function fpvSetText(el, text) {
    if (el.textContent !== text) el.textContent = text;
}

function fpvParse(payload) {
    if (typeof payload !== "string") return payload;
    try {
        return JSON.parse(payload.replace(/\0/g, ""));
    } catch (e) {
        return null;
    }
}

class IngamePanelFpvDrone extends TemplateElement {
    constructor() {
        super(...arguments);
        this.settings = null;
        this.state = null;
        this.tab = 0;
        this.controls = [];
        this.drag = null;
        this.lastStateTime = 0;
        this.pendingSend = {};
        this.sendTimer = null;
        this.resetArmed = false;
    }

    connectedCallback() {
        super.connectedCallback();
        this.tryInit(0);
    }

    // The element can be connected before its children exist: retry until they do.
    tryInit(attempt) {
        if (this.initialized) return;
        if (!document.getElementById("FpvPower")) {
            if (attempt < 100) setTimeout(() => this.tryInit(attempt + 1), 100);
            return;
        }
        try {
            this.init();
        } catch (e) {
            fpvReportError((e && e.message) || String(e));
        }
    }

    init() {
        this.initialized = true;
        window.fpvPanelStarted = true;

        this.ingameUi = document.getElementById("FpvDrone_Frame");
        this.elPower = document.getElementById("FpvPower");
        this.elStatus = document.getElementById("FpvStatus");
        this.elError = document.getElementById("FpvError");
        this.elTabs = document.getElementById("FpvTabs");
        this.elContent = document.getElementById("FpvContent");
        this.elStickLeft = document.getElementById("FpvStickLeft");
        this.elStickRight = document.getElementById("FpvStickRight");

        this.elStatus.textContent = "Connecting to the FPV Drone module...";
        // React on mouse down (no click to lose), with a short lock against double presses.
        this.elPower.addEventListener("mousedown", (e) => {
            if (e.button !== undefined && e.button !== 0) return;
            const now = Date.now();
            if (this.powerLockUntil && now < this.powerLockUntil) return;
            this.powerLockUntil = now + 600;
            // Ask for the state we want, not a toggle: a duplicated command then does no harm.
            const active = this.state && this.state.active;
            fpvSetText(this.elPower, "...");
            this.command(active ? "stop" : "start");
        });
        this.elError.addEventListener("click", () => {
            this.elError.textContent = "";
            this.command("clearError");
        });

        document.addEventListener("mousemove", (e) => this.onDragMove(e));
        document.addEventListener("mouseup", () => this.onDragEnd());

        setInterval(() => this.watchdog(), 1000);
        this.renderTabs();
        this.renderContent();

        if (typeof RegisterCommBusListener !== "function") {
            fpvReportError("RegisterCommBusListener is not available (CommBus.js not loaded).");
            return;
        }
        // Handlers are attached once the listener is registered, calls before that are lost.
        this.commBusReady = false;
        this.commBus = RegisterCommBusListener(() => {
            this.commBus.on("FpvDrone.Settings", (payload) => this.onSettings(payload));
            this.commBus.on("FpvDrone.State", (payload) => this.onState(payload));
            this.commBusReady = true;
            this.command("getSettings");
        });

        if (this.ingameUi) {
            this.ingameUi.addEventListener("panelActive", () => this.command("getSettings"));
        }
    }

    // ---------- CommBus ----------

    command(cmd) {
        if (this.commBus && this.commBusReady) this.commBus.callWasm("FpvDrone.Command", cmd);
    }

    setValue(key, value, immediate) {
        this.settings[key] = value;
        this.pendingSend[key] = value;
        if (immediate) {
            this.flush();
        } else if (!this.sendTimer) {
            this.sendTimer = setTimeout(() => this.flush(), 50);
        }
    }

    flush() {
        if (this.sendTimer) clearTimeout(this.sendTimer);
        this.sendTimer = null;
        const parts = [];
        for (const key in this.pendingSend) parts.push(key + "=" + this.pendingSend[key]);
        this.pendingSend = {};
        if (parts.length && this.commBus && this.commBusReady) this.commBus.callWasm("FpvDrone.Set", parts.join(";"));
    }

    onSettings(payload) {
        const s = fpvParse(payload);
        if (!s) return;
        const first = !this.settings;
        const typeChanged = this.settings && this.settings.ratesType !== s.ratesType;
        const floorChanged = this.settings && (this.settings.floorMode !== s.floorMode || this.settings.inspireMode !== s.inspireMode ||
            this.settings.wingEnabled !== s.wingEnabled || this.settings.windEnabled !== s.windEnabled);
        // Keep the value being dragged, the module echo may be one step behind.
        if (this.drag && this.settings) s[this.drag.item.key] = this.settings[this.drag.item.key];
        this.settings = s;
        if (first || typeChanged || floorChanged) this.renderContent();
        else this.refreshValues();
    }

    onState(payload) {
        const st = fpvParse(payload);
        if (!st) return;
        this.state = st;
        this.lastStateTime = Date.now();
        if (!this.settings) this.command("getSettings");
        this.renderState();
    }

    watchdog() {
        if (Date.now() - this.lastStateTime > 3000) {
            fpvSetText(this.elStatus, this.commBusReady
                ? "No answer from the FPV Drone module (check DevMode > Debug > WASM Debug)."
                : "Waiting for the CommBus...");
            this.elPower.classList.remove("on");
        }
    }

    // ---------- Header ----------

    renderState() {
        const st = this.state;
        if (!this.powerLockUntil || Date.now() >= this.powerLockUntil - 400)
            fpvSetText(this.elPower, st.active ? "TURN OFF" : "TURN ON");
        fpvSetClass(this.elPower, "on", !!st.active);

        const parts = [];
        if (!st.aircraft) parts.push("No aircraft");
        if (st.paused) parts.push("Paused");
        parts.push("Camera: " + (FPV_CAMERA_STATES[st.cam] || "?") + (st.camGame ? " (sim has control)" : ""));
        if (st.active) {
            parts.push(Math.round(st.speed) + " km/h");
            parts.push(Math.round(st.height) + " m");
            if (st.frozen) parts.push("Aircraft frozen");
        }
        fpvSetText(this.elStatus, parts.join("  ·  "));
        fpvSetText(this.elError, st.error || "");

        // Left stick: yaw / throttle. Right stick: roll / pitch (forward = up).
        this.placeDot(this.elStickLeft, st.yaw, st.thr);
        this.placeDot(this.elStickRight, st.roll, st.pitch);

        for (const c of this.controls) if (c.live) c.live(st);
    }

    placeDot(el, x, y) {
        el.style.left = (50 + Math.max(-1, Math.min(1, x)) * 42) + "%";
        el.style.top = (50 - Math.max(-1, Math.min(1, y)) * 42) + "%";
    }

    // ---------- Tabs ----------

    renderTabs() {
        this.elTabs.innerHTML = "";
        FPV_TABS.forEach((tab, i) => {
            const t = fpvEl("div", "fpv-tab" + (i === this.tab ? " selected" : ""), tab.name);
            t.addEventListener("click", () => {
                this.tab = i;
                this.renderTabs();
                this.renderContent();
            });
            this.elTabs.appendChild(t);
        });
    }

    renderContent() {
        this.elContent.innerHTML = "";
        this.controls = [];
        if (!this.settings) {
            this.elContent.appendChild(fpvEl("div", "fpv-note", "Loading settings..."));
            return;
        }
        for (const item of FPV_TABS[this.tab].items) this.addItem(this.elContent, item);
        this.refreshValues();
        if (this.state) this.renderState();
    }

    refreshValues() {
        for (const c of this.controls) if (c.update) c.update(this.settings);
    }

    addItem(parent, item) {
        if (item.showIf && !item.showIf(this.settings)) return;
        switch (item.t) {
            case "section": parent.appendChild(fpvEl("div", "fpv-section", item.label)); break;
            case "note": parent.appendChild(fpvEl("div", "fpv-note", item.text)); break;
            case "slider": this.addSlider(parent, item); break;
            case "check": this.addCheck(parent, item); break;
            case "choice": this.addChoice(parent, item); break;
            case "axis": this.addAxis(parent, item); break;
            case "rates": this.addRates(parent); break;
            case "reset": this.addReset(parent); break;
            case "button": this.addButtons(parent, [item]); break;
            case "buttons": this.addButtons(parent, item.buttons); break;
            case "inspire": this.addInspire(parent); break;
            case "dualStatus": this.addDualStatus(parent); break;
            case "wingStatus": this.addWingStatus(parent); break;
            case "windStatus": this.addWindStatus(parent); break;
        }
    }

    addSlider(parent, item) {
        const row = fpvEl("div", "fpv-row");
        row.appendChild(fpvEl("div", "fpv-label", item.label));
        const track = fpvEl("div", "fpv-slider");
        const fill = fpvEl("div", "fpv-slider-fill");
        const value = fpvEl("div", "fpv-slider-value");
        track.appendChild(fill);
        track.appendChild(value);
        row.appendChild(track);
        parent.appendChild(row);

        const ctrl = {
            item: item,
            track: track,
            update: (s) => {
                const v = s[item.key];
                const f = Math.max(0, Math.min(1, (v - item.min) / (item.max - item.min)));
                fill.style.width = (f * 100) + "%";
                const shown = v * (item.mult || 1);
                fpvSetText(value, shown.toFixed(item.dec) + (item.unit ? " " + item.unit : ""));
            },
        };
        track.addEventListener("mousedown", (e) => {
            this.drag = ctrl;
            this.onDragMove(e);
        });
        this.controls.push(ctrl);
    }

    onDragMove(e) {
        if (!this.drag) return;
        const item = this.drag.item;
        const rect = this.drag.track.getBoundingClientRect();
        const f = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
        let v = item.min + f * (item.max - item.min);
        v = Math.round(v / item.step) * item.step;
        v = Math.max(item.min, Math.min(item.max, parseFloat(v.toFixed(6))));
        if (v !== this.settings[item.key]) {
            this.setValue(item.key, v, false);
            this.refreshValues();
        }
    }

    onDragEnd() {
        if (!this.drag) return;
        this.drag = null;
        this.flush();
    }

    addCheck(parent, item) {
        const row = fpvEl("div", "fpv-row");
        const box = fpvEl("div", "fpv-check");
        const label = fpvEl("div", "fpv-check-label", item.label);
        row.appendChild(box);
        row.appendChild(label);
        parent.appendChild(row);
        const toggle = () => {
            this.setValue(item.key, this.settings[item.key] >= 0.5 ? 0 : 1, true);
            // Options that show or hide other controls rebuild the tab (after the click is handled).
            if (item.key === "windEnabled" || item.key === "wingEnabled") setTimeout(() => this.renderContent(), 0);
            else this.refreshValues();
        };
        box.addEventListener("click", toggle);
        label.addEventListener("click", toggle);
        this.controls.push({ update: (s) => fpvSetClass(box, "on", s[item.key] >= 0.5) });
    }

    addChoice(parent, item) {
        const row = fpvEl("div", "fpv-row");
        row.appendChild(fpvEl("div", "fpv-label", item.label));
        const box = fpvEl("div", "fpv-choices");
        const buttons = item.options.map((label, i) => {
            const b = fpvEl("div", "fpv-choice", label);
            b.addEventListener("click", () => {
                const changed = this.settings[item.key] !== i;
                this.setValue(item.key, i, true);
                if (changed && (item.key === "ratesType" || item.key === "floorMode" || item.key === "inspireMode")) this.renderContent();
                else this.refreshValues();
            });
            box.appendChild(b);
            return b;
        });
        row.appendChild(box);
        parent.appendChild(row);
        this.controls.push({
            update: (s) => buttons.forEach((b, i) => {
                const on = Math.round(s[item.key]) === i;
                fpvSetClass(b, "selected", on);
                fpvSetText(b, (on ? "\u2713 " : "") + item.options[i]);
            }),
        });
    }

    addAxis(parent, item) {
        const row = fpvEl("div", "fpv-row");
        row.appendChild(fpvEl("div", "fpv-label", item.label));
        const bar = fpvEl("div", "fpv-bar");
        const fill = fpvEl("div", "fpv-bar-fill");
        bar.appendChild(fpvEl("div", "fpv-bar-center"));
        bar.appendChild(fill);
        const out = fpvEl("div", "fpv-bar-value", "0.00");
        row.appendChild(bar);
        row.appendChild(out);
        parent.appendChild(row);

        const invRow = fpvEl("div", "fpv-row");
        const box = fpvEl("div", "fpv-check");
        const label = fpvEl("div", "fpv-check-label", "Invert " + item.label.toLowerCase());
        invRow.appendChild(box);
        invRow.appendChild(label);
        parent.appendChild(invRow);
        const toggle = () => {
            this.setValue(item.key, this.settings[item.key] >= 0.5 ? 0 : 1, true);
            // Options that show or hide other controls rebuild the tab (after the click is handled).
            if (item.key === "windEnabled" || item.key === "wingEnabled") setTimeout(() => this.renderContent(), 0);
            else this.refreshValues();
        };
        box.addEventListener("click", toggle);
        label.addEventListener("click", toggle);

        this.controls.push({
            update: (s) => fpvSetClass(box, "on", s[item.key] >= 0.5),
            live: (st) => {
                const v = Math.max(-1, Math.min(1, st[item.state] || 0));
                fill.style.left = (v >= 0 ? 50 : 50 + v * 50) + "%";
                fill.style.width = (Math.abs(v) * 50) + "%";
                fpvSetText(out, v.toFixed(2));
            },
        });
    }

    addRates(parent) {
        const type = Math.round(this.settings.ratesType);
        this.addChoice(parent, { key: "ratesType", label: "Rates type", options: FPV_RATE_TYPES });

        const axes = [["roll", "Roll"], ["pitch", "Pitch"], ["yaw", "Yaw"]];
        for (const axis of axes) {
            const p = axis[0];
            const header = fpvEl("div", "fpv-section", axis[1]);
            const out = fpvEl("span", "fpv-rate-out", "");
            header.appendChild(out);
            parent.appendChild(header);

            if (type === 1) {
                this.addSlider(parent, { key: p + "Center", label: "Center sensitivity", min: 10, max: 2000, step: 1, dec: 0 });
                this.addSlider(parent, { key: p + "Max", label: "Max rate", min: 0, max: 2000, step: 1, unit: "°/s", dec: 0 });
                this.addSlider(parent, { key: p + "Expo", label: "Expo", min: 0, max: 1, step: 0.01, dec: 2 });
            } else {
                this.addSlider(parent, { key: p + "Rate", label: "Rate", min: 0, max: 3, step: 0.01, dec: 2 });
                this.addSlider(parent, { key: p + "Super", label: "Super rate", min: 0, max: 0.99, step: 0.01, dec: 2 });
                if (type === 2) this.addSlider(parent, { key: p + "Curve", label: "Curve", min: 0, max: 1, step: 0.01, dec: 2 });
                else this.addSlider(parent, { key: p + "Expo", label: "Expo", min: 0, max: 1, step: 0.01, dec: 2 });
            }

            this.controls.push({
                update: (s) => {
                    const max = fpvRate(type, 1, s[p + "Rate"], s[p + "Super"], s[p + "Expo"], s[p + "Center"], s[p + "Max"], s[p + "Curve"]);
                    fpvSetText(out, "  max " + Math.round(max) + " °/s");
                },
            });
        }

        const reset = fpvEl("div", "fpv-button", "Reset rates");
        reset.addEventListener("click", () => this.command("resetRates"));
        parent.appendChild(reset);
    }

    addButtons(parent, defs) {
        const row = fpvEl("div", "fpv-row fpv-buttons");
        for (const def of defs) {
            const b = fpvEl("div", "fpv-button", def.label);
            // Mouse down, like the power button: no click lost when the panel refreshes.
            b.addEventListener("mousedown", (e) => {
                if (e.button !== undefined && e.button !== 0) return;
                this.command(def.cmd);
            });
            row.appendChild(b);
            if (def.live) this.controls.push({ live: (st) => fpvSetText(b, def.live(st)) });
        }
        parent.appendChild(row);
    }

    addInspire(parent) {
        const p = FPV_INSPIRE_PREFIX[Math.round(this.settings.inspireMode)] || "normal";
        const defs = [
            { key: p + "MaxSpeed", label: "Max speed", min: 0.5, max: 40, step: 0.5, mult: 3.6, unit: "km/h", dec: 0 },
            { key: p + "Climb", label: "Climb speed", min: 0.5, max: 15, step: 0.1, unit: "m/s", dec: 1 },
            { key: p + "YawRate", label: "Yaw rate", min: 5, max: 300, step: 1, unit: "\u00B0/s", dec: 0 },
            { key: p + "Response", label: "Response", min: 0.05, max: 3, step: 0.05, unit: "s", dec: 2 },
            { key: p + "MaxAccel", label: "Max acceleration", min: 0.5, max: 30, step: 0.5, unit: "m/s\u00B2", dec: 1 },
            { key: p + "MaxTilt", label: "Max tilt", min: 0, max: 60, step: 1, unit: "\u00B0", dec: 0 },
            { key: p + "Expo", label: "Stick expo", min: 0, max: 1, step: 0.05, dec: 2 },
        ];
        for (const d of defs) this.addSlider(parent, d);
    }

    addWindStatus(parent) {
        const line = fpvEl("div", "fpv-note fpv-dual-status", "");
        parent.appendChild(line);
        this.controls.push({
            live: (st) => {
                const sim = "Sim wind: " + (st.wind || 0).toFixed(0) + " km/h from " + ((st.windFrom || 0) % 360).toFixed(0) + "\u00B0";
                const on = this.settings && this.settings.windEnabled >= 0.5;
                fpvSetText(line, sim + "  \u00B7  " + (on ? "applied: " + (st.windOn || 0).toFixed(0) + " km/h" : "not applied (calm air)"));
            },
        });
    }

    addWingStatus(parent) {
        const line = fpvEl("div", "fpv-note fpv-dual-status", "");
        parent.appendChild(line);
        this.controls.push({
            live: (st) => {
                let text;
                if (!this.settings || this.settings.wingEnabled < 0.5) text = "Fixed wing off.";
                else if (!st.active) text = "Turn the drone on to fly the wing.";
                else if (st.wingWait) text = "In the hand: raise the throttle to launch.";
                else {
                    const aoa = st.aoa || 0;
                    text = "Angle of attack: " + aoa.toFixed(1) + "\u00B0  \u00B7  " + (st.speed || 0).toFixed(0) + " km/h";
                    if (Math.abs(aoa) > 13) text += "  \u00B7  STALL";
                }
                fpvSetText(line, text);
            },
        });
    }

    addDualStatus(parent) {
        const line = fpvEl("div", "fpv-note fpv-dual-status", "");
        parent.appendChild(line);
        this.controls.push({
            live: (st) => {
                let text;
                const take = (st.take || 0).toFixed(1) + " s";
                if (st.play === 1 || st.play === 2) {
                    text = (st.play === 1 ? "Replay gimbal" : "Replay FPV") + ": " + (st.ptime || 0).toFixed(1) + " / " + take;
                    if (st.grec) text += "  \u00B7  RECORDING GIMBAL";
                } else if (st.recording) {
                    text = "Recording the flight: " + take;
                } else if (st.take > 0) {
                    text = "Flight recorded: " + take + (st.takeFrozen ? " (kept until deleted)" : "");
                } else {
                    text = "No flight recorded yet.";
                }
                if (st.gtrack > 0) text += "  \u00B7  Gimbal track: " + st.gtrack.toFixed(1) + " s";
                fpvSetText(line, text);
            },
        });
    }

    addReset(parent) {
        const reset = fpvEl("div", "fpv-button", "Reset all settings");
        reset.addEventListener("click", () => {
            if (!this.resetArmed) {
                this.resetArmed = true;
                reset.textContent = "Click again to reset everything";
                setTimeout(() => {
                    this.resetArmed = false;
                    reset.textContent = "Reset all settings";
                }, 3000);
                return;
            }
            this.resetArmed = false;
            this.command("resetAll");
        });
        parent.appendChild(reset);
    }
}

window.customElements.define("ingamepanel-fpvdrone", IngamePanelFpvDrone);
checkAutoload();

// Fallback: if the custom element never starts (template quirks), start the panel logic directly.
setTimeout(() => {
    if (window.fpvPanelStarted) return;
    const host = document.querySelector("ingamepanel-fpvdrone");
    if (!host) {
        fpvReportError("Panel element not found.");
        return;
    }
    try {
        IngamePanelFpvDrone.prototype.tryInit.call(host, 0);
    } catch (e) {
        fpvReportError((e && e.message) || String(e));
    }
}, 2000);
