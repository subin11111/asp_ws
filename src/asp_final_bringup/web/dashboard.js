const MISSION_STEPS = ["IDLE", "MISSION1_CARRIER", "MISSION2_3_PARALLEL", "MISSION4_LANDING", "COMPLETE"];

function pose(x, y, z, yaw = 0) {
  return { x, y, z, yaw };
}

class MockDashboardDataProvider {
  constructor() {
    this.startedAt = Date.now();
  }

  snapshot() {
    const t = (Date.now() - this.startedAt) / 1000;
    const error = Math.max(0.08, 0.62 - (Math.sin(t * 0.8) + 1) * 0.17 - (t % 5) * 0.03);
    const uav = pose(42 + Math.sin(t * 0.28) * 2.2, 22 + Math.cos(t * 0.24) * 1.8, 8.2 + Math.sin(t * 0.55) * 0.35, 1.12);
    const ugv = pose(30 + Math.sin(t * 0.12) * 1.4, 16 + Math.cos(t * 0.1) * 1.2, 0.15, 0.35);
    const target = pose(44.2, 23.1, 0.1, 0);
    const detectedIds = new Set([0, 1, 2, 3, 4, 5, 8, 10]);

    return {
      mission: {
        state: "MISSION4_LANDING",
        progress: 82,
        elapsedSec: Math.floor(735 + t),
        simTimeSec: Math.floor(1260 + t * 1.08),
        statusText: "Precision landing alignment",
        complete: false,
      },
      uav: {
        pose: uav,
        velocity: { vx: 0.12, vy: -0.08, vz: -0.04, speed: 0.19 },
        phase: "LANDING_APPROACH",
        armed: true,
        offboard: true,
        targetPose: target,
        landingTarget: target,
        landingXYError: error,
        markerUsable: error < 0.48,
        altitude: uav.z,
      },
      ugv: {
        pose: ugv,
        linearSpeed: 0.08,
        angularSpeed: 0.01,
        state: "RENDEZVOUS_HOLD",
        stopped: true,
        rendezvousDistance: 0.18,
      },
      paths: {
        uavWaypoints: [pose(8, 6, 9), pose(18, 12, 10), pose(28, 20, 10), pose(38, 21, 9), pose(44.2, 23.1, 7.8)],
        ugvWaypoints: [pose(4, 4, 0.15), pose(12, 6, 0.15), pose(20, 12, 0.15), pose(30, 16, 0.15), pose(44, 23, 0.15)],
        uavTrail: Array.from({ length: 28 }, (_, i) => pose(14 + i * 1.05, 16 + Math.sin(i * 0.32) * 3, 7.5 + i * 0.08)),
        ugvTrail: Array.from({ length: 24 }, (_, i) => pose(6 + i * 1.12, 5 + Math.sin(i * 0.25) * 1.6, 0.15)),
      },
      imageTopics: [
        {
          name: "/asp_final/uav/camera/image_raw",
          label: "UAV front camera",
          status: "ArUco overlay",
          fps: 30,
          latencyMs: 42,
        },
        {
          name: "/asp_final/landing/camera/image_raw",
          label: "Landing camera",
          status: "marker_10 lock",
          fps: 24,
          latencyMs: 55,
        },
      ],
      markers: Array.from({ length: 11 }, (_, id) => ({
        id,
        type: id === 10 ? "landing" : "mission",
        detected: detectedIds.has(id),
        lastSeenSecAgo: detectedIds.has(id) ? Number(((id * 0.37 + t) % 4.5).toFixed(1)) : undefined,
        confidence: detectedIds.has(id) ? Number((0.74 + ((id + 2) % 5) * 0.045).toFixed(2)) : undefined,
        pose: detectedIds.has(id) ? pose(8 + id * 3.3, 5 + (id % 4) * 4.6, id === 10 ? 0.1 : 0.2) : undefined,
      })),
      health: {
        rosBridgeConnected: true,
        tfFresh: true,
        px4Connected: true,
        px4Offboard: true,
        px4Armed: true,
        gazeboClockActive: true,
        uavCameraActive: true,
        ugvCameraActive: true,
        markerDetectorActive: true,
      },
      events: [
        { time: "12:08:14", level: "success", message: "landing_marker_detected:10" },
        { time: "12:08:11", level: "info", message: "landing_started" },
        { time: "12:07:49", level: "success", message: "rendezvous_reached" },
        { time: "12:07:32", level: "success", message: "mission2_complete" },
        { time: "12:06:10", level: "success", message: "mission1_complete" },
        { time: "12:04:00", level: "info", message: "mission_started" },
      ],
    };
  }
}

class DashboardWebSocketClient {
  constructor(url) {
    this.url = url;
    this.socket = null;
  }

  connect(onMessage, onStatus) {
    this.socket = new WebSocket(this.url);
    this.socket.onopen = () => onStatus?.(true);
    this.socket.onclose = () => onStatus?.(false);
    this.socket.onerror = () => onStatus?.(false);
    this.socket.onmessage = (event) => onMessage(JSON.parse(event.data));
  }

  disconnect() {
    this.socket?.close();
  }
}

class RosDashboardDataAdapter {
  constructor(client) {
    this.client = client;
  }

  subscribe(onData, onStatus) {
    this.client.connect(onData, onStatus);
    return () => this.client.disconnect();
  }
}

function formatTime(totalSec) {
  const h = Math.floor(totalSec / 3600).toString().padStart(2, "0");
  const m = Math.floor((totalSec % 3600) / 60).toString().padStart(2, "0");
  const s = Math.floor(totalSec % 60).toString().padStart(2, "0");
  return `${h}:${m}:${s}`;
}

function fmt(value, digits = 2) {
  return Number(value ?? 0).toFixed(digits);
}

function landingErrorStatus(error) {
  if (error <= 0.2) return { label: "PRECISION LOCK", tone: "ok", color: "#34d399" };
  if (error <= 0.5) return { label: "ALIGNING", tone: "warn", color: "#f59e0b" };
  return { label: "OFFSET HIGH", tone: "bad", color: "#f87171" };
}

function esc(value) {
  return String(value).replace(/[&<>"']/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#039;" })[char]);
}

function metric(label, value) {
  return `<div class="metric-mini"><span>${label}</span><strong>${value}</strong></div>`;
}

function statusRow(label, value) {
  return `<div class="status-row"><span class="text-xs text-slate-400">${label}</span><strong class="value text-xs text-cyan-200">${esc(value)}</strong></div>`;
}

function telemetryCard(title, rows) {
  return `
    <section class="telemetry-card">
      <div class="section-kicker">${title}</div>
      <div class="telemetry-grid">
        ${rows.map(([label, value]) => `<div class="telemetry-item"><span>${label}</span><strong class="text-cyan-100">${esc(value)}</strong></div>`).join("")}
      </div>
    </section>
  `;
}

function markerCard(marker, large = false) {
  const recent = marker.detected && (marker.lastSeenSecAgo ?? 99) < 2;
  return `
    <div class="marker-card ${marker.detected ? "detected" : "missing"} ${recent ? "recent" : ""}">
      <div class="flex items-center justify-between">
        <strong class="${large ? "text-xl" : "text-base"}">ID ${marker.id}</strong>
        <span class="pill ${marker.detected ? "ok" : "bad"}">${marker.detected ? "DETECTED" : "NO LOCK"}</span>
      </div>
      ${marker.type === "landing" ? '<div class="mt-1 text-xs font-bold text-amber-300">LANDING PAD</div>' : ""}
      <div class="mt-3 grid gap-1 text-[11px] text-slate-400">
        <div><span class="value">${marker.lastSeenSecAgo ?? "--"}</span>s · score <span class="value">${marker.confidence ?? "--"}</span></div>
        <div class="truncate">map <span class="value">${marker.pose ? `${fmt(marker.pose.x, 1)}, ${fmt(marker.pose.y, 1)}, ${fmt(marker.pose.z, 1)}` : "--"}</span></div>
      </div>
    </div>
  `;
}

function imageTopicCard(topic) {
  return `
    <div class="topic-card">
      <div class="topic-view">
        ${topic.src ? `<img class="topic-image" src="${esc(topic.src)}" alt="${esc(topic.label)} stream" />` : ""}
        <span class="bbox-label" style="left:8px;top:8px">${esc(topic.status)}</span>
      </div>
      <div class="topic-body">
        <div class="section-kicker">${esc(topic.label)}</div>
        <div class="value truncate text-xs text-cyan-100">${esc(topic.name)}</div>
        <div class="mt-2 flex flex-wrap gap-2">
          <span class="pill ok">FPS ${topic.fps}</span>
          <span class="pill ok">LAT ${topic.latencyMs}ms</span>
        </div>
      </div>
    </div>
  `;
}

function renderDashboard(data) {
  const activeIndex = MISSION_STEPS.indexOf(data.mission.state);
  const missionMarkers = data.markers.filter((m) => m.type === "mission");
  const landing = data.markers.find((m) => m.type === "landing");
  const detected = data.markers.filter((m) => m.detected).length;
  const errorStatus = landingErrorStatus(data.uav.landingXYError ?? 0);
  const healthItems = [
    ["ROS bridge", data.health.rosBridgeConnected],
    ["TF fresh", data.health.tfFresh],
    ["PX4 connected", data.health.px4Connected],
    ["PX4 offboard", data.health.px4Offboard],
    ["PX4 armed", data.health.px4Armed],
    ["Gazebo clock", data.health.gazeboClockActive],
    ["UAV camera", data.health.uavCameraActive],
    ["UGV camera", data.health.ugvCameraActive],
    ["Marker detector", data.health.markerDetectorActive],
  ];

  document.getElementById("root").innerHTML = `
    <div class="dashboard-shell">
      <header class="mission-header ${data.mission.state === "MISSION4_LANDING" ? "landing" : ""}">
        <div>
          <div class="brand-title">ASP Autonomous Mission Dashboard</div>
          <div class="brand-subtitle">PX4 + Gazebo + ROS2 Mission Control</div>
        </div>
        <div class="timeline">
          ${MISSION_STEPS.map((step, index) => `<div class="timeline-step ${index < activeIndex ? "done" : ""} ${index === activeIndex ? "active" : ""}">${index < activeIndex ? "✓" : "●"} ${step.replace("MISSION", "M")}</div>`).join("")}
        </div>
        <div class="header-metrics">
          ${metric("Elapsed", formatTime(data.mission.elapsedSec))}
          ${metric("Sim Time", formatTime(data.mission.simTimeSec))}
          ${metric("Progress", `${data.mission.progress}%`)}
          <div class="metric-mini"><span>Health</span><strong class="${data.health.rosBridgeConnected ? "ok" : "bad"}">${data.health.rosBridgeConnected ? "NOMINAL" : "LINK DOWN"}</strong></div>
        </div>
      </header>

      <div class="main-grid">
        <aside class="panel left-panel">
          <div>
            <div class="section-kicker">Mission / Event Panel</div>
            <h2 class="mt-1 text-lg font-bold text-slate-100">${esc(data.mission.statusText)}</h2>
          </div>
          <div class="mission-clock ${data.mission.complete ? "complete" : ""}">
            <div class="section-kicker">${data.mission.complete ? "Mission Complete Time" : "Mission Elapsed Time"}</div>
            <div class="mission-clock-value">${formatTime(data.mission.elapsedSec)}</div>
          </div>
          <div class="status-card">
            <div class="status-grid">
              ${statusRow("Current Mission State", data.mission.state)}
              ${statusRow("UAV Phase", data.uav.phase)}
              ${statusRow("UGV State", data.ugv.state)}
              ${statusRow("Landing State", data.uav.markerUsable ? "MARKER_USABLE" : "SEARCHING")}
              ${statusRow("Last Mission Event", data.events.find((e) => e.message.includes("mission"))?.message ?? "-")}
              ${statusRow("Last Landing Event", data.events.find((e) => e.message.includes("landing"))?.message ?? "-")}
              ${statusRow("Mission Complete", data.mission.complete ? "TRUE" : "FALSE")}
            </div>
          </div>
          <div class="status-card">
            <div class="section-kicker">System Health Panel</div>
            <div class="mt-3 flex flex-wrap gap-2">
              ${healthItems.map(([label, ok]) => `<span class="pill ${ok ? "ok" : "bad"}">${label}</span>`).join("")}
            </div>
          </div>
          <div class="section-kicker">Event Log Stream</div>
          <div class="event-log">
            ${data.events.map((event, index) => `<div class="event-row event-${event.level}" key="${index}"><span class="value text-[11px] text-slate-500">${event.time}</span><span class="truncate text-xs text-slate-200">${esc(event.message)}</span></div>`).join("")}
          </div>
        </aside>

        <main class="map-shell">
          <canvas id="tactical-map"></canvas>
          <div class="map-overlay">
            <div class="map-toolbar">
              ${["TOP", "FOLLOW UAV", "FOLLOW UGV", "FREE"].map((item, index) => `<button class="${index === 0 ? "active" : ""}" type="button">${item}</button>`).join("")}
            </div>
            <div class="rviz-panel">
              <div class="section-kicker">RViz Displays</div>
              <div class="rviz-display-list">
                <div class="rviz-display-row">Grid / map</div>
                <div class="rviz-display-row">TF selected frames</div>
                <div class="rviz-display-row">UGV path + trail</div>
                <div class="rviz-display-row">UAV path + trail</div>
                <div class="rviz-display-row">MarkerArray 0-10</div>
                <div class="rviz-display-row">Image topics</div>
              </div>
            </div>
            <div class="map-callout">
              <div class="section-kicker">RViz-like Tactical Map</div>
              <div class="mt-2 text-xs text-slate-300">UAV / UGV / Paths / Marker / Landing</div>
              <div class="mt-3 value text-cyan-200">XY ERR ${fmt(data.uav.landingXYError)} m</div>
            </div>
            <div class="tf-panel">
              <div class="section-kicker">TF / Fixed Frame</div>
              <div class="mt-2 grid gap-1 text-[11px] text-slate-300">
                <div><span class="value text-emerald-300">map</span> → X1_asp/base_link</div>
                <div><span class="value text-cyan-200">map</span> → x500_gimbal_0/base_link</div>
                <div><span class="value text-amber-300">map</span> → X1_asp/aruco_marker_10_link</div>
              </div>
            </div>
            <div class="map-hud">
              <div class="hud-row text-xs"><span>Scale Ruler</span><b class="value">10 m</b></div>
              <div class="hud-row text-xs"><span>UAV Altitude</span><b class="value text-cyan-200">${fmt(data.uav.altitude)} m</b></div>
              <div class="hud-row text-xs"><span>Landing Target</span><b class="value text-emerald-300">aruco_marker_10_link</b></div>
            </div>
            <div class="topic-strip">
              ${data.imageTopics.map((topic) => imageTopicCard(topic)).join("")}
            </div>
          </div>
        </main>

        <div>
          <aside class="panel right-panel">
            ${telemetryCard("UAV Telemetry", [
              ["Position", `${fmt(data.uav.pose.x)}, ${fmt(data.uav.pose.y)}, ${fmt(data.uav.pose.z)}`],
              ["Yaw", `${fmt(data.uav.pose.yaw, 1)} rad`],
              ["Altitude", `${fmt(data.uav.altitude)} m`],
              ["H Speed", `${fmt(data.uav.velocity.speed)} m/s`],
              ["V Speed", `${fmt(data.uav.velocity.vz)} m/s`],
              ["Armed", data.uav.armed ? "TRUE" : "FALSE"],
              ["Offboard", data.uav.offboard ? "TRUE" : "FALSE"],
              ["Target", data.uav.targetPose ? `${fmt(data.uav.targetPose.x)}, ${fmt(data.uav.targetPose.y)}` : "-"],
              ["Target Dist", `${fmt(data.uav.landingXYError)} m`],
              ["Marker Usable", data.uav.markerUsable ? "TRUE" : "FALSE"],
            ])}
            <div class="landing-gauge">
              <div class="flex items-center justify-between">
                <div><div class="section-kicker">Landing XY Error</div><div class="mt-1 text-lg font-black ${errorStatus.tone}">${errorStatus.label}</div></div>
                <div class="value text-2xl" style="color:${errorStatus.color}">${fmt(data.uav.landingXYError)}<span class="unit">m</span></div>
              </div>
              <div class="gauge-track"><div class="gauge-fill" style="width:${Math.min(100, (data.uav.landingXYError ?? 0) * 140)}%;background:${errorStatus.color}"></div></div>
            </div>
            ${telemetryCard("UGV Telemetry", [
              ["Position", `${fmt(data.ugv.pose.x)}, ${fmt(data.ugv.pose.y)}, ${fmt(data.ugv.pose.z)}`],
              ["Yaw", `${fmt(data.ugv.pose.yaw, 1)} rad`],
              ["Linear Speed", `${fmt(data.ugv.linearSpeed)} m/s`],
              ["Angular Speed", `${fmt(data.ugv.angularSpeed)} rad/s`],
              ["Mission Mode", data.ugv.state],
              ["Rendezvous", `${fmt(data.ugv.rendezvousDistance)} m`],
              ["Stopped", data.ugv.stopped ? "TRUE" : "FALSE"],
            ])}
            <section class="camera-card">
              <div class="camera-view">
                <div class="bbox"><div class="bbox-label">LANDING MARKER LOCKED</div></div>
                <div class="absolute bottom-2 left-2 text-[10px] font-bold text-cyan-200">UAV CAMERA / ArUco Overlay</div>
              </div>
              <div class="camera-meta"><span class="pill ok">FPS 30</span><span class="pill ok">LAT 42ms</span><span class="pill ok">AGE 0.1s</span></div>
            </section>
          </aside>
        </div>
      </div>

      <section class="bottom-panel">
        <div>
          <div class="flex items-center justify-between">
            <div class="section-kicker">Marker Detection Matrix 0 1 2 3 4 5 6 7 8 9</div>
            <div class="value text-sm text-cyan-200">Detected ${detected} / 11 markers</div>
          </div>
          <div class="progress-track"><div class="progress-fill" style="width:${(detected / 11) * 100}%"></div></div>
          <div class="marker-grid">${missionMarkers.map((marker) => markerCard(marker)).join("")}</div>
        </div>
        <div class="landing-marker">
          <div class="section-kicker">Landing Marker 10</div>
          ${landing ? markerCard(landing, true) : ""}
        </div>
      </section>
    </div>
  `;
  drawMap(data);
}

function drawMap(data) {
  const canvas = document.getElementById("tactical-map");
  if (!canvas) return;
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  canvas.style.width = `${rect.width}px`;
  canvas.style.height = `${rect.height}px`;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  const w = rect.width;
  const h = rect.height;
  const allPoints = [
    ...data.paths.uavWaypoints,
    ...data.paths.ugvWaypoints,
    ...data.paths.uavTrail,
    ...data.paths.ugvTrail,
    ...data.markers.filter((m) => m.pose).map((m) => m.pose),
    data.uav.pose,
    data.ugv.pose,
    data.uav.landingTarget,
  ].filter(Boolean);
  const bounds = allPoints.reduce(
    (acc, p) => ({
      minX: Math.min(acc.minX, p.x),
      maxX: Math.max(acc.maxX, p.x),
      minY: Math.min(acc.minY, p.y),
      maxY: Math.max(acc.maxY, p.y),
    }),
    { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity }
  );
  const mapW = Math.max(1, bounds.maxX - bounds.minX);
  const mapH = Math.max(1, bounds.maxY - bounds.minY);
  const reservedLeft = 220;
  const reservedBottom = 138;
  const scale = Math.min((w - reservedLeft - 80) / mapW, (h - reservedBottom - 74) / mapH);
  const offsetX = reservedLeft + (w - reservedLeft - mapW * scale) / 2 - bounds.minX * scale;
  const offsetY = 48 + (h - reservedBottom - mapH * scale) / 2 + bounds.maxY * scale;
  const toScreen = (p) => ({ x: offsetX + p.x * scale, y: offsetY - p.y * scale });

  ctx.clearRect(0, 0, w, h);
  const gradient = ctx.createLinearGradient(0, 0, w, h);
  gradient.addColorStop(0, "#020617");
  gradient.addColorStop(1, "#061826");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(34,211,238,0.1)";
  ctx.lineWidth = 1;
  for (let x = 0; x < w; x += 32) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += 32) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  const origin = toScreen({ x: 0, y: 0 });
  ctx.save();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(248,113,113,0.75)";
  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(origin.x + 54, origin.y);
  ctx.stroke();
  ctx.strokeStyle = "rgba(52,211,153,0.75)";
  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(origin.x, origin.y - 54);
  ctx.stroke();
  ctx.fillStyle = "#94a3b8";
  ctx.font = "11px ui-monospace, monospace";
  ctx.fillText("map", origin.x + 8, origin.y + 16);
  ctx.fillText("X", origin.x + 58, origin.y + 4);
  ctx.fillText("Y", origin.x - 4, origin.y - 60);
  ctx.restore();

  const line = (points, color, width = 2, dash = []) => {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.beginPath();
    points.map(toScreen).forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.stroke();
    ctx.restore();
  };
  const dot = (p, color, r, label) => {
    const s = toScreen(p);
    ctx.fillStyle = color;
    ctx.shadowBlur = 18;
    ctx.shadowColor = color;
    ctx.beginPath();
    ctx.arc(s.x, s.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;
    if (label) {
      ctx.fillStyle = "#dbeafe";
      ctx.font = "12px ui-monospace, monospace";
      ctx.fillText(label, s.x + r + 7, s.y - r - 3);
    }
  };
  const footprint = (p, color, label, width, height) => {
    const s = toScreen(p);
    ctx.save();
    ctx.translate(s.x, s.y);
    ctx.rotate(-(p.yaw ?? 0));
    ctx.strokeStyle = color;
    ctx.fillStyle = color.replace(")", ",0.16)").replace("rgb", "rgba");
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-width / 2, -height / 2, width, height);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(width / 2, 0);
    ctx.lineTo(width / 2 + 18, 0);
    ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "#dbeafe";
    ctx.font = "12px ui-monospace, monospace";
    ctx.fillText(label, s.x + 15, s.y - 12);
  };

  ctx.save();
  ctx.fillStyle = "rgba(15,23,42,0.35)";
  ctx.strokeStyle = "rgba(148,163,184,0.12)";
  for (let i = 0; i < 9; i += 1) {
    const cell = toScreen({ x: 10 + i * 4.2, y: 9 + (i % 3) * 3.2 });
    ctx.fillRect(cell.x - 12, cell.y - 12, 24, 24);
    ctx.strokeRect(cell.x - 12, cell.y - 12, 24, 24);
  }
  ctx.restore();

  line(data.paths.ugvWaypoints, "rgba(245,158,11,0.78)", 3);
  line(data.paths.uavWaypoints, "rgba(34,211,238,0.78)", 3);
  line(data.paths.uavTrail, "rgba(56,189,248,0.56)", 2);
  line(data.paths.ugvTrail, "rgba(52,211,153,0.56)", 2);
  line([data.uav.pose, { ...data.uav.landingTarget, z: data.uav.pose.z }], "rgba(34,211,238,0.82)", 2, [8, 8]);

  data.markers.filter((m) => m.pose).forEach((m) => dot(m.pose, m.id === 10 ? "#f59e0b" : "#64748b", m.id === 10 ? 7 : 4, `M${m.id}`));
  footprint(data.ugv.pose, "rgb(249,115,22)", "UGV X1_asp", 34, 18);
  footprint(data.uav.pose, "rgb(34,211,238)", "UAV x500_gimbal_0", 28, 28);

  const target = toScreen(data.uav.landingTarget);
  ctx.strokeStyle = "#34d399";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(target.x, target.y, 28 + Math.sin(Date.now() / 220) * 4, 0, Math.PI * 2);
  ctx.stroke();
  ctx.strokeStyle = "rgba(34,211,238,0.54)";
  ctx.beginPath();
  ctx.arc(target.x, target.y, Math.max(12, (data.uav.landingXYError ?? 0.2) * 70), 0, Math.PI * 2);
  ctx.stroke();

  const uav = toScreen(data.uav.pose);
  const ground = toScreen({ ...data.uav.pose, z: 0 });
  ctx.strokeStyle = "rgba(103,232,249,0.48)";
  ctx.setLineDash([4, 6]);
  ctx.beginPath();
  ctx.moveTo(uav.x, uav.y);
  ctx.lineTo(ground.x, ground.y + 42);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#67e8f9";
  ctx.font = "11px ui-monospace, monospace";
  ctx.fillText(`alt ${fmt(data.uav.altitude)}m`, uav.x + 10, uav.y + 22);

  ctx.strokeStyle = "rgba(103,232,249,0.5)";
  ctx.beginPath();
  ctx.moveTo(uav.x, uav.y);
  ctx.lineTo(uav.x, target.y);
  ctx.stroke();

  ctx.save();
  ctx.strokeStyle = "rgba(34,211,238,0.22)";
  ctx.fillStyle = "rgba(34,211,238,0.05)";
  ctx.beginPath();
  ctx.moveTo(uav.x, uav.y);
  ctx.lineTo(uav.x + 70, uav.y + 42);
  ctx.lineTo(uav.x + 70, uav.y - 42);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

const provider = new MockDashboardDataProvider();
let latest = provider.snapshot();

async function refreshDashboard() {
  try {
    const response = await fetch("/dashboard.json", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`dashboard.json ${response.status}`);
    }
    latest = await response.json();
  } catch (error) {
    latest = provider.snapshot();
    latest.health.rosBridgeConnected = false;
  }
  renderDashboard(latest);
}

refreshDashboard();
setInterval(refreshDashboard, 500);
window.addEventListener("resize", () => renderDashboard(latest));
