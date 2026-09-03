const RAD_TO_DEG = 180 / Math.PI;

function finiteNumber(value) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function normalizeHeading(value) {
  const parsed = finiteNumber(value);
  if (parsed === null) return null;
  return ((parsed % 360) + 360) % 360;
}

function element(name, className, text = "") {
  const node = document.createElement(name);
  node.className = className;
  if (text) node.textContent = text;
  return node;
}

function createAttitudeIndicator(containerId, name) {
  const container = document.getElementById(containerId);
  if (!container) return null;

  const root = element("div", "imu-attitude-indicator");
  root.setAttribute("role", "img");
  root.setAttribute("aria-label", `${name}姿态球与航向`);

  const display = element("div", "imu-attitude-display");
  const compass = element("div", "imu-heading-disc");
  [
    ["N", "north"],
    ["E", "east"],
    ["S", "south"],
    ["W", "west"],
  ].forEach(([text, direction]) => compass.append(element("b", direction, text)));

  const sphere = element("div", "imu-attitude-sphere");
  const horizon = element("div", "imu-attitude-horizon-plane");
  const sky = element("i", "imu-attitude-sky");
  const ground = element("i", "imu-attitude-ground");
  const horizonLine = element("i", "imu-attitude-horizon-line");
  const ladder = element("div", "imu-pitch-ladder");
  [-30, -15, 0, 15, 30].forEach((degree) => {
    const mark = element("span", degree === 0 ? "zero" : "");
    mark.style.top = `${50 + degree * 1.1}%`;
    if (degree !== 0) mark.append(element("small", "", String(Math.abs(degree))));
    ladder.append(mark);
  });
  horizon.append(sky, ground, horizonLine, ladder);
  sphere.append(horizon);

  const aircraft = element("div", "imu-aircraft-reference");
  aircraft.append(
    element("i", "wing left"),
    element("i", "center"),
    element("i", "wing right"),
  );
  display.append(compass, sphere, aircraft, element("i", "imu-roll-pointer"));

  const heading = element("div", "imu-heading-readout");
  heading.append(element("span", "", "航向"));
  const headingValue = element("strong", "", "--°");
  heading.append(headingValue);

  root.append(display, heading);
  container.replaceChildren(root);

  return {
    root,
    compass,
    horizon,
    heading: headingValue,
  };
}

function updateAttitudeIndicator(indicator, imu) {
  if (!indicator) return;
  const rpy = imu?.rpy_rad || [];
  const rollRad = finiteNumber(rpy[0]);
  const pitchRad = finiteNumber(rpy[1]);
  const yawRad = finiteNumber(rpy[2]);
  const hasOrientation = rollRad !== null && pitchRad !== null && yawRad !== null;

  const rollDeg = hasOrientation ? rollRad * RAD_TO_DEG : 0;
  const pitchDeg = hasOrientation ? pitchRad * RAD_TO_DEG : 0;
  const yawDeg = hasOrientation ? yawRad * RAD_TO_DEG : 0;
  const headingDeg = normalizeHeading(hasOrientation ? yawDeg : null);
  const pitchOffset = clamp(pitchDeg, -45, 45) * 0.72;

  indicator.root.classList.toggle("data-missing", !hasOrientation);
  indicator.horizon.style.transform =
    `translateY(${pitchOffset}px) rotate(${-rollDeg}deg)`;
  indicator.compass.style.transform = `rotate(${-yawDeg}deg)`;
  indicator.heading.textContent = headingDeg === null ? "--°" : `${headingDeg.toFixed(0)}°`;
}

const torsoIndicator = createAttitudeIndicator("torsoGyroGauges", "机身 IMU");
const hipIndicator = createAttitudeIndicator("hipGyroGauges", "髋部 IMU");

function render(data) {
  updateAttitudeIndicator(torsoIndicator, data?.imu?.torso);
  updateAttitudeIndicator(hipIndicator, data?.imu?.hip);
}

window.addEventListener("g1:telemetry", (event) => render(event.detail));
window.addEventListener("g1:connection", (event) => {
  const stale = event.detail?.online === false;
  torsoIndicator?.root.classList.toggle("stale", stale);
  hipIndicator?.root.classList.toggle("stale", stale);
});
if (window.g1LatestTelemetry) render(window.g1LatestTelemetry);
