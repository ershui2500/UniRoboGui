// Manifest parameters select presets without introducing product-specific UI paths.
window.MotionPolicy = {
  preset(parameters, fsm, speed) {
    const entries = String(parameters.fsm_speed_presets || "").split(",")
      .map((entry) => entry.split(":").map(Number));
    const selected = entries.find((entry) => entry.length === 5 &&
      entry[0] === Number(fsm) && entry[1] === Number(speed));
    const fallback = String(parameters.speed_presets || "").split(",")
      .map((entry) => entry.split(":").map(Number))
      .find((entry) => entry.length === 4 && entry[0] === Number(speed));
    const values = selected?.slice(2) || fallback?.slice(1);
    if (!values || !values.every((value) => Number.isFinite(value) && value > 0)) return null;
    return {forwardSpeed: values[0], lateralSpeed: values[1], yawSpeed: values[2]};
  },
  label(parameters, speed, language) {
    const entry = String(parameters.speed_labels || "").split(";")
      .find((value) => value.startsWith(`${speed}:`));
    const labels = entry?.slice(entry.indexOf(":") + 1).split("|");
    return labels ? labels[language === "en" ? 1 : 0] : null;
  },
};
