function renderBatteryCard(card, battery, source) {
  const $ = (id) => card.querySelector(`[data-battery-field="${id}"]`) || document.getElementById(id);
  const setText = (id, value) => { $(id).textContent = value; };
  const fresh = source && source.status === "online";
  const soc = fresh ? Math.max(0, Math.min(100, Number(battery.soc_pct) || 0)) : 0;
  $("batteryRing").style.setProperty("--soc", soc);
  setText("batterySoc", fresh ? `${integer(battery.soc_pct)}%` : "--");
  setText("batterySoh", fresh ? `${integer(battery.soh_pct)}%` : "--");
  setText("batteryCycle", fresh ? integer(battery.cycle) : "--");
  setText("batteryCurrent", fresh ? integer(battery.current_raw) : "--");
  setText("batteryVersion", fresh ? battery.version || "--" : "--");

  const temperatures = fresh && Array.isArray(battery.temperature_raw)
    ? battery.temperature_raw.filter(Number.isFinite)
    : [];
  setText(
    "batteryTemp",
    temperatures.length ? integer(Math.max(...temperatures)) : "--",
  );

  const cells = fresh && Array.isArray(battery.cell_voltage_raw)
    ? battery.cell_voltage_raw.filter((value) => Number(value) > 0)
    : [];
  setText(
    "batteryCells",
    cells.length
      ? `${integer(Math.min(...cells))}–${integer(Math.max(...cells))}`
      : "--",
  );
}

function renderBatteries(batteries, battery, source) {
  const original = document.querySelector(".battery-panel:not([data-battery-copy])");
  const copies = [...document.querySelectorAll("[data-battery-copy]")];
  original.hidden = batteries.length > 0 || !telemetrySourceDeclared("bms");
  if (!batteries.length) {
    copies.forEach((card) => card.remove());
    renderBatteryCard(original, battery, source);
    setText("topBattery", source?.status === "online" ? `${integer(battery.soc_pct)}%` : "--");
    return;
  }
  copies.filter((card) => !batteries.some((item) => item.id === card.dataset.batteryCopy))
    .forEach((card) => card.remove());
  for (const item of batteries) {
    let card = copies.find((element) => element.dataset.batteryCopy === item.id);
    if (!card) {
      card = original.cloneNode(true);
      card.dataset.batteryCopy = item.id;
      card.removeAttribute("data-telemetry-source");
      card.querySelectorAll("[id]").forEach((element) => {
        element.dataset.batteryField = element.id;
        element.removeAttribute("id");
      });
      original.parentNode.insertBefore(card, original);
    }
    card.hidden = false;
    card.querySelector("h3").textContent = manifestText(`电池 ${item.id}`, `Battery ${item.id}`);
    card.querySelector(".tag").textContent = `${item.topic} · ${manifestText(
      item.state?.status === "online" ? "在线" : item.state?.status === "delayed" ? "数据延迟" : "离线",
      item.state?.status === "online" ? "Online" : item.state?.status === "delayed" ? "Delayed" : "Offline")}`;
    renderBatteryCard(card, item, item.state);
  }
  setText("topBattery", batteries.map((item) => `${item.id}: ${
    item.state?.status === "online" ? `${integer(item.soc_pct)}%` : "--"}`).join(" / "));
}

// Keep each pack's freshness independent; the legacy battery is never a fallback for another pack.
function renderDiagnosticBatteries(data, sohSupported) {
  const original = document.getElementById("diagnosticBatteryHealth").closest("article");
  const copies = [...document.querySelectorAll("[data-diagnostic-battery-copy]")];
  const packs = Array.isArray(data.batteries) ? data.batteries : [];
  original.hidden = packs.length > 0 || !telemetrySourceDeclared("bms");
  copies.filter((card) => !packs.some((pack) => pack.id === card.dataset.diagnosticBatteryCopy))
    .forEach((card) => card.remove());
  if (!packs.length) {
    const fresh = telemetrySourceFresh(data.sources, "bms");
    renderBatteryDiagnostics(original, fresh ? data.battery || {} : {}, fresh, sohSupported);
    return;
  }
  for (const pack of packs) {
    let card = copies.find((element) => element.dataset.diagnosticBatteryCopy === pack.id);
    if (!card) {
      card = original.cloneNode(true);
      card.dataset.diagnosticBatteryCopy = pack.id;
      card.querySelectorAll("[id]").forEach((element) => element.removeAttribute("id"));
      original.parentNode.insertBefore(card, original);
    }
    card.hidden = false;
    card.querySelector("h3").textContent = manifestText(`电池 ${pack.id}`, `Battery ${pack.id}`);
    card.querySelector(".tag").textContent = `${pack.topic} · ${manifestText(
      statusLabels[pack.state?.status] || "离线",
      {online: "Online", delayed: "Delayed", offline: "Offline"}[pack.state?.status] || "Offline")}`;
    const fresh = pack.state?.status === "online";
    renderBatteryDiagnostics(card, fresh ? pack : {}, fresh, sohSupported);
  }
}
