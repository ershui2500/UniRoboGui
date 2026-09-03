const buttons = [...document.querySelectorAll("[data-target-workspace]")];
const panels = [...document.querySelectorAll("[data-workspace]")];
const sidebar = document.getElementById("sidebar");
const backdrop = document.getElementById("sidebarBackdrop");
const mobileMenu = document.getElementById("mobileMenu");
const sidebarClose = document.getElementById("sidebarClose");
const dashboard = document.querySelector(".dashboard");
const workspaceTitle = document.getElementById("workspaceTitle");
const workspaceEmblem = document.getElementById("workspaceEmblem");
const workspaceGlyph = document.getElementById("workspaceGlyph");

const consoleRegions = new Set(["control", "perception", "voice"]);
const workspaceTitles = {
  console: "综合工作台",
  robot: "机器人状态",
  "joint-debug": "机器人调试台",
  diagnostics: "诊断",
};
const workspaceGlyphs = {
  console: "WS",
  robot: "3D",
  "joint-debug": "JD",
  diagnostics: "DG",
};

let currentWorkspace = "console";
let currentRegion = null;

function closeSidebar() {
  document.body.classList.remove("sidebar-open");
  mobileMenu.setAttribute("aria-expanded", "false");
  backdrop.hidden = true;
}

function openSidebar() {
  document.body.classList.add("sidebar-open");
  mobileMenu.setAttribute("aria-expanded", "true");
  backdrop.hidden = false;
}

function emitRegionChange(region, reason = "navigation") {
  const previous = currentRegion;
  currentRegion = region;
  window.dispatchEvent(new CustomEvent("g1:console-region-change", {
    detail: { region, previous, reason },
  }));
}

function setupRobotViewerDock() {
  const panel = document.querySelector(".robot-viewer-panel");
  const consoleSlot = document.getElementById("robotViewerConsoleSlot");
  const debugSlot = document.getElementById("jointDebugViewerSlot");
  const homeMarker = document.createComment("robot-viewer-panel-home");
  panel.parentNode.insertBefore(homeMarker, panel);

  function place(workspace) {
    const dockInConsole = workspace === "console";
    const dockInDebug = workspace === "joint-debug";
    const targetParent = dockInConsole
      ? consoleSlot.parentNode
      : dockInDebug ? debugSlot.parentNode : homeMarker.parentNode;
    if (panel.parentNode === targetParent && panel.classList.contains("console-docked") === dockInConsole) return;

    if (dockInConsole) consoleSlot.parentNode.insertBefore(panel, consoleSlot);
    else if (dockInDebug) debugSlot.parentNode.insertBefore(panel, debugSlot);
    else homeMarker.parentNode.insertBefore(panel, homeMarker.nextSibling);
    panel.classList.toggle("console-docked", dockInConsole);
    panel.classList.toggle("joint-debug-docked", dockInDebug);
    window.dispatchEvent(new CustomEvent("g1:robot-viewer-dock-change", {
      detail: { workspace, docked: dockInConsole },
    }));
  }

  return { place };
}

function setupRunDetailsDrawer() {
  const dialog = document.getElementById("runDetailsDrawer");
  const body = document.getElementById("runDetailsDrawerBody");
  const trigger = document.getElementById("runDetailsButton");
  const sections = [...document.querySelectorAll("[data-run-details]")].map((section) => {
    const marker = document.createComment("run-details-home");
    section.parentNode.insertBefore(marker, section);
    return { section, marker };
  });

  function restoreSections() {
    sections.forEach(({ section, marker }) => {
      marker.parentNode.insertBefore(section, marker.nextSibling);
      section.classList.remove("drawer-section");
      section.hidden = currentWorkspace !== "robot";
    });
  }

  trigger.addEventListener("click", () => {
    emitRegionChange("details", "run_details");
    sections.forEach(({ section }) => {
      section.hidden = false;
      section.classList.add("drawer-section");
      body.append(section);
    });
    dialog.showModal();
  });
  dialog.addEventListener("close", restoreSections);
}

function setupPanelDrawer({ dialogId, bodyId, panelId, triggerId }) {
  const dialog = document.getElementById(dialogId);
  const body = document.getElementById(bodyId);
  const panel = document.getElementById(panelId);
  const trigger = document.getElementById(triggerId);
  const marker = document.createComment(`${panelId}-home`);
  panel.parentNode.insertBefore(marker, panel);

  function restorePanel() {
    if (panel.parentNode !== marker.parentNode) {
      marker.parentNode.insertBefore(panel, marker.nextSibling);
    }
    panel.classList.remove("drawer-expanded");
    trigger.textContent = panelId === "robot-control" ? "高级控制抽屉" : "完整语音面板";
    trigger.setAttribute("aria-expanded", "false");
  }

  function openPanel() {
    emitRegionChange("drawer", "advanced_drawer");
    body.append(panel);
    panel.classList.add("drawer-expanded");
    trigger.textContent = "返回综合工作台";
    trigger.setAttribute("aria-expanded", "true");
    dialog.showModal();
  }

  trigger.setAttribute("aria-expanded", "false");
  trigger.addEventListener("click", () => {
    if (dialog.open) dialog.close();
    else openPanel();
  });
  dialog.addEventListener("close", restorePanel);
  return { dialog, restorePanel };
}

setupRunDetailsDrawer();
const robotViewerDock = setupRobotViewerDock();
const voiceDrawer = setupPanelDrawer({
  dialogId: "voiceWorkspaceDrawer",
  bodyId: "voiceWorkspaceDrawerBody",
  panelId: "voice-interaction",
  triggerId: "expandVoicePanel",
});

const managedDrawers = [...document.querySelectorAll("dialog.workspace-drawer")];
document.querySelectorAll("[data-close-drawer]").forEach((button) => {
  button.addEventListener("click", () => document.getElementById(button.dataset.closeDrawer)?.close());
});
function closeDrawers() {
  managedDrawers.forEach((dialog) => {
    if (dialog.open) dialog.close();
  });
  voiceDrawer.restorePanel();
}

function setButtonState(workspace, region) {
  buttons.forEach((button) => {
    const target = button.dataset.targetWorkspace;
    const active = workspace === "console" ? target === "console" : target === workspace;
    button.classList.toggle("active", active);
    if (active) button.setAttribute("aria-current", "page");
    else button.removeAttribute("aria-current");
  });
}

function focusConsoleRegion(region) {
  if (!region) return;
  const panel = document.querySelector(`[data-workspace="${region}"]`);
  if (!panel) return;
  panel.classList.remove("region-focus-pulse");
  requestAnimationFrame(() => panel.classList.add("region-focus-pulse"));
  window.setTimeout(() => panel.classList.remove("region-focus-pulse"), 900);
}

function showWorkspace(requested, updateHash = true) {
  const alias = requested === "overview" ? "console" : requested;
  const region = consoleRegions.has(alias) ? alias : null;
  const workspace = region ? "console" : (workspaceTitles[alias] ? alias : "console");

  closeDrawers();
  robotViewerDock.place(workspace);
  panels.forEach((panel) => {
    if (!panel.dataset.workspace) return;
    panel.hidden = workspace === "console"
      ? !consoleRegions.has(panel.dataset.workspace)
      : panel.dataset.workspace !== workspace;
  });

  currentWorkspace = workspace;
  dashboard.classList.toggle("console-active", workspace === "console");
  dashboard.classList.toggle("joint-debug-active", workspace === "joint-debug");
  document.body.classList.toggle("console-workspace-active", workspace === "console");
  document.body.classList.toggle("joint-debug-workspace-active", workspace === "joint-debug");
  document.documentElement.classList.toggle("console-workspace-active", workspace === "console");
  document.documentElement.classList.toggle("joint-debug-workspace-active", workspace === "joint-debug");
  setButtonState(workspace, region);
  workspaceTitle.textContent = workspaceTitles[workspace];
  workspaceEmblem.dataset.workspace = workspace;
  workspaceGlyph.textContent = workspaceGlyphs[workspace];
  document.title = `${workspaceTitles[workspace]} · UniRoboGui`;

  const nextHash = region || workspace;
  if (updateHash && location.hash !== `#${nextHash}`) {
    history.replaceState(null, "", `#${nextHash}`);
  }

  window.dispatchEvent(new CustomEvent("g1:workspace-change", {
    detail: { workspace, region },
  }));
  emitRegionChange(region, "workspace");
  closeSidebar();
  window.scrollTo({ top: 0, behavior: "instant" });
  if (workspace === "console") requestAnimationFrame(() => focusConsoleRegion(region));
}

buttons.forEach((button) => {
  button.addEventListener("click", () => showWorkspace(button.dataset.targetWorkspace));
});

consoleRegions.forEach((region) => {
  const panel = document.querySelector(`[data-workspace="${region}"]`);
  panel?.addEventListener("pointerdown", () => {
    if (currentWorkspace !== "console" || panel.closest("dialog[open]")) return;
    if (currentRegion !== region) {
      emitRegionChange(region, "panel_interaction");
      setButtonState("console", region);
    }
  }, { capture: true });
});

mobileMenu.addEventListener("click", openSidebar);
sidebarClose.addEventListener("click", closeSidebar);
backdrop.addEventListener("click", closeSidebar);
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && document.body.classList.contains("sidebar-open")) closeSidebar();
});
window.addEventListener("hashchange", () => showWorkspace(location.hash.slice(1), false));

showWorkspace(location.hash.slice(1) || "console", false);
