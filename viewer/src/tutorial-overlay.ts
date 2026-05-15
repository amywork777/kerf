/**
 * tutorial-overlay.ts
 *
 * Renders a translucent "first run" tour overlay with three stops pointing at
 * key UI elements.  Uses localStorage to remember dismissal so repeat visitors
 * never see it.
 *
 * Public API
 * ----------
 * mountTutorialOverlay()  — mounts if not yet dismissed; no-op otherwise.
 * resetTutorial()         — clears the localStorage flag (used by "Show tour").
 * showTutorial()          — unconditionally mounts the overlay.
 */

export const STORAGE_KEY = "kerf_tutorial_dismissed";

const STOPS = [
  {
    anchorId: "drop",
    label: "Drop JSON or pick an example here",
    arrowDir: "right" as const,
  },
  {
    anchorId: "actions2",
    label: "Feature insertion and export live here",
    arrowDir: "right" as const,
  },
  {
    anchorId: "status",
    label: "Rebuild times + parameters show up here",
    arrowDir: "right" as const,
  },
];

let currentOverlay: HTMLElement | null = null;

function buildOverlay(): HTMLElement {
  const overlay = document.createElement("div");
  overlay.id = "tutorial-overlay";
  overlay.setAttribute("role", "dialog");
  overlay.setAttribute("aria-label", "First-run tutorial");

  Object.assign(overlay.style, {
    position: "fixed",
    inset: "0",
    zIndex: "1000",
    background: "rgba(0,0,0,0.55)",
    display: "flex",
    alignItems: "flex-start",
    justifyContent: "flex-start",
    pointerEvents: "none",
  });

  // --- tour card in top-right of canvas ---
  const card = document.createElement("div");
  card.id = "tutorial-card";
  Object.assign(card.style, {
    position: "absolute",
    top: "24px",
    right: "24px",
    width: "260px",
    background: "#161b22",
    border: "1px solid #30363d",
    borderRadius: "8px",
    padding: "18px 20px",
    boxShadow: "0 8px 32px rgba(0,0,0,0.6)",
    pointerEvents: "auto",
    fontFamily: "-apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif",
    fontSize: "13px",
    color: "#e6edf3",
    lineHeight: "1.5",
  });

  const title = document.createElement("p");
  title.style.cssText = "margin:0 0 12px; font-weight:600; font-size:14px;";
  title.textContent = "Quick tour (3 stops)";
  card.appendChild(title);

  const stopList = document.createElement("ol");
  stopList.style.cssText = "margin:0 0 14px; padding-left:18px;";
  STOPS.forEach((s) => {
    const li = document.createElement("li");
    li.style.cssText = "margin-bottom:8px; color:#8b949e;";
    const anchor = document.createElement("strong");
    anchor.style.color = "#58a6ff";
    anchor.textContent = `#${s.anchorId}`;
    li.appendChild(anchor);
    li.appendChild(document.createTextNode(` — ${s.label}`));
    stopList.appendChild(li);
  });
  card.appendChild(stopList);

  const skipBtn = document.createElement("button");
  skipBtn.id = "tutorial-skip";
  skipBtn.textContent = "Skip tour";
  Object.assign(skipBtn.style, {
    background: "#0d1116",
    color: "#e6edf3",
    border: "1px solid #30363d",
    borderRadius: "4px",
    padding: "6px 14px",
    font: "inherit",
    cursor: "pointer",
    width: "100%",
  });
  skipBtn.addEventListener("click", dismissOverlay);
  card.appendChild(skipBtn);

  overlay.appendChild(card);

  // --- highlight rings around anchored elements ---
  STOPS.forEach((s) => {
    const el = document.getElementById(s.anchorId);
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const ring = document.createElement("div");
    Object.assign(ring.style, {
      position: "fixed",
      top: `${rect.top - 4}px`,
      left: `${rect.left - 4}px`,
      width: `${rect.width + 8}px`,
      height: `${rect.height + 8}px`,
      border: "2px solid #58a6ff",
      borderRadius: "6px",
      boxShadow: "0 0 0 4px rgba(88,166,255,0.18)",
      pointerEvents: "none",
    });
    overlay.appendChild(ring);
  });

  return overlay;
}

function dismissOverlay() {
  if (currentOverlay && currentOverlay.parentNode) {
    currentOverlay.parentNode.removeChild(currentOverlay);
  }
  currentOverlay = null;
  try {
    localStorage.setItem(STORAGE_KEY, "1");
  } catch {
    // storage unavailable — ignore
  }
}

/** Mount the overlay only if the user has not dismissed it before. */
export function mountTutorialOverlay(): void {
  try {
    if (localStorage.getItem(STORAGE_KEY)) return;
  } catch {
    // storage unavailable — show it anyway
  }
  showTutorial();
}

/** Unconditionally mount (or re-mount) the overlay. */
export function showTutorial(): void {
  // Remove any existing instance first.
  if (currentOverlay) dismissOverlayNoPersist();
  const overlay = buildOverlay();
  currentOverlay = overlay;
  document.body.appendChild(overlay);
}

function dismissOverlayNoPersist() {
  if (currentOverlay && currentOverlay.parentNode) {
    currentOverlay.parentNode.removeChild(currentOverlay);
  }
  currentOverlay = null;
}

/** Clear the localStorage flag so the tour will show again on next load. */
export function resetTutorial(): void {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch {
    // ignore
  }
}
