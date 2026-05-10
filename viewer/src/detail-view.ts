// detail-view.ts — extracts a zoomed region from a source canvas view and
// renders it as a separate panel with a leader line back to the region.
// No Three.js WASM — pure canvas-to-canvas copy + transform.

export interface DetailViewParams {
  baseView: "front" | "top" | "side";
  /** Rectangle in base-view pixel space (relative to the view's top-left). */
  rect: { x: number; y: number; w: number; h: number };
  /** Scale factor — default 2. */
  zoom?: number;
  /** Short identifier shown in the view label, e.g. "A". */
  label: string;
}

export interface DetailViewResult {
  /** Width of the rendered detail panel, px. */
  outW: number;
  /** Height of the rendered detail panel, px. */
  outH: number;
}

/**
 * Copies a rectangular sub-region from `srcCanvas` (the already-rendered
 * base view) into `destCtx`, scaled by `zoom`, starting at (`destX`, `destY`).
 * Also draws:
 *   1. A dashed highlight rectangle on `srcCtx` showing the detail region.
 *   2. A leader line from the highlighted region to the detail panel.
 *   3. A border + label on the detail panel.
 *
 * @param srcCanvas  The off-screen (or shared sheet) canvas that holds the base view.
 * @param srcRegionX Left edge of the base view on `srcCanvas`, in sheet coords.
 * @param srcRegionY Top edge of the base view on `srcCanvas`, in sheet coords.
 * @param params     DetailViewParams (rect is relative to the base-view origin).
 * @param destCtx    2-D context of the destination sheet canvas.
 * @param destX      Where to place the detail panel (sheet X).
 * @param destY      Where to place the detail panel (sheet Y).
 */
export function renderDetailPanel(
  srcCanvas: HTMLCanvasElement | OffscreenCanvas,
  srcRegionX: number,
  srcRegionY: number,
  params: DetailViewParams,
  destCtx: CanvasRenderingContext2D,
  destX: number,
  destY: number,
): DetailViewResult {
  const zoom = params.zoom ?? 2;
  const { x, y, w, h } = params.rect;
  const outW = Math.round(w * zoom);
  const outH = Math.round(h * zoom);

  // 1. Draw detail panel background + border.
  destCtx.fillStyle = "#f8fafc";
  destCtx.fillRect(destX, destY, outW, outH);
  destCtx.strokeStyle = "#30363d";
  destCtx.lineWidth = 1.5;
  destCtx.strokeRect(destX + 0.5, destY + 0.5, outW - 1, outH - 1);

  // 2. Blit the zoomed sub-region from the source canvas.
  destCtx.drawImage(
    srcCanvas,
    srcRegionX + x,  // src x
    srcRegionY + y,  // src y
    w,               // src width
    h,               // src height
    destX,           // dst x
    destY,           // dst y
    outW,            // dst width
    outH,            // dst height
  );

  // Re-draw border on top of the blitted image.
  destCtx.strokeStyle = "#1a3a6b";
  destCtx.lineWidth = 1.5;
  destCtx.strokeRect(destX + 0.5, destY + 0.5, outW - 1, outH - 1);

  // 3. Label: "DETAIL A (×zoom)" above the panel.
  destCtx.font = "bold 14px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  destCtx.fillStyle = "#161b22";
  destCtx.fillText(`DETAIL ${params.label}  ×${zoom}`, destX, destY - 6);

  // 4. Highlight rectangle in the base view.
  const srcAbsX = srcRegionX + x;
  const srcAbsY = srcRegionY + y;
  destCtx.setLineDash([4, 3]);
  destCtx.strokeStyle = "#e07b00";
  destCtx.lineWidth = 1.5;
  destCtx.strokeRect(srcAbsX + 0.5, srcAbsY + 0.5, w - 1, h - 1);
  destCtx.setLineDash([]);

  // 5. Circle marker at centre of the highlight region.
  const markerX = srcAbsX + w / 2;
  const markerY = srcAbsY + h / 2;
  const markerR = 6;
  destCtx.beginPath();
  destCtx.arc(markerX, markerY, markerR, 0, Math.PI * 2);
  destCtx.strokeStyle = "#e07b00";
  destCtx.lineWidth = 1.5;
  destCtx.stroke();
  // Label letter inside circle.
  destCtx.font = "bold 9px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  destCtx.fillStyle = "#e07b00";
  destCtx.textAlign = "center";
  destCtx.textBaseline = "middle";
  destCtx.fillText(params.label, markerX, markerY);
  destCtx.textAlign = "start";
  destCtx.textBaseline = "alphabetic";

  // 6. Leader line from the circle to the detail panel.
  const leaderStartX = markerX + (destX > markerX ? markerR : -markerR);
  const leaderStartY = markerY;
  const leaderEndX = destX > markerX ? destX : destX + outW;
  const leaderEndY = destY + outH / 2;
  destCtx.beginPath();
  destCtx.moveTo(leaderStartX, leaderStartY);
  // Elbow midpoint.
  destCtx.lineTo(leaderEndX + (destX > markerX ? -20 : 20), leaderStartY);
  destCtx.lineTo(leaderEndX, leaderEndY);
  destCtx.strokeStyle = "#e07b00";
  destCtx.lineWidth = 1;
  destCtx.stroke();

  return { outW, outH };
}

// ---------------------------------------------------------------------------
// Pure geometry helper used by tests — no DOM required.
// ---------------------------------------------------------------------------

/**
 * Computes the output dimensions of a detail view without touching the DOM.
 * Useful for pre-calculating layout before canvas allocation.
 */
export function detailViewOutputSize(
  rect: { w: number; h: number },
  zoom: number,
): { outW: number; outH: number } {
  return {
    outW: Math.round(rect.w * zoom),
    outH: Math.round(rect.h * zoom),
  };
}
