// Native file save/open via the File System Access API, with a graceful
// fallback to <a download> + <input type="file"> for browsers that don't
// support it (Firefox, Safari at the time of writing).
//
// The store-and-reuse semantic is the SW-feature: once the user picks a
// save location, subsequent saves write back to the same file without a
// dialog.

export interface OpenedFile {
  name: string;
  text: string;
}

export interface SavedFile {
  name: string;
}

export interface FileHandle {
  isNativeSupported(): boolean;
  /** Prompt the user to pick a file. Caches the handle so saveExisting works. */
  open(accept: { description: string; mimes: string[]; extensions: string[] }): Promise<OpenedFile | null>;
  /** Prompt for a save location, write `content`, cache the handle. */
  saveAs(
    content: string,
    suggestedName: string,
    accept: { description: string; mimes: string[]; extensions: string[] },
  ): Promise<SavedFile | null>;
  /** Write `content` to the cached handle. Falls back to saveAs if no handle. */
  saveExisting(
    content: string,
    suggestedName: string,
    accept: { description: string; mimes: string[]; extensions: string[] },
  ): Promise<SavedFile | null>;
  /** True once a handle has been cached (after open or saveAs). */
  hasHandle(): boolean;
  /** Name of the last opened/saved file, or null. */
  currentName(): string | null;
}

interface FsApi {
  showOpenFilePicker?: (opts: unknown) => Promise<unknown[]>;
  showSaveFilePicker?: (opts: unknown) => Promise<unknown>;
}

interface NativeHandle {
  name: string;
  getFile(): Promise<File>;
  createWritable(): Promise<{ write(chunk: string): Promise<void>; close(): Promise<void> }>;
}

function buildNativePickerOpts(accept: {
  description: string;
  mimes: string[];
  extensions: string[];
}, suggestedName?: string): unknown {
  const acceptMap: Record<string, string[]> = {};
  for (const m of accept.mimes) acceptMap[m] = accept.extensions;
  const opts: Record<string, unknown> = {
    types: [{ description: accept.description, accept: acceptMap }],
  };
  if (suggestedName) opts.suggestedName = suggestedName;
  return opts;
}

export function createFileHandle(): FileHandle {
  const fs = window as unknown as FsApi;
  let handle: NativeHandle | null = null;
  let lastName: string | null = null;

  const native = typeof fs.showSaveFilePicker === "function"
    && typeof fs.showOpenFilePicker === "function";

  return {
    isNativeSupported: () => native,
    hasHandle: () => handle !== null,
    currentName: () => lastName,

    async open(accept) {
      if (native) {
        try {
          const handles = await fs.showOpenFilePicker!(buildNativePickerOpts(accept));
          const h = handles[0] as NativeHandle | undefined;
          if (!h) return null;
          handle = h;
          lastName = h.name;
          const file = await h.getFile();
          return { name: h.name, text: await file.text() };
        } catch (e) {
          // User cancelled, or call failed (e.g. permission policy). Treat
          // as "no file selected" — don't blow up.
          if (isAbortError(e)) return null;
          throw e;
        }
      }
      // Fallback: programmatically click a hidden <input type="file">.
      return openViaInput(accept);
    },

    async saveAs(content, suggestedName, accept) {
      if (native) {
        try {
          const h = (await fs.showSaveFilePicker!(
            buildNativePickerOpts(accept, suggestedName),
          )) as NativeHandle;
          handle = h;
          lastName = h.name;
          const w = await h.createWritable();
          await w.write(content);
          await w.close();
          return { name: h.name };
        } catch (e) {
          if (isAbortError(e)) return null;
          throw e;
        }
      }
      // Fallback: anchor download. No handle is retained.
      downloadViaAnchor(content, suggestedName);
      lastName = suggestedName;
      return { name: suggestedName };
    },

    async saveExisting(content, suggestedName, accept) {
      if (native && handle) {
        const w = await handle.createWritable();
        await w.write(content);
        await w.close();
        return { name: handle.name };
      }
      // No cached handle — first save behaves like saveAs.
      return this.saveAs(content, suggestedName, accept);
    },
  };
}

function isAbortError(e: unknown): boolean {
  return typeof e === "object" && e !== null && (e as { name?: string }).name === "AbortError";
}

function downloadViaAnchor(content: string, name: string): void {
  const blob = new Blob([content], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function openViaInput(accept: {
  description: string;
  mimes: string[];
  extensions: string[];
}): Promise<OpenedFile | null> {
  return new Promise((resolve) => {
    const input = document.createElement("input");
    input.type = "file";
    input.accept = accept.extensions.join(",");
    input.addEventListener("change", async () => {
      const f = input.files?.[0];
      if (!f) {
        resolve(null);
        return;
      }
      resolve({ name: f.name, text: await f.text() });
    });
    // Cancel detection is best-effort: the input fires no event on cancel,
    // so a Promise that never resolves would leak. Resolve to null on the
    // next microtask if no file was chosen — but we only know that *after*
    // a focus event fires on the window. Pragmatic: accept the leak; the
    // GC will collect the closure once the Promise drops out of scope.
    input.click();
  });
}
