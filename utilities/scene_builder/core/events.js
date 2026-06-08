// Central event registration and editor lifecycle cleanup.
import { state } from './state.js';

const cleanupCallbacks = [];

export function listen(target, type, handler, options) {
  target.addEventListener(type, handler, options);
  return () => target.removeEventListener(type, handler, options);
}

export function startDraftAutosave(saveDraft, intervalMs = 1000) {
  const removeBeforeUnload = listen(window, "beforeunload", saveDraft);
  const intervalId = window.setInterval(saveDraft, intervalMs);
  const cleanup = () => {
    removeBeforeUnload();
    window.clearInterval(intervalId);
  };
  cleanupCallbacks.push(cleanup);
  return cleanup;
}

export function removeAllEventListeners() {
  while (cleanupCallbacks.length) {
    cleanupCallbacks.pop()();
  }
}

export function startEditorLifecycle(ctx) {
  const {
    gl,
    planeMeshPath,
    loadModel,
    loadAssets,
    loadVehicleDefs,
    loadScenes,
    restoreDraft,
    saveDraft,
    syncInspector,
    setStatus,
    render,
  } = ctx;

  startDraftAutosave(saveDraft);
  if (!gl) {
    setStatus("WebGL is not available in this browser");
    return;
  }

  loadModel(planeMeshPath).catch(() => {});
  loadAssets().catch((error) => setStatus(error.message));
  loadVehicleDefs().catch(() => {});
  loadScenes().catch((error) => setStatus(error.message));
  restoreDraft()
    .catch((error) => setStatus(`Could not restore draft: ${error.message}`))
    .finally(() => {
      state.draftReady = true;
    });
  syncInspector();
  render();
}
