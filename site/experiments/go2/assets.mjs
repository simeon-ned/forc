import manifest from './model.json';
export { manifest };

export async function loadAssets(mj, progress) {
  let cache;
  try { cache = await caches.open('forc-menagerie-v1'); } catch { /* Private browsing can disable persistent storage. */ }
  let completed = 0;
  const queue = [...manifest.files];
  mj.FS.mkdirTree('/go2/assets');
  async function read(file) {
    const url = manifest.baseUrl + file.path;
    let response = await cache?.match(url);
    const cached = !!response;
    if (!response) response = await fetch(url, { signal: AbortSignal.timeout(60000) });
    if (!response.ok) throw new Error(`Could not download ${file.path} (${response.status}). Check your connection and retry.`);
    const bytes = new Uint8Array(await response.arrayBuffer());
    const hash = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), n => n.toString(16).padStart(2, '0')).join('');
    if (bytes.length !== file.bytes || hash !== file.sha256) {
      await cache?.delete(url);
      throw new Error(`Model integrity check failed for ${file.path}. Retry to download a fresh copy.`);
    }
    if (!cached && cache) { try { await cache.put(url, new Response(bytes)); } catch { /* Quota failure must not prevent simulation. */ } }
    mj.FS.writeFile('/go2/' + file.path, bytes);
    progress(++completed, manifest.files.length);
  }
  await Promise.all(Array.from({ length: 4 }, async () => { while (queue.length) await read(queue.shift()); }));
}

// Render MuJoCo's compiled mesh coordinates, not unprocessed OBJ coordinates.
export function describeScene(model) {
  const meshes = {};
  for (let i = 0; i < model.nmesh; i++) {
    const va = model.mesh_vertadr[i], vn = model.mesh_vertnum[i];
    const fa = model.mesh_faceadr[i], fn = model.mesh_facenum[i];
    meshes[i] = { vertices: model.mesh_vert.slice(3 * va, 3 * (va + vn)), faces: model.mesh_face.slice(3 * fa, 3 * (fa + fn)) };
  }
  const geoms = Array.from({ length: model.ngeom }, (_, i) => {
    const material = model.geom_matid[i];
    return { id: i, type: model.geom_type[i], mesh: model.geom_dataid[i], group: model.geom_group[i],
      size: Array.from(model.geom_size.slice(3 * i, 3 * i + 3)),
      rgba: Array.from(material >= 0 ? model.mat_rgba.slice(4 * material, 4 * material + 4) : model.geom_rgba.slice(4 * i, 4 * i + 4)) };
  });
  return { meshes, geoms };
}
