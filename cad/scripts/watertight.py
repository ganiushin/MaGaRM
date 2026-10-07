"""Make OCC-exported STLs watertight without changing the geometry.

Two mesher artefacts occur in the vendor STEP models:
  * zero-area triangles (two vertices merged), which show up as zero-length 'open' edges -> removed;
  * single missing triangles (~0.3 mm) next to the r0.5 fillets of planar faces -> each open loop is closed with
    triangles on its own vertices, wound opposite to its neighbours. No vertex is moved or removed.
"""
import numpy as np
import trimesh
import networkx as nx


def ear_clip(V, loop):
    """triangulate a small planar loop on its own vertices, keeping the loop orientation"""
    P = V[loop]; n = np.zeros(3)
    for i in range(len(P)):                                   # Newell normal of the loop
        a, b = P[i], P[(i + 1) % len(P)]
        n += np.array([(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])])
    idx, tris = list(range(len(loop))), []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if np.dot(np.cross(P[i1] - P[i0], P[i2] - P[i1]), n) <= 0: continue        # reflex corner
            others = [j for j in idx if j not in (i0, i1, i2)]
            def inside(q):
                return all(np.dot(np.cross(P[b_] - P[a_], q - P[a_]), n) > 0 for a_, b_ in ((i0, i1), (i1, i2), (i2, i0)))
            if any(inside(P[j]) for j in others): continue
            tris.append([loop[i0], loop[i1], loop[i2]]); idx.pop(k); break
        else:
            tris.append([loop[idx[0]], loop[idx[1]], loop[idx[2]]]); idx.pop(1)
    tris.append([loop[i] for i in idx])
    return tris


def make_watertight(m: trimesh.Trimesh, max_loop=64) -> trimesh.Trimesh:
    m = trimesh.Trimesh(m.vertices.copy(), m.faces.copy(), process=False)
    m.merge_vertices(digits_vertex=4)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    for _ in range(3):
        open_idx = trimesh.grouping.group_rows(m.edges_sorted, require_count=1)
        if len(open_idx) == 0: break
        # directed boundary half-edges reversed (the fill must run opposite to its neighbours), split into simple loops
        G = nx.DiGraph(); G.add_edges_from((int(b), int(a)) for a, b in m.edges[open_idx])
        new = []
        for comp in nx.weakly_connected_components(G):
            sub = nx.DiGraph(G.subgraph(comp))
            while sub.number_of_edges():
                cyc = min((c for c in nx.simple_cycles(sub, length_bound=max_loop)), key=len, default=None)
                if cyc is None: break
                new += ear_clip(m.vertices, cyc)
                sub.remove_edges_from(zip(cyc, cyc[1:] + cyc[:1]))
        if not new: break
        m = trimesh.Trimesh(m.vertices, np.vstack([m.faces, np.array(new)]), process=False)
    return m


if __name__ == '__main__':
    import sys
    for f in sys.argv[1:]:
        m = trimesh.load(f, process=False); v0 = m.volume
        r = make_watertight(m)
        n_open = len(trimesh.grouping.group_rows(r.edges_sorted, require_count=1))
        print(f'{f}: watertight {r.is_watertight}, winding ok {r.is_winding_consistent}, open {n_open}, '
              f'volume {v0 / 1000:.3f} -> {r.volume / 1000:.3f} cm3')
