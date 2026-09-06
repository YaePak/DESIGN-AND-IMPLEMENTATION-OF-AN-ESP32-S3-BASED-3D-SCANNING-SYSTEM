import argparse
import struct
import sys

import numpy as np


# ---------------------------------------------------------------- đọc file
def load_xyz(path):
    pts = np.loadtxt(path, delimiter=",", dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        sys.exit(f"File {path} không đúng định dạng x,y,z mỗi dòng.")
    return pts


# ---------------------------------------------------------------- phương pháp GRID
def build_grid(pts, samples, layer_mm):
    """
    Dựng lại lưới [lớp][góc] = bán kính từ x,y,z.
    Góc được khôi phục bằng atan2(y, x), lớp bằng z / layer_mm.
    Ô nào không có điểm (do firmware lọc bỏ) → NaN, sẽ nội suy sau.
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.hypot(x, y)
    theta = np.mod(np.arctan2(y, x), 2 * np.pi)
    d_angle = 2 * np.pi / samples

    ai = np.rint(theta / d_angle).astype(int) % samples
    zi = np.rint(z / layer_mm).astype(int)
    zi -= zi.min()
    n_layers = zi.max() + 1

    grid_sum = np.zeros((n_layers, samples))
    grid_cnt = np.zeros((n_layers, samples))
    np.add.at(grid_sum, (zi, ai), r)
    np.add.at(grid_cnt, (zi, ai), 1)

    grid = np.full((n_layers, samples), np.nan)
    mask = grid_cnt > 0
    grid[mask] = grid_sum[mask] / grid_cnt[mask]

    z0 = z.min()
    return grid, z0


def fill_missing(grid):
    """Nội suy tuyến tính theo vòng tròn cho các ô thiếu trong từng lớp."""
    n_layers, samples = grid.shape
    keep = np.ones(n_layers, dtype=bool)
    idx = np.arange(samples)
    for j in range(n_layers):
        row = grid[j]
        valid = ~np.isnan(row)
        if valid.sum() == 0:
            keep[j] = False
            continue
        if valid.all():
            continue
        # Mở rộng tuần hoàn để nội suy qua điểm nối 0°/360°
        vi = idx[valid]
        vv = row[valid]
        vi_ext = np.concatenate([vi - samples, vi, vi + samples])
        vv_ext = np.concatenate([vv, vv, vv])
        row[~valid] = np.interp(idx[~valid], vi_ext, vv_ext)
    return grid[keep], keep


def smooth_grid(grid, passes):
    """Lọc trung bình 3×3 (tuần hoàn theo góc), lặp `passes` lần. Giảm nhiễu cảm biến."""
    g = grid.copy()
    for _ in range(passes):
        # theo góc (tuần hoàn)
        g = (np.roll(g, 1, axis=1) + g + np.roll(g, -1, axis=1)) / 3.0
        # theo z (giữ nguyên hai lớp biên)
        if g.shape[0] >= 3:
            inner = (g[:-2] + g[1:-1] + g[2:]) / 3.0
            g[1:-1] = inner
    return g


def grid_to_mesh(grid, samples, layer_mm, z0):
    """
    Sinh đỉnh + tam giác từ lưới bán kính.
    Vành: nối 4 điểm kề nhau (i,j),(i+1,j),(i,j+1),(i+1,j+1) thành 2 tam giác.
    Đáy/đỉnh: một đỉnh tâm + quạt tam giác → mesh kín hoàn toàn.
    """
    n_layers = grid.shape[0]
    theta = np.arange(samples) * (2 * np.pi / samples)

    verts = []
    for j in range(n_layers):
        zj = z0 + j * layer_mm
        rj = grid[j]
        verts.append(np.column_stack([rj * np.cos(theta), rj * np.sin(theta), np.full(samples, zj)]))
    verts = np.vstack(verts)

    faces = []
    for j in range(n_layers - 1):
        for i in range(samples):
            a = j * samples + i
            b = j * samples + (i + 1) % samples
            c = (j + 1) * samples + i
            d = (j + 1) * samples + (i + 1) % samples
            faces.append((a, b, d))
            faces.append((a, d, c))

    # Đóng đáy và đỉnh
    bottom_c = len(verts)
    top_c = len(verts) + 1
    verts = np.vstack([verts, [0, 0, z0], [0, 0, z0 + (n_layers - 1) * layer_mm]])
    for i in range(samples):
        a = i
        b = (i + 1) % samples
        faces.append((bottom_c, b, a))
        ta = (n_layers - 1) * samples + i
        tb = (n_layers - 1) * samples + (i + 1) % samples
        faces.append((top_c, ta, tb))

    faces = np.array(faces, dtype=np.int64)
    return verts, faces


def ensure_outward(verts, faces):
    """Nếu thể tích có dấu âm → pháp tuyến đang hướng vào trong → đảo thứ tự đỉnh."""
    v0 = verts[faces[:, 0]]
    v1 = verts[faces[:, 1]]
    v2 = verts[faces[:, 2]]
    vol = np.einsum("ij,ij->i", v0, np.cross(v1, v2)).sum() / 6.0
    if vol < 0:
        faces = faces[:, [0, 2, 1]]
    return faces, abs(vol)


def write_stl_binary(path, verts, faces):
    v0 = verts[faces[:, 0]]
    v1 = verts[faces[:, 1]]
    v2 = verts[faces[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0)
    lens = np.linalg.norm(normals, axis=1)
    lens[lens == 0] = 1.0
    normals /= lens[:, None]

    with open(path, "wb") as f:
        f.write(b"xyz_to_stl grid method".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(faces)))
        for n, a, b, c in zip(normals, v0, v1, v2):
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def build_stl(pts, args):
    grid, z0 = build_grid(pts, args.samples, args.layer)
    grid, keep = fill_missing(grid)
    dropped_layers = int((~keep).sum())
    if args.smooth > 0:
        grid = smooth_grid(grid, args.smooth)
    verts, faces = grid_to_mesh(grid, args.samples, args.layer, z0)
    faces, vol = ensure_outward(verts, faces)
    write_stl_binary(args.output, verts, faces)

    print(f"{len(pts)} điểm → lưới {grid.shape[0]} lớp × {args.samples} mẫu")
    if dropped_layers:
        print(f"Bỏ {dropped_layers} lớp trống hoàn toàn")
    print(f"{len(verts)} đỉnh, {len(faces)} tam giác, mesh kín, thể tích ≈ {vol/1000:.1f} cm³")
    print(f"Kích thước bao: X {verts[:,0].min():.1f}..{verts[:,0].max():.1f}  "
          f"Y {verts[:,1].min():.1f}..{verts[:,1].max():.1f}  "
          f"Z {verts[:,2].min():.1f}..{verts[:,2].max():.1f} mm")
    print(f"Đã ghi {args.output}")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Chuyển XYZ → STL không cần MeshLab")
    ap.add_argument("input", help="file x,y,z (ví dụ scan_received.txt)")
    ap.add_argument("-o", "--output", default=None, help="file STL đầu ra (mặc định: cùng tên .stl)")
    ap.add_argument("--samples", type=int, default=80, help="số mẫu góc mỗi vòng (STEPS_PER_REV trong firmware)")
    ap.add_argument("--layer", type=float, default=1.0, help="chiều cao lớp mm (Z_LAYER_MM trong firmware)")
    ap.add_argument("--smooth", type=int, default=0, help="số lần lọc trung bình làm mượt (0 = không lọc)")
    args = ap.parse_args()

    if args.output is None:
        args.output = args.input.rsplit(".", 1)[0] + ".stl"

    pts = load_xyz(args.input)
    build_stl(pts, args)
    

if __name__ == "__main__":
    main()