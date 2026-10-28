import argparse
import importlib.util
import sys
import warnings
from pathlib import Path

import numpy as np


def load(path):
    rows = [l for l in Path(path).read_text(encoding="utf-8-sig").splitlines()
            if l.strip() and not l.lstrip().startswith("#")]
    p = np.loadtxt(rows, delimiter=",", ndmin=2)
    if p.shape[1] != 3 or len(p) < 10:
        raise ValueError("File can it nhat 10 dong x,y,z")
    if not np.isfinite(p).all():
        raise ValueError("Co toa do NaN/Inf")
    return p[np.hypot(p[:, 0], p[:, 1]) >= 0.5]


def guess_samples(theta, r):
    if (r >= 5).sum() >= 10:
        theta = theta[r >= 5]
    for n in range(3, 2001):
        step = 2 * np.pi / n
        if np.all(np.abs(theta - np.rint(theta / step) * step) <= 0.2 * step):
            return n
    raise ValueError("Khong tu nhan duoc so goc/vong. Dung --samples (firmware: 200)")


def unoffset(p, e, n):
    step = 2 * np.pi / n
    th = np.rint(np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi) / step) * step
    r = np.hypot(p[:, 0], p[:, 1])
    x, y = r * np.cos(th) - e * np.sin(th), r * np.sin(th) + e * np.cos(th)
    a, out = np.arange(n) * step, []
    for z in np.unique(p[:, 2]):
        m = p[:, 2] == z
        if m.sum() < 3:
            continue
        phi, rho = np.mod(np.arctan2(y[m], x[m]), 2 * np.pi), np.hypot(x[m], y[m])
        o = np.argsort(phi)
        pe = np.r_[phi[o][-1] - 2 * np.pi, phi[o], phi[o][0] + 2 * np.pi]
        re = np.r_[rho[o][-1], rho[o], rho[o][0]]
        j = np.clip(np.searchsorted(pe, a), 1, len(pe) - 1)
        lo, hi = pe[j - 1], pe[j]
        ok = hi - lo <= 2.5 * step
        v = re[j - 1] + (re[j] - re[j - 1]) * (a - lo) / np.maximum(hi - lo, 1e-9)
        out.append(np.column_stack([v[ok] * np.cos(a[ok]), v[ok] * np.sin(a[ok]), np.full(ok.sum(), z)]))
    if not out:
        raise ValueError("Khong con diem sau khi bu lech tia")
    return np.vstack(out)


def need_scipy(what):
    if importlib.util.find_spec("scipy") is None:
        raise ValueError(f"{what} can thu vien scipy: pip install scipy")


def square_dist(xy, cx, cy, a, h):
    ca, sa = np.cos(a), np.sin(a)
    x, y = xy[:, 0] - cx, xy[:, 1] - cy
    return np.maximum(np.abs(ca * x + sa * y), np.abs(-sa * x + ca * y)) - h


def square_fit(xy, starts=(0, 30, 60)):
    from scipy.optimize import minimize
    c0 = xy.mean(0)

    def cost(v):
        d = square_dist(xy, *v)
        return np.mean(np.minimum(d * d, 4.0))

    best = None
    for a0 in np.radians(starts):
        h0 = np.median(square_dist(xy, c0[0], c0[1], a0, 0))
        r = minimize(cost, [c0[0], c0[1], a0, h0], method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-5, maxiter=2000))
        if best is None or r.fun < best.fun:
            best = r
    cx, cy, a, h = best.x
    return float(np.sqrt(best.fun)), (cx, cy, np.mod(a + np.pi / 4, np.pi / 2) - np.pi / 4, abs(h))


def square_score(p, e, n, zs):
    q = unoffset(p, e, n)
    res, sides = [], []
    for z in zs:
        xy = q[q[:, 2] == z][:, :2]
        if len(xy) >= 0.5 * n:
            r, sq = square_fit(xy)
            res.append(r)
            sides.append(2 * sq[3])
    if not res:
        return np.inf, np.nan
    return float(np.median(res)), float(np.median(sides))


def find_offset(p, samples, limit=25.0):
    need_scipy("--beam-offset auto")
    n = samples or guess_samples(np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi), np.hypot(p[:, 0], p[:, 1]))
    z = np.unique(p[:, 2])
    zs = z[len(z) // 4: max(len(z) // 4 + 1, 3 * len(z) // 4)][::max(1, len(z) // 16)]
    p = p[np.isin(p[:, 2], zs)]
    coarse = [(e, *square_score(p, e, n, zs)) for e in np.arange(-limit, limit + 0.01, 1.0)]
    e0 = min(coarse, key=lambda t: t[1])[0]
    fine = [(e, *square_score(p, e, n, zs)) for e in np.arange(e0 - 1, e0 + 1.01, 0.1)]
    e, err, size = min(fine, key=lambda t: t[1])
    if not np.isfinite(err):
        raise ValueError("Khong du diem de tim do lech tia. Vat co phai khoi vuong dat giua ban khong?")
    return round(float(e), 1), err, size


def build_grid(p, samples, layer, offset=0.0):
    theta = np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi)
    samples = samples or guess_samples(theta, np.hypot(p[:, 0], p[:, 1]))
    z = p[:, 2]
    if layer is None:
        steps = np.diff(np.unique(np.round(z, 3)))
        if not len(steps):
            raise ValueError("Can it nhat 2 lop Z")
        layer = float(steps.min())
    z0 = z.min()
    zi = np.rint((z - z0) / layer).astype(int)
    if np.abs(z - z0 - zi * layer).max() > 0.1 * layer:
        raise ValueError("Toa do Z khong deu theo buoc --layer")
    layers = zi.max() + 1
    ratio = np.minimum(np.bincount(zi, minlength=layers) / samples, 1.0)
    if offset:
        p = unoffset(p, offset, samples)
        theta = np.mod(np.arctan2(p[:, 1], p[:, 0]), 2 * np.pi)
        zi = np.rint((p[:, 2] - z0) / layer).astype(int)
    ai = np.rint(theta / (2 * np.pi / samples)).astype(int) % samples
    cells = zi * samples + ai
    count = np.bincount(cells, minlength=layers * samples)
    total = np.bincount(cells, np.hypot(p[:, 0], p[:, 1]), minlength=layers * samples)
    with np.errstate(invalid="ignore"):
        grid = (total / count).reshape(-1, samples)
    return grid, z0 + np.arange(layers) * layer, samples, layer, ratio


def drop_grazing(grid, max_deg, offset=0.0):
    if max_deg <= 0:
        return grid, 0
    n = grid.shape[1]
    k = max(1, round(n / 50))
    t = np.arange(n) * 2 * np.pi / n
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        r = np.nanmedian(np.stack([np.roll(grid, 1, 1), grid, np.roll(grid, -1, 1)]), 0)
    r = np.where(np.isfinite(grid), r, np.nan)
    tx = np.roll(r, -k, 1) * np.cos(np.roll(t, -k)) - np.roll(r, k, 1) * np.cos(np.roll(t, k))
    ty = np.roll(r, -k, 1) * np.sin(np.roll(t, -k)) - np.roll(r, k, 1) * np.sin(np.roll(t, k))
    with np.errstate(invalid="ignore", divide="ignore"):
        b = t - np.arctan2(offset, np.sqrt(np.maximum(r ** 2 - offset ** 2, 0)))
        sin_inc = np.abs(tx * np.cos(b) + ty * np.sin(b)) / np.hypot(tx, ty)
    bad = np.isfinite(grid) & (sin_inc > np.sin(np.radians(max_deg)))
    out = grid.copy()
    out[bad] = np.nan
    return out, int(bad.sum())


def fill(grid, z, ratio, min_ratio=0.9, max_gap_deg=36.0):
    n = grid.shape[1]
    ok = np.isfinite(grid)

    def largest_gap(row):
        idx = np.flatnonzero(row)
        return n if not len(idx) else (np.diff(np.r_[idx, idx[0] + n]) - 1).max()

    good = (ratio >= min_ratio) & (np.array([largest_gap(r) for r in ok]) <= n * max_gap_deg / 360)
    if good.sum() < 2:
        raise ValueError(f"Can it nhat 2 lop co >= {min_ratio:.0%} diem. Vat co nam giua ban xoay khong? (thu giam --min-diem)")
    first, last = np.flatnonzero(good)[[0, -1]]
    trimmed = int(first + len(grid) - 1 - last)
    grid, z, ok, good = grid[first:last + 1].copy(), z[first:last + 1], ok[first:last + 1], good[first:last + 1]
    grid[~good] = np.nan
    ok = ok & good[:, None]
    replaced = int((~good).sum())
    a = np.arange(n)
    for j in np.flatnonzero(good):
        miss = ~ok[j]
        grid[j, miss] = np.interp(a[miss], a[ok[j]], grid[j, ok[j]], period=n)
    for i in range(n):
        miss = np.isnan(grid[:, i])
        grid[miss, i] = np.interp(z[miss], z[good], grid[good, i])
    return grid, z, int((~ok[good]).sum()), trimmed, replaced


def smooth(r, passes, edge=10.0):
    if passes <= 0 or len(r) < 3:
        return r
    med = np.median(np.stack([r[:-2], r[1:-1], r[2:]]), 0)
    dev = r[1:-1] - med
    sigma = 1.4826 * np.median(np.abs(dev))
    r = r.copy()
    r[1:-1] = np.where(np.abs(dev) > max(3 * sigma, 0.5), med, r[1:-1])
    base, tol = r.copy(), max(4 * sigma, 0.8)
    for _ in range(passes):
        pad = np.vstack([2 * r[:1] - r[1:2], r, 2 * r[-1:] - r[-2:-1]])
        nbs = [np.roll(r, 1, 1), np.roll(r, -1, 1), pad[:-2], pad[2:]]
        w = [np.exp(-((nb - r) / edge) ** 2) for nb in nbs]
        r = r + 0.5 * sum(wi * (nb - r) for wi, nb in zip(w, nbs)) / sum(w)
        r = np.clip(r, base - tol, base + tol)
    return np.maximum(r, 0.2)


def add_base(r, z, base):
    if base <= 0:
        return r, z
    return np.vstack([r[:1], r]), np.r_[0.0, z + base]


def mesh(r, z):
    layers, n = r.shape
    t = np.arange(n) * 2 * np.pi / n
    v = np.column_stack([(r * np.cos(t)).ravel(), (r * np.sin(t)).ravel(), np.repeat(z, n)])
    v = np.vstack([v, [0, 0, z[0]], [0, 0, z[-1]]])
    j, i = np.meshgrid(np.arange(layers - 1), np.arange(n), indexing="ij")
    a, b = j * n + i, j * n + (i + 1) % n
    k = np.arange(n)
    bottom, top = len(v) - 2, len(v) - 1
    f = np.vstack([
        np.stack([a, b, b + n], -1).reshape(-1, 3),
        np.stack([a, b + n, a + n], -1).reshape(-1, 3),
        np.column_stack([np.full(n, bottom), (k + 1) % n, k]),
        np.column_stack([np.full(n, top), (layers - 1) * n + k, (layers - 1) * n + (k + 1) % n]),
    ])
    tri = v[f]
    volume = np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6
    if volume < 0:
        tri, volume = tri[:, [0, 2, 1]], -volume
    return tri, volume


def write_stl(path, tri):
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tri), [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = normal, tri
    with open(path, "wb") as fh:
        fh.write(b"scanner STL, mm".ljust(80, b"\0") + np.uint32(len(tri)).tobytes() + rec.tobytes())


def main():
    ap = argparse.ArgumentParser(description="Dung STL tu file x,y,z cua may quet")
    ap.add_argument("input", nargs="?", default="scan_received.txt")
    ap.add_argument("-o", "--output")
    ap.add_argument("--samples", type=int, help="so goc/vong (mac dinh tu nhan)")
    ap.add_argument("--layer", type=float, help="buoc Z mm (mac dinh tu nhan)")
    ap.add_argument("--smooth", type=int, default=12, help="so lan lam min, 0 = tat")
    ap.add_argument("--mirror", action="store_true", help="lat guong neu mo hinh nguoc trai/phai")
    ap.add_argument("--beam-offset", default="14.1",
                    help="mm tia do (mat phat) lech ngang khoi truc ban; 'auto' = tu tim tu ban quet khoi vuong")
    ap.add_argument("--min-diem", type=float, default=90,
                    help="%% so diem toi thieu cua 1 lop (vd 90 = 90/100); lop it hon bi bo, nap dat o lop du diem cuoi")
    ap.add_argument("--max-incidence", type=float, default=60,
                    help="bo diem ma tia gan nhu luot qua mat vat (goc toi > so do nay); 0 = tat")
    ap.add_argument("--zmin", type=float, help="bo cac lop co z nho hon (mm, theo z trong file quet)")
    ap.add_argument("--zmax", type=float, help="bo cac lop co z lon hon (mm, theo z trong file quet)")
    ap.add_argument("--z-day", type=float, default=0,
                    help="mm tu mat ban len tia luc z=0; keo lop duoi cung xuong toi mat ban")
    a = ap.parse_args()
    try:
        if not 0 < a.min_diem <= 100:
            raise ValueError("--min-diem phai trong khoang 1..100")
        if a.z_day < 0:
            raise ValueError("--z-day phai >= 0")
        p = load(a.input)
        if a.beam_offset.strip().lower() == "auto":
            a.beam_offset, err, size = find_offset(p, a.samples)
            print(f"Tu tim do lech tia: {a.beam_offset:g} mm (lech so voi hinh vuong {err:.2f} mm, canh {size:.1f} mm)")
            if err > 1.5:
                print("Canh bao: sai so lon. Kiem tra vat co vuong va dan cung loai giay")
        else:
            try:
                a.beam_offset = float(a.beam_offset)
            except ValueError:
                raise ValueError("--beam-offset phai la so mm hoac 'auto'")
        grid, z, samples, layer, ratio = build_grid(p, a.samples, a.layer, a.beam_offset)
        z_all = z
        cut = (z >= (-np.inf if a.zmin is None else a.zmin - 1e-6)) & (z <= (np.inf if a.zmax is None else a.zmax + 1e-6))
        if cut.sum() < 2:
            raise ValueError("--zmin/--zmax cat het lop, con duoi 2 lop")
        grid, z, ratio = grid[cut], z[cut], ratio[cut]
        grid, grazing = drop_grazing(grid, a.max_incidence, a.beam_offset)
        grid, z, filled, trimmed, replaced = fill(grid, z, ratio, a.min_diem / 100)
        kept = (z[0], z[-1])
        r, z = add_base(smooth(grid, a.smooth), z, a.z_day)
        tri, _ = mesh(r, z)
        if a.mirror:
            tri = tri[:, [0, 2, 1]] * [1, -1, 1]
        out = a.output or str(Path(a.input).with_suffix(".stl"))
        write_stl(out, tri)
    except (OSError, ValueError) as e:
        print(f"Loi: {e}", file=sys.stderr)
        return 1
    parts = []
    if (~cut).any():
        lo = "" if a.zmin is None else f"{a.zmin:g}"
        hi = "" if a.zmax is None else f"{a.zmax:g}"
        parts.append(f"{int((~cut).sum())} lop ngoai z {lo}..{hi} mm")
    if trimmed:
        parts.append(f"{trimmed} lop day/dinh < {a.min_diem:g}% diem")
    print(f"Cat: {', '.join(parts) if parts else 'khong'} -> giu z {kept[0]:g} -> {kept[1]:g} mm (quet {z_all[0]:g} -> {z_all[-1]:g})")
    how = [f"{filled} o thieu trong lop theo goc"]
    if replaced:
        how.append(f"{replaced} lop giua thieu diem lay tu lop tren/duoi")
    if grazing:
        how.append(f"bo {grazing} diem tia luot")
    if a.smooth > 0:
        how.append(f"lam min {a.smooth} lan")
    if a.z_day:
        how.append(f"keo day xuong mat ban {a.z_day:g} mm")
    print(f"Noi suy: {', '.join(how)}")
    size = np.ptp(tri.reshape(-1, 3), 0)
    print(f"Kich thuoc: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm")
    print(f"STL: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
