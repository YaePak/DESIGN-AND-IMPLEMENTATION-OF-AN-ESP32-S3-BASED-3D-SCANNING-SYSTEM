import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

MAX_GRID_CELLS = 1000000
MAX_FILE_BYTES = 64 * 1024 * 1024


def load_xyz(path):
    path = Path(path)
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError("File XYZ vuot gioi han 64 MiB")
    raw = path.read_bytes()
    points = []
    for number, line in enumerate(raw.decode("utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        fields = line.split(",")
        if len(fields) != 3:
            raise ValueError(f"Dong {number}: can dung 3 cot x,y,z")
        try:
            values = [float(field.strip()) for field in fields]
        except ValueError as error:
            raise ValueError(f"Dong {number}: toa do khong phai so") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Dong {number}: toa do chua NaN hoac Inf")
        points.append(values)
    if len(points) < 2:
        raise ValueError("Khong du diem de dung be mat 3D")
    return np.asarray(points, dtype=np.float64), hashlib.sha256(raw).hexdigest()


def read_metadata(path, point_count, digest, allow_incomplete):
    path = Path(path)
    sidecar = path.with_name(path.stem + ".meta.json")
    if not sidecar.exists():
        return {}, ["Khong co metadata; chua xac nhan du lieu truyen day du."]
    metadata = json.loads(sidecar.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("Metadata phai la mot JSON object")
    if metadata.get("status") == "receiving":
        raise ValueError("Receiver dang ghi file. Hay ket thuc nhan truoc khi dung STL")
    problems = []
    if metadata.get("status") not in ("legacy_unverified", "verified_transport"):
        problems.append(f"Trang thai lan nhan: {metadata.get('status', 'unknown')}")
    if metadata.get("points_saved") != point_count:
        problems.append("So diem trong metadata khong khop file XYZ")
    if metadata.get("sha256") != digest:
        problems.append("SHA-256 khong khop; file XYZ hoac metadata da thay doi")
    if metadata.get("units") != "mm":
        problems.append("Don vi trong metadata khong phai mm")
    if metadata.get("invalid_lines", 0):
        problems.append("Receiver da loai bo dong khong hop le")
    if problems and not allow_incomplete:
        raise ValueError("; ".join(problems) + ". Chi dung --allow-incomplete khi chu dong cuu du lieu")
    warnings = list(problems)
    if metadata.get("status") == "legacy_unverified":
        warnings.append("Firmware cu khong gui END; chua xac nhan du so diem cua ca lan quet.")
    return metadata, warnings


def select_parameters(args, metadata):
    header_samples = metadata.get("samples")
    header_layer = metadata.get("layer_mm")
    if header_samples is not None:
        if not isinstance(header_samples, int) or not 3 <= header_samples <= 10000:
            raise ValueError("samples trong metadata khong hop le")
    if header_layer is not None:
        if not isinstance(header_layer, (int, float)) or not math.isfinite(header_layer) or header_layer <= 0:
            raise ValueError("layer_mm trong metadata khong hop le")
    if args.samples is not None and header_samples is not None and args.samples != header_samples:
        raise ValueError("--samples khong khop metadata tu firmware")
    if args.layer is not None and header_layer is not None and not math.isclose(args.layer, header_layer, rel_tol=1e-9):
        raise ValueError("--layer khong khop metadata tu firmware")
    args.samples = args.samples if args.samples is not None else (header_samples if header_samples is not None else 80)
    args.layer = args.layer if args.layer is not None else (header_layer if header_layer is not None else 1.0)
    if not 3 <= args.samples <= 10000:
        raise ValueError("--samples phai tu 3 den 10000")
    if not math.isfinite(args.layer) or args.layer <= 0:
        raise ValueError("--layer phai lon hon 0 va huu han")
    if not math.isfinite(args.min_coverage) or not 0 < args.min_coverage <= 1:
        raise ValueError("--min-coverage phai nam trong (0,1]")
    if not 0 <= args.max_angle_gap < args.samples:
        raise ValueError("--max-angle-gap phai >= 0 va nho hon --samples")
    if not 0 <= args.max_missing_layers <= 10000:
        raise ValueError("--max-missing-layers phai tu 0 den 10000")
    if args.smooth is not None and not 0 <= args.smooth <= 100:
        raise ValueError("--smooth phai tu 0 den 100")
    if not 0 <= args.subdivide <= 3:
        raise ValueError("--subdivide phai tu 0 den 3")
    if args.edge is not None and not (math.isfinite(args.edge) and args.edge > 0):
        raise ValueError("--edge phai lon hon 0")
    args.z_tolerance = 0.1 * args.layer if args.z_tolerance is None else args.z_tolerance
    if not math.isfinite(args.z_tolerance) or not 0 <= args.z_tolerance < 0.5 * args.layer:
        raise ValueError("--z-tolerance phai >= 0 va nho hon nua buoc Z")


def build_grid(points, samples, layer_mm, z_tolerance):
    x, y, z = points.T
    radius = np.hypot(x, y)
    if not np.isfinite(radius).all() or np.any(radius <= 1e-6):
        raise ValueError("Ban kinh phai huu han va lon hon 0 (diem nam dung tam da bi loc truoc do)")
    z0 = float(z.min())
    span = float(z.max() - z0)
    layer_span = span / layer_mm
    if not math.isfinite(layer_span) or layer_span + 1 > MAX_GRID_CELLS / samples:
        raise ValueError("Luoi qua lon. Kiem tra don vi, --layer va du lieu Z")
    zi = np.rint((z - z0) / layer_mm).astype(np.int64)
    n_layers = int(zi.max()) + 1
    if n_layers < 2:
        raise ValueError("Can it nhat 2 lop Z khac nhau de dung be mat 3D")
    z_error = np.abs(z - (z0 + zi * layer_mm))
    if np.any(z_error > z_tolerance + 1e-8):
        raise ValueError("Toa do Z khong khop --layer. Kiem tra buoc quet va don vi")
    theta = np.mod(np.arctan2(y, x), 2 * np.pi)
    d_angle = 2 * np.pi / samples
    ai = np.rint(theta / d_angle).astype(np.int64) % samples
    angle_error = np.abs((theta - ai * d_angle + np.pi) % (2 * np.pi) - np.pi)
    if np.any(angle_error > 0.2 * d_angle + 1e-8):
        raise ValueError("Goc diem khong khop --samples hoac goc goc 0 cua firmware")
    flat = zi * samples + ai
    counts = np.bincount(flat, minlength=n_layers * samples).reshape(n_layers, samples)
    sums = np.bincount(flat, weights=radius, minlength=n_layers * samples).reshape(n_layers, samples)
    grid = np.full((n_layers, samples), np.nan)
    measured = counts > 0
    grid[measured] = sums[measured] / counts[measured]
    z_levels = z0 + np.arange(n_layers) * layer_mm
    z_counts = np.bincount(zi, minlength=n_layers)
    z_sums = np.bincount(zi, weights=z, minlength=n_layers)
    present = z_counts > 0
    z_levels[present] = z_sums[present] / z_counts[present]
    if np.any(np.diff(z_levels) <= 0):
        raise ValueError("Thu tu cac lop Z khong hop le")
    return grid, z_levels, {
        "input_points": int(len(points)),
        "measured_cells": int(measured.sum()),
        "duplicate_points_averaged": int(len(points) - measured.sum()),
        "max_z_alignment_error_mm": float(z_error.max()),
        "max_angle_alignment_error_degrees": float(np.rad2deg(angle_error.max())),
    }


def true_runs(mask):
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def layer_quality(row, min_coverage, max_angle_gap):
    """(so goc do duoc, ti le, so goc thieu lien tiep lon nhat, lop co du tot khong)."""
    samples = len(row)
    valid = np.flatnonzero(np.isfinite(row))
    if not len(valid):
        return 0, 0.0, samples, False
    largest_gap = int((np.diff(np.r_[valid, valid[0] + samples]) - 1).max())
    coverage = len(valid) / samples
    good = len(valid) >= 3 and coverage + 1e-12 >= min_coverage and largest_gap <= max_angle_gap
    return len(valid), coverage, largest_gap, good


def fill_missing(grid, z_levels, min_coverage, max_angle_gap, max_missing_layers, strict):
    """
    Bu cac o thieu cho VAT BAT KY (khong gia dinh hinh dang), tam vong = truc quay:
      - Lop "tot" (du goc, khe thieu nho): noi suy theo goc ngay trong lop.
      - Lop "yeu"/trong o giua: o thieu lay theo Z tu lop tot gan nhat ben duoi va
        ben tren; diem do that trong lop van giu nguyen.
      - Lop "yeu" o dinh/day vat (thuong la phan lech khoi truc quay, vd dinh dau vit)
        de nguyen o day; reconstruct_edges() dung lai chung theo tam rieng.
    --strict: thay vi bu, bao loi va dung (de kiem tra chat luong lan quet).
    """
    result = grid.copy()
    n_layers, samples = result.shape
    quality = [layer_quality(row, min_coverage, max_angle_gap) for row in result]
    counts = np.array([q[0] for q in quality])
    coverage = counts / samples
    good = np.array([q[3] for q in quality])
    warnings = []

    if strict:
        for j, (count, cov, gap, ok) in enumerate(quality):
            if count and not ok:
                if count < 3 or cov + 1e-12 < min_coverage:
                    raise ValueError(
                        f"Lop Z={z_levels[j]:g} mm chi co {count}/{samples} goc do that; can it nhat 3 goc va {min_coverage:.0%}"
                    )
                raise ValueError(f"Lop Z={z_levels[j]:g} mm thieu {gap} goc lien tiep; gioi han {max_angle_gap}")
        for start, stop in true_runs(counts == 0):
            if stop - start > max_missing_layers:
                raise ValueError(
                    f"Thieu {stop-start} lop Z lien tiep tu {z_levels[start]:g} mm; gioi han --max-missing-layers={max_missing_layers}"
                )

    if int(good.sum()) < 2:
        raise ValueError(
            "Can it nhat 2 lop do du goc de dung be mat. Kiem tra vat co dat giua ban xoay, "
            "nguong strength va --samples/--layer co khop firmware khong."
        )

    first = int(np.argmax(good))                      # lop tot thap nhat
    last = n_layers - 1 - int(np.argmax(good[::-1]))  # lop tot cao nhat

    # 1) Lop tot: noi suy theo goc trong cung lop
    angle_filled = 0
    for j in np.flatnonzero(good):
        row = result[j]
        absent = ~np.isfinite(row)
        if absent.any():
            valid = np.flatnonzero(~absent)
            row[absent] = np.interp(np.flatnonzero(absent), valid, row[valid], period=samples)
            angle_filled += int(absent.sum())

    # 2) Lop yeu / trong o giua (giua lop tot thap nhat va cao nhat):
    #    noi suy theo Z tu 2 lop tot gan nhat, giu diem do that
    vertical_filled = 0
    weak_z = []
    long_runs = []
    inner = np.zeros(n_layers, dtype=bool)
    inner[first:last + 1] = ~good[first:last + 1]
    for start, stop in true_runs(inner):
        below, above = start - 1, stop
        fraction = (z_levels[start:stop] - z_levels[below]) / (z_levels[above] - z_levels[below])
        guess = result[below] + fraction[:, None] * (result[above] - result[below])
        block = result[start:stop]
        absent = ~np.isfinite(block)
        block[absent] = guess[absent]
        vertical_filled += int(absent.sum())
        weak_z.extend(z_levels[start:stop].tolist())
        if stop - start > max_missing_layers:
            long_runs.append(f"{z_levels[start]:g}..{z_levels[stop-1]:g} mm")
    if weak_z:
        warnings.append(f"{len(weak_z)} lop thieu goc o giua duoc bu bang noi suy theo Z tu lop tren/duoi.")
    if long_runs:
        warnings.append(
            f"Vung thieu dai hon {max_missing_layers} lop ({', '.join(long_runs)}): hinh o do chi la uoc luong."
        )

    measured = counts > 0
    return result, first, last, warnings, {
        "coverage_per_layer": coverage.tolist(),
        "minimum_measured_layer_coverage": float((counts[measured] / samples).min()),
        "angle_cells_interpolated": angle_filled,
        "vertical_cells_interpolated": vertical_filled,
        "whole_layers_interpolated": int((~measured).sum()),
        "interpolated_layer_z_mm": weak_z,
        "total_interpolated_cells": int(angle_filled + vertical_filled),
    }


MIN_RADIUS_MM = 0.2


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def fit_center(points):
    """Tam cua mot lat cat tu cac diem do: khop duong tron (vom, dau, nap chai...);
    neu cung do qua ngan hoac khop xau thi dung trong tam cac diem."""
    centroid = points.mean(axis=0)
    if len(points) >= 5:
        x, y = points.T
        A = np.column_stack([x, y, np.ones_like(x)])
        try:
            (D, E, F), *_ = np.linalg.lstsq(A, -(x * x + y * y), rcond=None)
        except np.linalg.LinAlgError:
            return centroid
        center = np.array([-D / 2.0, -E / 2.0])
        r2 = float(center @ center - F)
        if np.isfinite(r2) and r2 > 0:
            d = points - center
            ang = np.sort(np.arctan2(d[:, 1], d[:, 0]))
            largest_gap = np.diff(np.r_[ang, ang[0] + 2 * np.pi]).max()
            if 2 * np.pi - largest_gap >= np.deg2rad(120) and np.linalg.norm(center - centroid) < 2 * np.sqrt(r2):
                return center
    return centroid


def ring_from_points(points, center, samples):
    """Vong ban kinh quanh `center` tu cac diem do; mat khuat (phia truc quay) noi suy theo goc."""
    d = points - center
    phi = np.mod(np.arctan2(d[:, 1], d[:, 0]), 2 * np.pi)
    rho = np.hypot(d[:, 0], d[:, 1])
    k = np.rint(phi / (2 * np.pi / samples)).astype(np.int64) % samples
    sums = np.bincount(k, weights=rho, minlength=samples)
    cnt = np.bincount(k, minlength=samples)
    valid = np.flatnonzero(cnt > 0)
    if len(valid) < 2:
        return np.full(samples, max(float(rho.mean()), MIN_RADIUS_MM))
    ring = sums[valid] / cnt[valid]
    return np.maximum(np.interp(np.arange(samples), valid, ring, period=samples), MIN_RADIUS_MM)


def regrid_ring(ring, center, samples):
    """Do lai mot vong (tam = truc quay) theo tam moi: ban kinh = giao diem xa nhat cua tia tu tam moi."""
    theta = np.arange(samples) * (2 * np.pi / samples)
    poly = np.column_stack([ring * np.cos(theta), ring * np.sin(theta)])
    edge = np.roll(poly, -1, axis=0) - poly
    ray = np.column_stack([np.cos(theta), np.sin(theta)])
    w = poly - center
    denom = _cross(ray[:, None, :], edge[None, :, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = _cross(w[None, :, :], edge[None, :, :]) / denom
        s = _cross(w[None, :, :], ray[:, None, :]) / denom
    ok = np.isfinite(t) & (t > 1e-9) & (s >= -1e-9) & (s <= 1 + 1e-9)
    out = np.where(ok, t, -np.inf).max(axis=1)
    bad = ~np.isfinite(out)
    if bad.all():
        return np.maximum(ring, MIN_RADIUS_MM)
    if bad.any():
        good = np.flatnonzero(~bad)
        out[bad] = np.interp(np.flatnonzero(bad), good, out[good], period=samples)
    return np.maximum(out, MIN_RADIUS_MM)


def reconstruct_edges(grid, z_levels, first, last, layer_mm):
    """
    Dung lai cac lop yeu o DINH (va DAY) vat - thuong la phan nho nam lech khoi truc quay
    (dinh dau vit, nap, num...). Cam bien chi thay mat ngoai cua phan do, khong thay mat
    quay vao truc. Cach lam:
      1) Moi lop: tim tam rieng cua lat cat tu diem do (khop duong tron) va dung vong
         quanh tam do; phan mat khuat noi suy theo goc quanh tam -> lat cat tron, khong
         bi keo ve truc quay (het lom/nhon).
      2) Vai lop tot ngay ben duoi duoc doi tam dan tu truc quay sang tam moi
         -> chuyen tiep tron, khong gay xoan.
      3) Dinh vom duoc khep them theo toc do thu nho cua cac lop tren cung.
    Tra ve ban kinh, tam tung lop, z, do cao khep dinh, canh bao, thong ke.
    """
    n, samples = grid.shape
    theta = np.arange(samples) * (2 * np.pi / samples)
    rho = grid.copy()
    centers = np.zeros((n, 2))
    keep = np.ones(n, dtype=bool)
    apex_dz = 0.0
    rebuilt = {"top": 0, "bottom": 0}
    both = last < n - 1 and first > 0
    span = last - first

    for top in (True, False):
        weak = list(range(last + 1, n)) if top else list(range(first - 1, -1, -1))
        anchor, step = (last, 1) if top else (first, -1)
        rings = []
        for j in weak:
            valid = np.isfinite(grid[j])
            if valid.sum() < 3:
                keep[j] = False
                continue
            pts = np.column_stack([grid[j][valid] * np.cos(theta[valid]), grid[j][valid] * np.sin(theta[valid])])
            rings.append((j, fit_center(pts), pts))
        dropped = [j for j in weak if not keep[j]]
        if not rings:
            continue
        rebuilt["top" if top else "bottom"] = len(rings)

        C = np.array([c for _, c, _ in rings])
        if len(C) >= 3:
            smoothed = C.copy()
            smoothed[1:-1] = (C[:-2] + 2 * C[1:-1] + C[2:]) / 4.0
            C = smoothed
        for (j, _, pts), c in zip(rings, C):
            rho[j] = ring_from_points(pts, c, samples)
            centers[j] = c

        # Chuyen tam dan tu truc quay (lop tot) sang tam cua lop yeu dau tien
        target = C[0]
        dist = float(np.linalg.norm(target))
        blend = int(min(span // 2 if both else span, max(3, np.ceil(dist / layer_mm))))
        if dist > 1e-6 and blend > 0:
            direction = np.arctan2(target[1], target[0])
            for i in range(blend):
                j = anchor - step * i
                t = (blend - i) / (blend + 1.0)
                reach = float(np.interp(direction % (2 * np.pi), theta, grid[j], period=2 * np.pi))
                length = min(t * dist, 0.7 * reach)
                c = target / dist * length
                rho[j] = regrid_ring(grid[j], c, samples)
                centers[j] = c

        if top:
            # Khep dinh vom: ngoai suy ban kinh^2 cua 2 lop tren cung ve 0
            (ja, _, _), (jb, _, _) = (rings[-2], rings[-1]) if len(rings) >= 2 else (rings[-1], rings[-1])
            ra, rb = float(rho[ja].mean()), float(rho[jb].mean())
            if jb != ja and rb < ra:
                slope = (rb * rb - ra * ra) / (z_levels[jb] - z_levels[ja])
                apex_dz = -rb * rb / slope
            if dropped:
                apex_dz = max(apex_dz, float(max(z_levels[dropped]) - z_levels[jb]))
            apex_dz = float(np.clip(apex_dz, 0.0, 2.0 * layer_mm))

    warnings = []
    if rebuilt["top"] or rebuilt["bottom"]:
        warnings.append(
            f"{rebuilt['top']} lop dinh, {rebuilt['bottom']} lop day nam lech truc quay: dung lai quanh tam rieng, "
            f"mat khuat duoc noi suy; dinh vom khep them {apex_dz:.1f} mm."
        )
    stats = {
        "edge_layers_rebuilt_top": rebuilt["top"],
        "edge_layers_rebuilt_bottom": rebuilt["bottom"],
        "edge_layers_dropped_too_few_points": int((~keep).sum()),
        "top_apex_extra_mm": apex_dz,
    }
    return rho[keep], centers[keep], z_levels[keep], apex_dz, warnings, stats


def _z_neighbours(r):
    """Lop ben tren / ben duoi cua moi lop; o lop dau va cuoi dung lop ao ngoai suy tuyen tinh
    (de lop bien cung duoc lam muot ma khong bi keo vao/phinh ra tao ranh hay go)."""
    up, down = np.empty_like(r), np.empty_like(r)
    up[:-1], down[1:] = r[1:], r[:-1]
    up[-1] = 2 * r[-1] - r[-2]
    down[0] = 2 * r[0] - r[1]
    return up, down


def smooth_grid(grid, centers, z_levels, passes, edge_mm=10.0, tolerance_mm=None):
    """
    Lam min be mat (passes = so lan, 0 = tat):
      1) Loc gai: o nao lech han so voi trung vi 3 lop lien tiep (vuot 3 lan muc nhieu
         chung, toi thieu 0.5 mm) thi thay bang trung vi.
      2) Lam muot giu canh: moi lan, moi o tien ve trung binh cac o ke ben (theo goc va
         theo Z), nhung chi tinh cac o ke ben chenh lech nho (nhieu, gon song). O ke ben
         chenh lech lon hon --edge (canh that: vai, co, mep mo) gan nhu khong anh huong.
      3) Gioi han sai lech: moi o chi duoc dich toi da `tolerance_mm` so voi diem do
         (mac dinh ~4 lan muc nhieu do duoc, toi thieu 0.8 mm) -> be mat min trong pham vi
         nhieu, khong lam hut mui mo/duoi/dinh, goc khoi vuong.
    """
    result = grid.copy()
    centers = centers.copy()
    if passes <= 0 or len(z_levels) < 3:
        return result, centers
    median = np.median(np.stack([grid[:-2], grid[1:-1], grid[2:]]), axis=0)
    deviation = grid[1:-1] - median
    sigma = 1.4826 * float(np.median(np.abs(deviation)))
    spike = np.abs(deviation) > max(3.0 * sigma, 0.5)
    result[1:-1][spike] = median[spike]
    edge = float(edge_mm)
    tolerance = max(4.0 * sigma, 0.8) if tolerance_mm is None else float(tolerance_mm)
    # Moc gioi han = diem do sau khi lam muot rat nhe theo Z -> moc tron doc canh dung
    # (khong de lai rang cua o goc khoi vuong khi bi gioi han).
    base = result.copy()
    for _ in range(2):
        up, down = _z_neighbours(base)
        base = (down + 2 * base + up) / 4.0

    def laplacian(r):
        up, down = _z_neighbours(r)
        acc = np.zeros_like(r)
        wsum = np.zeros_like(r)
        for nb in (np.roll(r, 1, axis=1), np.roll(r, -1, axis=1), up, down):
            diff = nb - r
            w = np.exp(-(diff / edge) ** 2)
            acc += w * diff
            wsum += w
        return acc / np.maximum(wsum, 1e-9)

    for _ in range(passes):
        result = result + 0.5 * laplacian(result)
        result = np.clip(result, base - tolerance, base + tolerance)
    for _ in range(passes):
        up, down = _z_neighbours(centers)
        centers = (down + 2 * centers + up) / 4.0
    return np.maximum(result, MIN_RADIUS_MM), centers


def add_dome_cap(grid, centers, z_levels, apex_dz):
    """Khep dinh vom bang vai vong nho dan (ban kinh^2 giam deu theo Z, nhu dinh qua cau)
    thay vi mot nap phang -> dinh tron, khong co bac hay ranh."""
    if apex_dz <= 0:
        return grid, centers, z_levels, z_levels[-1]
    t = np.array([0.3, 0.55, 0.75, 0.9])
    rings = grid[-1][None, :] * np.sqrt(1.0 - t)[:, None]
    grid = np.vstack([grid, np.maximum(rings, MIN_RADIUS_MM)])
    centers = np.vstack([centers, np.repeat(centers[-1:], len(t), axis=0)])
    apex_z = z_levels[-1] + apex_dz
    z_levels = np.concatenate([z_levels, z_levels[-1] + apex_dz * t])
    return grid, centers, z_levels, apex_z


def _four_point(values):
    """Diem giua theo so do noi suy 4 diem (-1, 9, 9, -1)/16 doc truc 0, hai dau ke lap."""
    pad = np.concatenate([values[:1], values, values[-1:]])
    return (-pad[:-3] + 9 * pad[1:-2] + 9 * pad[2:-1] - pad[3:]) / 16.0


def subdivide_grid(grid, centers, z_levels, rounds):
    """
    Chia nho luoi (moi lan: gap doi so goc va so lop) bang so do noi suy 4 diem:
    be mat cong deu hon, it mat phang/goc canh hon khi xem STL. Diem goc giu nguyen.
    """
    for _ in range(rounds):
        r = grid
        mid = (-np.roll(r, 1, axis=1) + 9 * r + 9 * np.roll(r, -1, axis=1) - np.roll(r, -2, axis=1)) / 16.0
        ring = np.empty((r.shape[0], 2 * r.shape[1]))
        ring[:, 0::2], ring[:, 1::2] = r, mid
        n = ring.shape[0]
        grid = np.empty((2 * n - 1, ring.shape[1]))
        grid[0::2], grid[1::2] = ring, _four_point(ring)
        new_centers = np.empty((2 * n - 1, 2))
        new_centers[0::2], new_centers[1::2] = centers, _four_point(centers)
        centers = new_centers
        levels = np.empty(2 * n - 1)
        levels[0::2], levels[1::2] = z_levels, (z_levels[:-1] + z_levels[1:]) / 2.0
        z_levels = levels
    return np.maximum(grid, MIN_RADIUS_MM), centers, z_levels


def grid_to_mesh(grid, z_levels, caps, centers=None, apex_dz=0.0):
    n_layers, samples = grid.shape
    if centers is None:
        centers = np.zeros((n_layers, 2))
    theta = np.arange(samples) * (2 * np.pi / samples)
    vertices = np.column_stack([
        (centers[:, :1] + grid * np.cos(theta)).ravel(),
        (centers[:, 1:] + grid * np.sin(theta)).ravel(),
        np.repeat(z_levels, samples),
    ])
    j = np.arange(n_layers - 1)[:, None]
    i = np.arange(samples)[None, :]
    a = j * samples + i
    b = j * samples + (i + 1) % samples
    c = (j + 1) * samples + i
    d = (j + 1) * samples + (i + 1) % samples
    faces = [np.stack([a, b, d], axis=-1).reshape(-1, 3), np.stack([a, d, c], axis=-1).reshape(-1, 3)]
    if caps in ("bottom", "both"):
        center = len(vertices)
        vertices = np.vstack([vertices, [centers[0, 0], centers[0, 1], z_levels[0]]])
        k = np.arange(samples)
        faces.append(np.column_stack([np.full(samples, center), (k + 1) % samples, k]))
    if caps in ("top", "both"):
        center = len(vertices)
        vertices = np.vstack([vertices, [centers[-1, 0], centers[-1, 1], z_levels[-1] + apex_dz]])
        base = (n_layers - 1) * samples
        k = np.arange(samples)
        faces.append(np.column_stack([np.full(samples, center), base + k, base + (k + 1) % samples]))
    return vertices, np.concatenate(faces).astype(np.int64)


def validate_mesh(vertices, faces, samples, caps):
    if not np.isfinite(vertices).all() or np.any(np.abs(vertices) > np.finfo(np.float32).max):
        raise ValueError("Dinh luoi khong bieu dien duoc trong STL float32")
    vertices = vertices.astype(np.float32).astype(np.float64)
    triangles = vertices[faces]
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    if not np.isfinite(areas).all() or np.any(areas <= 0):
        raise ValueError("Luoi co tam giac suy bien sau khi doi sang float32")
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    keys = np.sort(edges, axis=1)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    if np.any(counts > 2):
        raise ValueError("Luoi co canh thuoc hon 2 tam giac")
    directions = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    balances = np.bincount(inverse, weights=directions)
    if np.any(balances[counts == 2] != 0):
        raise ValueError("Huong tam giac khong nhat quan")
    boundary_edges = int((counts == 1).sum())
    expected = samples * (2 - int(caps in ("bottom", "both")) - int(caps in ("top", "both")))
    if boundary_edges != expected:
        raise ValueError("So canh bien khong khop cau truc luoi")
    volume = None
    if boundary_edges == 0:
        volume = float(np.einsum("ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6)
        if not math.isfinite(volume) or volume == 0:
            raise ValueError("Luoi kin co the tich khong hop le")
        if volume < 0:
            faces = faces[:, [0, 2, 1]]
            volume = -volume
    return vertices, faces, {
        "vertices": int(len(vertices)),
        "triangles": int(len(faces)),
        "boundary_edges": boundary_edges,
        "closed_mesh": boundary_edges == 0,
        "volume_mm3": volume,
        "bounds_min_mm": vertices.min(axis=0).tolist(),
        "bounds_max_mm": vertices.max(axis=0).tolist(),
    }


def write_stl_binary(path, vertices, faces, overwrite):
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    records = np.zeros(len(faces), dtype=dtype)
    records["normal"] = normals
    records["vertices"] = triangles
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb" if overwrite else "xb") as handle:
        handle.write(b"XYZ radial surface; coordinate convention: mm".ljust(80, b"\0"))
        handle.write(struct.pack("<I", len(faces)))
        handle.write(records.tobytes())


def build_stl(points, args, warnings):
    if args.mirror:
        # Neu mo hinh ra nguoc trai/phai so voi vat that (do chieu quay ban xoay) -> lat truc Y
        points = points * np.array([1.0, -1.0, 1.0])
        warnings.append("Da lat guong theo truc Y (--mirror).")
    # Diem nam ngay tam quay khong co goc xac dinh -> bo, khong lam hong ca lan dung
    near_axis = np.hypot(points[:, 0], points[:, 1]) < 0.5
    if near_axis.any():
        points = points[~near_axis]
        warnings.append(f"Bo {int(near_axis.sum())} diem nam sat tam quay (ban kinh < 0.5 mm).")
    # TF-Luna o che do cm: ban kinh luon la boi so 10 mm -> can lam min manh hon, nguong canh lon hon
    radii = np.hypot(points[:, 0], points[:, 1])
    cm_data = len(radii) > 50 and float(np.mean(np.abs(radii / 10.0 - np.rint(radii / 10.0)) < 0.03)) > 0.9
    if args.smooth is None:
        args.smooth = 20 if cm_data else 8
    if args.edge is None:
        args.edge = 15.0 if cm_data else 10.0
    tolerance = 12.0 if cm_data else None      # None = tu tinh theo muc nhieu cua du lieu
    if cm_data:
        warnings.append(
            f"Du lieu nhay bac 10 mm (TF-Luna don vi cm): tu dong lam min manh hon (--smooth {args.smooth}, --edge {args.edge:g})."
        )
    grid, z_levels, input_stats = build_grid(points, args.samples, args.layer, args.z_tolerance)
    grid, first, last, fill_warnings, fill_stats = fill_missing(
        grid, z_levels, args.min_coverage, args.max_angle_gap, args.max_missing_layers, args.strict
    )
    warnings.extend(fill_warnings)
    grid, centers, z_levels, apex_dz, edge_warnings, edge_stats = reconstruct_edges(
        grid, z_levels, first, last, args.layer
    )
    warnings.extend(edge_warnings)
    measured_z = z_levels
    if args.smooth:
        grid, centers = smooth_grid(grid, centers, z_levels, args.smooth, args.edge, tolerance)
    apex_z = z_levels[-1]
    if args.caps in ("top", "both"):
        grid, centers, z_levels, apex_z = add_dome_cap(grid, centers, z_levels, apex_dz)
    if args.subdivide:
        grid, centers, z_levels = subdivide_grid(grid, centers, z_levels, args.subdivide)
    if args.caps != "none":
        warnings.append("Nap tren/duoi duoc tao them (cam bien khong do mat tren/duoi cua vat).")
    if fill_stats["total_interpolated_cells"]:
        warnings.append("Mot phan be mat duoc noi suy; xem so luong trong bao cao.")
    vertices, faces = grid_to_mesh(grid, z_levels, args.caps, centers, apex_z - z_levels[-1])
    vertices, faces, mesh_stats = validate_mesh(vertices, faces, grid.shape[1], args.caps)
    return vertices, faces, {
        "units": "mm",
        "samples": args.samples,
        "nominal_layer_mm": args.layer,
        "z_levels_mm": measured_z.tolist(),
        "mesh_ring_points": int(grid.shape[1]),
        "mesh_layers": int(grid.shape[0]),
        "smooth_passes": args.smooth,
        "edge_threshold_mm": args.edge,
        "cm_quantized_input": cm_data,
        "subdivide_rounds": args.subdivide,
        "caps": args.caps,
        "min_coverage_required": args.min_coverage,
        "max_angle_gap_allowed": args.max_angle_gap,
        "max_missing_layers_allowed": args.max_missing_layers,
        "strict": args.strict,
        "mirror": args.mirror,
        **input_stats,
        **fill_stats,
        **edge_stats,
        **mesh_stats,
        "warnings": warnings,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Dung STL tu XYZ theo luoi goc va Z")
    parser.add_argument("input", nargs="?", default="scan_received.txt")
    parser.add_argument("-o", "--output")
    parser.add_argument("--samples", type=int)
    parser.add_argument("--layer", type=float)
    parser.add_argument("--smooth", type=int, default=None,
                        help="so lan lam min be mat, 0..100 (mac dinh tu chon: 8; du lieu TF-Luna cm: 20). "
                             "0 = tat. So cang lon cang min")
    parser.add_argument("--subdivide", type=int, default=1,
                        help="so lan chia nho luoi, 0..3 (mac dinh 1). Moi lan tang 4 lan so tam giac, "
                             "be mat cong deu hon")
    parser.add_argument("--edge", type=float, default=None,
                        help="(nang cao) chenh lech ban kinh lon hon so mm nay duoc coi la canh that, "
                             "khong bi lam muot (mac dinh 10; du lieu TF-Luna cm: 15)")
    parser.add_argument("--min-coverage", type=float, default=0.50,
                        help="lop 'tot' phai do duoc it nhat ti le goc nay (mac dinh 0.5)")
    parser.add_argument("--max-angle-gap", type=int, default=8,
                        help="lop 'tot' thieu toi da bay nhieu goc lien tiep (mac dinh 8 = 36 do)")
    parser.add_argument("--max-missing-layers", type=int, default=2,
                        help="vung lop thieu dai hon so nay se duoc canh bao (mac dinh 2)")
    parser.add_argument("--z-tolerance", type=float)
    parser.add_argument("--caps", choices=["none", "bottom", "top", "both"], default="both",
                        help="nap phang tren/duoi (mac dinh both = luoi kin). Cam bien chi do mat ben, "
                             "khong do duoc mat tren/duoi nen nap luon la mat phang")
    parser.add_argument("--strict", action="store_true",
                        help="lop thieu goc thi bao loi va dung, khong tu bu (de kiem tra chat luong quet)")
    parser.add_argument("--mirror", action="store_true",
                        help="lat guong truc Y neu mo hinh ra nguoc trai/phai so voi vat that")
    parser.add_argument("--allow-incomplete", action="store_true")
    parser.add_argument("--no-overwrite", dest="overwrite", action="store_false",
                        help="khong ghi de STL cu (mac dinh: ghi de)")
    args = parser.parse_args(argv)
    try:
        source = Path(args.input).expanduser().resolve()
        output = Path(args.output).expanduser().resolve() if args.output else source.with_suffix(".stl")
        report_path = output.with_name(output.name + ".report.json")
        if source == output or source == report_path:
            raise ValueError("Duong dan dau ra trung file XYZ dau vao")
        if not args.overwrite and (output.exists() or report_path.exists()):
            raise ValueError("Dau ra da ton tai. Chon ten khac hoac bo --no-overwrite")
        points, digest = load_xyz(source)
        metadata, warnings = read_metadata(source, len(points), digest, args.allow_incomplete)
        select_parameters(args, metadata)
        vertices, faces, report = build_stl(points, args, warnings)
        report["source_file"] = str(source)
        report["source_sha256"] = digest
        report["source_status"] = metadata.get("status", "no_metadata")
        report["incomplete_override"] = bool(args.allow_incomplete)
        write_stl_binary(output, vertices, faces, args.overwrite)
        with report_path.open("w" if args.overwrite else "x", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
        print(f"{len(points)} diem do -> {len(report['z_levels_mm'])} lop x {args.samples} goc")
        print(f"Noi suy {report['total_interpolated_cells']} o, gom {report['whole_layers_interpolated']} lop trong")
        print(f"Lam min {args.smooth} lan, chia nho luoi {args.subdivide} lan -> "
              f"luoi {report['mesh_layers']} lop x {report['mesh_ring_points']} diem/vong")
        print(f"{report['vertices']} dinh, {report['triangles']} tam giac; canh bien: {report['boundary_edges']}")
        low, high = report["bounds_min_mm"], report["bounds_max_mm"]
        print(f"Kich thuoc: X={high[0]-low[0]:.3f}, Y={high[1]-low[1]:.3f}, Z={high[2]-low[2]:.3f} mm")
        if report["closed_mesh"]:
            print(f"Luoi kin theo kiem tra canh; the tich luoi ~ {report['volume_mm3']/1000:.3f} cm3")
        else:
            print("Luoi ho theo tuy chon caps; khong tinh the tich.")
        for warning in warnings:
            print(f"Luu y: {warning}")
        print(f"STL: {output}")
        print(f"Bao cao: {report_path}")
        return 0
    except (OSError, ValueError, OverflowError) as error:
        print(f"Loi: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())