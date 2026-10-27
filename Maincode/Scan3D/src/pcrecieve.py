import argparse
import hashlib
import json
import math
import os
import re
import socket
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path


MAX_LINE_BYTES = 512


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def metadata_path(path):
    return path.with_name(path.stem + ".meta.json")


def write_json(path, value):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as handle:
            temporary = Path(handle.name)
            json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def output_path(requested):
    # Luon dung CUNG MOT file (mac dinh scan_received.txt): moi lan quet ghi de len lan truoc.
    path = Path(requested).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def parse_xyz(text):
    fields = text.split(",")
    if len(fields) != 3:
        raise ValueError("Moi diem phai co dung 3 cot x,y,z")
    values = tuple(float(item.strip()) for item in fields)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Toa do chua NaN hoac Inf")
    return values


class Capture:
    def __init__(self, path, peer):
        self.path = path
        self.handle = None          # chi mo (va xoa noi dung cu) khi nhan duoc diem dau tien
        self.peer = list(peer)
        self.started_at = utc_now()
        self.points = 0
        self.invalid_lines = 0
        self.errors = []
        self.mode = "legacy_xyz"
        self.scan_id = None
        self.samples = None
        self.layer_mm = None
        self.expected_points = None
        self.end_received = False
        self.digest = hashlib.sha256()

    def reject(self, message):
        self.invalid_lines += 1
        if len(self.errors) < 10:
            self.errors.append(str(message)[:200])
            print(f"Bo dong khong hop le: {message}")

    def consume(self, raw):
        try:
            if len(raw) > MAX_LINE_BYTES:
                raise ValueError("Dong du lieu qua dai")
            text = raw.decode("ascii").strip()
            if not text:
                return False
            fields = [part.strip() for part in text.split(",")]
            if fields[0] == "START":
                if self.scan_id is not None or self.points:
                    raise ValueError("START phai nam truoc tat ca cac diem")
                if len(fields) != 4:
                    raise ValueError("Can START,scan_id,samples,layer_mm")
                if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", fields[1]):
                    raise ValueError("scan_id khong hop le")
                samples = int(fields[2])
                layer = float(fields[3])
                if not 3 <= samples <= 10000 or not math.isfinite(layer) or layer <= 0:
                    raise ValueError("Thong so START khong hop le")
                self.mode = "framed_v1"
                self.scan_id = fields[1]
                self.samples = samples
                self.layer_mm = layer
                return False
            if fields[0] == "END":
                if len(fields) != 3 or self.scan_id is None:
                    raise ValueError("END can co START tuong ung")
                if fields[1] != self.scan_id:
                    raise ValueError("scan_id trong END khong khop START")
                expected = int(fields[2])
                if expected < 0:
                    raise ValueError("So diem trong END khong hop le")
                self.expected_points = expected
                self.end_received = True
                if expected != self.points:
                    self.reject(f"END bao {expected} diem, PC nhan {self.points} diem")
                return True
            values = parse_xyz(text)
        except (UnicodeError, ValueError, OverflowError) as error:
            self.reject(error)
            return False
        if self.handle is None:
            # Diem dau tien cua lan quet moi -> bay gio moi ghi de file cu.
            # (ESP32 ket noi ngay luc khoi dong nhung chua quet thi file cu van con nguyen.)
            self.handle = self.path.open("w", encoding="ascii", newline="\n")
            write_json(metadata_path(self.path), self.snapshot("receiving"))
            print(f"Bat dau nhan diem, ghi de: {self.path}")
        encoded = (",".join(repr(value) for value in values) + "\n").encode("ascii")
        self.handle.write(encoded.decode("ascii"))
        self.handle.flush()
        self.digest.update(encoded)
        self.points += 1
        if self.points % 50 == 0:
            print(f"Da luu {self.points} diem")
        return False

    def snapshot(self, status, reason=None):
        return {
            "format_version": 1,
            "file": self.path.name,
            "units": "mm",
            "status": status,
            "reason": reason,
            "protocol": self.mode,
            "scan_id": self.scan_id,
            "samples": self.samples,
            "layer_mm": self.layer_mm,
            "points_saved": self.points,
            "expected_points": self.expected_points,
            "invalid_lines": self.invalid_lines,
            "errors": self.errors,
            "sha256": self.digest.hexdigest(),
            "peer": self.peer,
            "started_at": self.started_at,
            "finished_at": None if status == "receiving" else utc_now(),
        }


def receive_connection(conn, peer, args):
    path = output_path(args.output)
    capture = Capture(path, peer)
    pending = bytearray()
    last_data = None
    clean_eof = False
    interrupted = False
    reason = None
    print(f"Ket noi tu {peer[0]}:{peer[1]}")
    print("Dang cho diem; firmware co the dang cho nut HOME hoac hieu chuan.")
    try:
        conn.settimeout(1.0)
        while not capture.end_received:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                if (
                    args.idle_timeout > 0
                    and last_data is not None
                    and time.monotonic() - last_data >= args.idle_timeout
                ):
                    reason = "idle_timeout"
                    break
                continue
            if not data:
                clean_eof = True
                break
            last_data = time.monotonic()
            pending.extend(data)
            while b"\n" in pending:
                raw, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                if capture.consume(bytes(raw)):
                    if pending.strip():
                        capture.reject("Co du lieu sau END trong cung goi nhan")
                    pending.clear()
                    break
            if len(pending) > MAX_LINE_BYTES:
                capture.reject("Dong chua ket thuc vuot gioi han 512 byte")
                reason = "oversized_line"
                pending.clear()
                break
    except KeyboardInterrupt:
        interrupted = True
        reason = "user_interrupted"
    except OSError as error:
        reason = f"io_error: {error}"
    finally:
        if pending.strip():
            capture.reject("Bo dong cuoi thieu ky tu xuong dong")
        if capture.handle is not None:
            try:
                capture.handle.flush()
                os.fsync(capture.handle.fileno())
            except OSError as error:
                reason = f"file_flush_error: {error}"
            finally:
                capture.handle.close()
    if capture.handle is None:
        # Ket noi dong ma chua co diem nao (vd: reset ESP32 truoc khi quet) -> khong dong vao file cu.
        print(f"Ket noi dong ma khong nhan duoc diem nao. Giu nguyen file cu: {path}")
        return {"status": "empty"}, interrupted
    if interrupted:
        status = "interrupted"
    elif reason or capture.invalid_lines:
        status = "incomplete"
    elif capture.points == 0:
        status = "empty"
    elif capture.end_received:
        status = "verified_transport"
    elif clean_eof and capture.mode == "legacy_xyz":
        status = "legacy_unverified"
    else:
        status = "incomplete"
        reason = "missing_END"
    report = capture.snapshot(status, reason)
    write_json(metadata_path(path), report)
    if capture.end_received:
        response = "ACK" if status == "verified_transport" else "NACK"
        try:
            conn.sendall(f"{response},{capture.scan_id},{capture.points}\n".encode("ascii"))
        except OSError:
            print("Da luu file, nhung khong gui duoc ACK/NACK ve ESP32.")
    print(f"Da luu {capture.points} diem. Trang thai: {status}")
    print(f"XYZ: {path}")
    print(f"Metadata: {metadata_path(path)}")
    if status == "legacy_unverified":
        print("Firmware khong gui END: chua xac nhan du so diem cua ca lan quet.")
    elif status not in ("verified_transport", "empty"):
        print("Lan nhan chua hoan chinh. Kiem tra metadata truoc khi dung STL.")
    return report, interrupted


def main(argv=None):
    parser = argparse.ArgumentParser(description="Nhan XYZ tu ESP32-S3 qua TCP")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("-o", "--output", default="scan_received.txt")
    parser.add_argument("--idle-timeout", type=float, default=300.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("Port phai tu 1 den 65535")
    if not math.isfinite(args.idle_timeout) or args.idle_timeout < 0:
        parser.error("idle-timeout phai >= 0; 0 la tat timeout")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(4)
            print(f"Dang nghe {args.host}:{args.port}. Nhan Ctrl+C de dung.")
            print(f"Moi lan quet se GHI DE len: {output_path(args.output)}")
            while True:
                conn, peer = server.accept()
                with conn:
                    report, interrupted = receive_connection(conn, peer, args)
                if interrupted:
                    return 130
                if args.once:
                    return 0 if report["status"] in ("legacy_unverified", "verified_transport") else 2
                print("Tiep tuc cho ket noi moi...")
    except KeyboardInterrupt:
        print("\nDa dung chuong trinh nhan.")
        return 130
    except OSError as error:
        print(f"Loi: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
