import argparse
import socket
import time


def receive(conn, out, link_timeout):
    f, scan_id, n, buf, last = None, None, 0, b"", time.monotonic()
    conn.settimeout(1.0)
    try:
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                if time.monotonic() - last > link_timeout:
                    print(f"{link_timeout:g} s khong nhan gi tu ESP32 -> mat ket noi.")
                    return False
                continue
            if not data:
                print("ESP32 dong ket noi.")
                return False
            last = time.monotonic()
            *lines, buf = (buf + data).split(b"\n")
            for raw in lines:
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                head = line.split(",")
                if head[0] == "START":
                    if f:
                        f.close()
                    scan_id, n, f = head[1], 0, open(out, "w")
                    print(f"Bat dau quet {scan_id}")
                elif head[0] == "END":
                    ok = f is not None and head[1] == scan_id and int(head[2]) == n
                    if f:
                        f.close()
                        f = None
                    conn.sendall(f"{'ACK' if ok else 'NACK'},{scan_id},{n}\n".encode())
                    if ok and n > 0:
                        print(f"Xong: {n} diem -> {out}")
                    elif ok:
                        print(f"ESP32 khong thay vat (0 diem). File {out} rong.")
                    else:
                        print(f"Loi: ESP32 bao {head[2]} diem, PC nhan {n}. Cho ESP32 gui lai.")
                    return ok
                elif f:
                    try:
                        x, y, z = map(float, head)
                    except ValueError:
                        print(f"Bo dong loi: {line[:60]}")
                        continue
                    f.write(line + "\n")
                    n += 1
                    if n % 1000 == 0:
                        print(f"  {n} diem")
            if f:
                f.flush()
    finally:
        if f:
            f.close()
            print(f"Lan quet chua xong: {out} moi co {n} diem.")


def main():
    ap = argparse.ArgumentParser(description="Nhan diem quet tu ESP32 qua WiFi")
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("-o", "--output", default="scan_received.txt")
    ap.add_argument("--link-timeout", type=float, default=10.0)
    ap.add_argument("--once", action="store_true", help="thoat sau lan quet dau tien thanh cong")
    a = ap.parse_args()
    print(f"Ket noi ESP32 {a.host}:{a.port}, luu vao {a.output} (Ctrl+C de dung)")
    waiting = False
    try:
        while True:
            try:
                conn = socket.create_connection((a.host, a.port), timeout=3)
            except OSError:
                if not waiting:
                    print("Chua ket noi duoc ESP32 (PC da vao WiFi ScannerESP32 chua?). Tu thu lai moi 2 s...")
                    waiting = True
                time.sleep(2)
                continue
            waiting = False
            print("Da ket noi. ESP32 gui tung lop khi quet xong lop do...")
            with conn:
                try:
                    done = receive(conn, a.output, a.link_timeout)
                except OSError as e:
                    print(f"Loi ket noi: {e}")
                    done = False
            if done and a.once:
                return
            if done:
                print("Bat lai ESP32 de quet lan moi; script tu ket noi lai.")
                waiting = True
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nDa dung.")


if __name__ == "__main__":
    main()
