#!/usr/bin/env python3
# ============================================================================
# fpga_keygen_tester.py
# Prueba el generador de llaves secp256k1 de la FPGA (fpga_uart_top) por UART
# y compara cada llave pública contra librerías estándar de Python.
#
# Protocolo (ver verilog/fpga_uart_top.v), 115200 baudios 8N1:
#   PC  -> FPGA : 32 bytes de k, MSB primero
#   FPGA -> PC  : 64 bytes = Qx (32 B, MSB primero) || Qy (32 B, MSB primero)
#   k = 0 o k = n devuelven (0, 0) (punto en el infinito); k > n devuelve (k mod n)*G.
#
# Referencias soportadas (se usan todas las instaladas y se comparan entre sí):
#   ecdsa, cryptography, coincurve
#
# Uso:
#   python host/fpga_keygen_tester.py --list
#   python host/fpga_keygen_tester.py --port COM5
#   python host/fpga_keygen_tester.py --port COM5 --count 200 --csv resultados.csv
#   python host/fpga_keygen_tester.py --port COM5 --key 0x1 --key 18E14A7B6A30...
# ============================================================================

import argparse
import csv
import secrets
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Falta pyserial: pip install -r host/requirements.txt")

# Parámetros de secp256k1
P = 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F
N = 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEBAAEDCE6AF48A03BBFD25E8CD0364141
GX = 0x79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798
GY = 0x483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8

K_BYTES = 32
Q_BYTES = 64
INFINITY = (0, 0)  # Así representa la FPGA el punto en el infinito


# ----------------------------------------------------------------------------
# Referencias: librerías estándar de secp256k1
# ----------------------------------------------------------------------------
def _ref_ecdsa(k):
    from ecdsa import SECP256k1, SigningKey
    pt = SigningKey.from_secret_exponent(k, curve=SECP256k1).get_verifying_key().pubkey.point
    return (pt.x(), pt.y())


def _ref_cryptography(k):
    from cryptography.hazmat.primitives.asymmetric import ec
    nums = ec.derive_private_key(k, ec.SECP256K1()).public_key().public_numbers()
    return (nums.x, nums.y)


def _ref_coincurve(k):
    from coincurve import PrivateKey
    raw = PrivateKey(k.to_bytes(32, "big")).public_key.format(compressed=False)
    return (int.from_bytes(raw[1:33], "big"), int.from_bytes(raw[33:65], "big"))


def load_references():
    refs = []
    for name, fn, module in (("ecdsa", _ref_ecdsa, "ecdsa"),
                             ("cryptography", _ref_cryptography, "cryptography"),
                             ("coincurve", _ref_coincurve, "coincurve")):
        try:
            __import__(module)
            refs.append((name, fn))
        except ImportError:
            pass
    if not refs:
        sys.exit("No hay ninguna librería de referencia. Instala al menos una:\n"
                 "  pip install -r host/requirements.txt")
    return refs


def expected_public_key(k, refs):
    """Llave pública esperada según las librerías; (0, 0) si k = 0 mod n."""
    k_eff = k % N
    if k_eff == 0:
        return INFINITY
    results = {name: fn(k_eff) for name, fn in refs}
    values = set(results.values())
    if len(values) != 1:
        raise RuntimeError(f"Las librerías de referencia no coinciden para k={k:#x}: {results}")
    return values.pop()


# ----------------------------------------------------------------------------
# Comunicación con la FPGA
# ----------------------------------------------------------------------------
class FpgaLink:
    def __init__(self, port, baud, timeout):
        self.ser = serial.serial_for_url(port, baudrate=baud, bytesize=8, parity="N",
                                         stopbits=1, timeout=timeout)
        self.timeout = timeout

    def close(self):
        self.ser.close()

    def keygen(self, k):
        """Envía k y devuelve ((Qx, Qy), segundos). Lanza TimeoutError si no responde."""
        self.ser.reset_input_buffer()
        t0 = time.perf_counter()
        self.ser.write(k.to_bytes(K_BYTES, "big"))
        self.ser.flush()
        data = self.ser.read(Q_BYTES)
        dt = time.perf_counter() - t0
        if len(data) != Q_BYTES:
            raise TimeoutError(f"la FPGA devolvió {len(data)} de {Q_BYTES} bytes")
        return (int.from_bytes(data[:32], "big"), int.from_bytes(data[32:], "big")), dt

    def resync(self):
        """Recupera la alineación de bytes si la FPGA quedó esperando parte de un k.

        La FPGA no tiene delimitador de trama: si recibió solo parte de un k (ruido al
        encender, un byte perdido), espera los bytes que faltan. Se envían ceros de a
        uno hasta que responde, y esa respuesta se descarta.
        """
        self.ser.reset_input_buffer()
        old = self.ser.timeout
        self.ser.timeout = 0.25
        try:
            for sent in range(1, K_BYTES + 1):
                self.ser.write(b"\x00")
                self.ser.flush()
                first = self.ser.read(1)
                if first:
                    self.ser.timeout = self.timeout
                    rest = self.ser.read(Q_BYTES - 1)
                    self.ser.reset_input_buffer()
                    return sent, len(rest) + 1 == Q_BYTES
            return None, False
        finally:
            self.ser.timeout = old


# ----------------------------------------------------------------------------
# Vectores de prueba
# ----------------------------------------------------------------------------
KNOWN_VECTORS = [
    ("conocido", 0x1),
    ("conocido", 0x2),
    ("conocido", 0x3),
    ("conocido", 0x18E14A7B6A307F426A94F8114701E7C8E774E7F9A47E2C2035DB29A206321725),
    ("borde", N - 1),
    ("borde", 0x0),             # infinito
    ("borde", N),               # infinito
    ("borde", N + 1),           # = 1*G
    ("borde", (1 << 256) - 1),  # > n
    ("borde", 1 << 255),
    ("ventana", 0x3F),          # una ventana de 6 bits con todos los puntos impares
    ("ventana", int("1" + "0" * 249 + "100001", 2)),  # 256 bits, ventana final 33G
]


def build_vectors(args):
    vectors = []
    for text in args.key or []:
        vectors.append(("manual", int(text, 16)))
    if not args.no_fixed:
        vectors.extend(KNOWN_VECTORS)
    for _ in range(args.count):
        vectors.append(("aleatorio", 1 + secrets.randbelow(N - 1)))
    return vectors


# ----------------------------------------------------------------------------
# Diagnóstico de fallas
# ----------------------------------------------------------------------------
def on_curve(pt):
    x, y = pt
    return (y * y - x * x * x - 7) % P == 0


def diagnose(got, exp):
    if got == INFINITY:
        return "la FPGA devolvió el infinito (0, 0)"
    hints = []
    if exp != INFINITY:
        if got == (exp[0], (-exp[1]) % P):
            hints.append("es -Q (y negada)")
        if got == (exp[1], exp[0]):
            hints.append("x e y intercambiadas")
        rev = lambda v: int.from_bytes(v.to_bytes(32, "big")[::-1], "big")
        if got == (rev(exp[0]), rev(exp[1])):
            hints.append("bytes en orden inverso")
    if not hints:
        hints.append("punto válido de la curva, pero otro múltiplo de G" if on_curve(got)
                     else "el punto NO está en la curva")
    return "; ".join(hints)


# ----------------------------------------------------------------------------
# Programa principal
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Prueba el generador de llaves secp256k1 de la FPGA por UART.")
    ap.add_argument("--port", help="puerto serie (COM5, /dev/ttyUSB0) o URL de pyserial")
    ap.add_argument("--baud", type=int, default=115200, help="baudios (por defecto 115200)")
    ap.add_argument("--timeout", type=float, default=2.0, help="segundos de espera por llave (por defecto 2)")
    ap.add_argument("--count", type=int, default=20, help="cantidad de llaves aleatorias (por defecto 20)")
    ap.add_argument("--key", action="append", help="k en hexadecimal (se puede repetir)")
    ap.add_argument("--no-fixed", action="store_true", help="omitir los vectores conocidos y de borde")
    ap.add_argument("--no-resync", action="store_true", help="no intentar resincronizar al inicio")
    ap.add_argument("--stop-on-fail", action="store_true", help="detenerse en la primera falla")
    ap.add_argument("--csv", help="guardar los resultados en un archivo CSV")
    ap.add_argument("--list", action="store_true", help="listar los puertos serie y salir")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:12s} {p.description}")
        return 0
    if not args.port:
        ap.error("falta --port (usa --list para ver los puertos)")

    refs = load_references()
    print(f"Referencias: {', '.join(name for name, _ in refs)}")
    assert expected_public_key(1, refs) == (GX, GY), "la referencia no devuelve G para k = 1"

    vectors = build_vectors(args)
    link = FpgaLink(args.port, args.baud, args.timeout)
    rows, n_ok, n_fail = [], 0, 0
    try:
        if not args.no_resync:
            # Si la FPGA responde bien a k = 1, ya está alineada
            try:
                got, _ = link.keygen(1)
                aligned = got == (GX, GY)
            except TimeoutError:
                aligned = False
            if not aligned:
                sent, ok = link.resync()
                if sent is None:
                    sys.exit("La FPGA no responde. Revisa el puerto, los baudios y presiona reset.")
                print(f"Resincronizado: faltaban {sent} byte(s) de una trama anterior")

        print(f"{'#':>4}  {'tipo':10s} {'k':20s} {'resultado':9s} {'ms':>7}")
        for i, (kind, k) in enumerate(vectors, 1):
            exp = expected_public_key(k, refs)
            k_txt = f"{k:064x}"
            k_short = k_txt[:8] + ".." + k_txt[-8:]
            try:
                got, dt = link.keygen(k)
            except TimeoutError as e:
                n_fail += 1
                print(f"{i:4d}  {kind:10s} {k_short:20s} {'TIMEOUT':9s} {'':>7}  {e}")
                rows.append([i, kind, k_txt, "TIMEOUT", "", "", f"{exp[0]:064x}", f"{exp[1]:064x}", "", str(e)])
                print("      Intentando resincronizar...")
                sent, _ = link.resync()
                if sent is None or args.stop_on_fail:
                    break
                continue

            ok = got == exp
            n_ok += ok
            n_fail += not ok
            print(f"{i:4d}  {kind:10s} {k_short:20s} {'OK' if ok else 'FALLA':9s} {dt * 1000:7.1f}")
            note = ""
            if not ok:
                note = diagnose(got, exp)
                print(f"      esperado x={exp[0]:064x}\n"
                      f"               y={exp[1]:064x}\n"
                      f"      FPGA     x={got[0]:064x}\n"
                      f"               y={got[1]:064x}\n"
                      f"      -> {note}")
            rows.append([i, kind, k_txt, "OK" if ok else "FALLA", f"{got[0]:064x}", f"{got[1]:064x}",
                         f"{exp[0]:064x}", f"{exp[1]:064x}", f"{dt * 1000:.1f}", note])
            if not ok and args.stop_on_fail:
                break
    except KeyboardInterrupt:
        print("\nInterrumpido.")
    finally:
        link.close()

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["n", "tipo", "k", "resultado", "fpga_x", "fpga_y", "ref_x", "ref_y", "ms", "diagnostico"])
            w.writerows(rows)
        print(f"Resultados guardados en {args.csv}")

    total = n_ok + n_fail
    print(f"\n{n_ok}/{total} llaves correctas" + (f", {n_fail} con falla" if n_fail else ""))
    return 0 if total and n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
