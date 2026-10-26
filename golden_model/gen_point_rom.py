# ==========================================
# Generador de verilog/ecc_point_rom.v
# Tabla de puntos impares 1G, 3G, ..., 63G de secp256k1 en coordenadas
# AFINES (Z = 1) y dominio Montgomery (x*R mod p, y*R mod p), R = 2^256.
# Uso: python golden_model/gen_point_rom.py
# ==========================================
import os

P = 2**256 - 2**32 - 977
R = 2**256
G = (0x79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798,
     0x483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8)
WINDOW_SIZE = 6
NUM_POINTS = 1 << (WINDOW_SIZE - 1)  # 32


def point_add(a, b):
    if a[0] == b[0]:
        lam = 3 * a[0] * a[0] * pow(2 * a[1], P - 2, P) % P
    else:
        lam = (b[1] - a[1]) * pow(b[0] - a[0], P - 2, P) % P
    x = (lam * lam - a[0] - b[0]) % P
    return (x, (lam * (a[0] - x) - a[1]) % P)


def main():
    g2 = point_add(G, G)
    points = [G]
    for _ in range(NUM_POINTS - 1):
        points.append(point_add(points[-1], g2))  # (2i+1)G = (2i-1)G + 2G

    for x, y in points:
        assert (y * y - x * x * x - 7) % P == 0

    lines = []
    for i, (x, y) in enumerate(points):
        xm, ym = x * R % P, y * R % P
        lines.append(f"            5'd{i:<2d}: begin dout_x <= 256'h{xm:064X}; "
                     f"dout_y <= 256'h{ym:064X}; end // {2 * i + 1}G")

    src = f"""// ============================================================================
// Módulo: ecc_point_rom.v (GENERADO por golden_model/gen_point_rom.py; no editar)
// Descripción: Tabla constante de puntos impares 1G..63G de secp256k1 para la
//              ventana deslizante (w = {WINDOW_SIZE}). Dirección i -> (2i+1)G.
//              Coordenadas AFINES (Z = 1, que en Montgomery es R mod p) y en
//              dominio Montgomery (x*R mod p, y*R mod p).
// Lectura síncrona: dout es válido el ciclo siguiente a fijar addr.
// Optimizado para: Intel Cyclone IV (inferencia de ROM en M9K)
// ============================================================================

module ecc_point_rom #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = {WINDOW_SIZE}
)(
    input  wire                       clk,
    input  wire [WINDOW_SIZE-2:0]     addr,
    (* romstyle = "M9K" *) output reg [WIDTH-1:0] dout_x,
    (* romstyle = "M9K" *) output reg [WIDTH-1:0] dout_y
);

    always @(posedge clk) begin
        case (addr)
{chr(10).join(lines)}
            default: begin dout_x <= {{WIDTH{{1'b0}}}}; dout_y <= {{WIDTH{{1'b0}}}}; end
        endcase
    end

endmodule
"""
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "verilog", "ecc_point_rom.v")
    with open(out, "w", encoding="utf-8", newline="\r\n") as f:
        f.write(src)
    print("escrito", os.path.normpath(out))


if __name__ == "__main__":
    main()
