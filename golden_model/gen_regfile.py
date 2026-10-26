# ==========================================
# Generador de verilog/ecc_regfile.v
# Banco de registros de 256 bits (M9K) del procesador ECC, inicializado con:
#   0..63 : tabla de puntos impares 1G, 3G, ..., 63G de secp256k1 en
#           coordenadas AFINES y dominio Montgomery (x*R mod p, y*R mod p).
#           Dirección 2i -> x de (2i+1)G, dirección 2i+1 -> y de (2i+1)G.
#   64    : 0
#   65    : 1 en dominio Montgomery (R mod p)
#   66    : 1 literal (para salir del dominio Montgomery)
# El resto (Q, temporales) arranca en 0. Las direcciones deben coincidir con
# las de verilog/jacobian_point_op.v.
# Uso: python golden_model/gen_regfile.py
# ==========================================
import os

P = 2**256 - 2**32 - 977
R = 2**256
G = (0x79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798,
     0x483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8)
WINDOW_SIZE = 6
NUM_POINTS = 1 << (WINDOW_SIZE - 1)  # 32
DEPTH = 128
RF_ZERO, RF_ONE, RF_LIT1 = 64, 65, 66


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

    init = []
    for i, (x, y) in enumerate(points):
        init.append(f"        mem[{2 * i:3d}] = 256'h{x * R % P:064X}; // {2 * i + 1}G.x")
        init.append(f"        mem[{2 * i + 1:3d}] = 256'h{y * R % P:064X}; // {2 * i + 1}G.y")
    init.append(f"        mem[{RF_ZERO:3d}] = 256'h{0:064X}; // 0")
    init.append(f"        mem[{RF_ONE:3d}] = 256'h{R % P:064X}; // 1 Montgomery (R mod p)")
    init.append(f"        mem[{RF_LIT1:3d}] = 256'h{1:064X}; // 1 literal")

    src = f"""// ============================================================================
// Módulo: ecc_regfile.v (GENERADO por golden_model/gen_regfile.py; no editar)
// Descripción: Banco de registros de {DEPTH} x 256 bits del procesador ECC, en M9K
//              (RAM de doble puerto simple: 1 escritura, 1 lectura).
//              Contenido inicial (cargado al configurar la FPGA):
//                0..63 : tabla 1G..63G afín en dominio Montgomery
//                        (2i -> x de (2i+1)G, 2i+1 -> y de (2i+1)G)
//                64    : 0
//                65    : 1 en Montgomery (R mod p)
//                66    : 1 literal
//              La tabla y las constantes nunca se escriben.
// Lectura síncrona: q es válido el ciclo siguiente a fijar raddr.
// Optimizado para: Intel Cyclone IV (8 M9K en modo 256 x 32)
// ============================================================================

module ecc_regfile #(
    parameter WIDTH = 256,
    parameter AW    = 7
)(
    input  wire              clk,
    input  wire              we,
    input  wire [AW-1:0]     waddr,
    input  wire [WIDTH-1:0]  wdata,
    input  wire [AW-1:0]     raddr,
    output reg  [WIDTH-1:0]  q
);

    (* ramstyle = "M9K, no_rw_check" *) reg [WIDTH-1:0] mem [0:(1<<AW)-1];

    integer i;
    initial begin
        for (i = 0; i < (1<<AW); i = i + 1) mem[i] = {{WIDTH{{1'b0}}}};
{chr(10).join(init)}
    end

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        q <= mem[raddr];
    end

endmodule
"""
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "verilog", "ecc_regfile.v")
    with open(out, "w", encoding="utf-8", newline="\r\n") as f:
        f.write(src)
    print("escrito", os.path.normpath(out))


if __name__ == "__main__":
    main()
